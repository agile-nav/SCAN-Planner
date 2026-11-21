#!/usr/bin/env python3
# ============================================================================
# 文件：tools/sim_check_probe.py —— 仿真闭环自检探针（被 tools/sim_check.sh 调用）
#
# 在仿真栈已经拉起后，检查：
#   ① 模型身份   ：/robot_description 的 <robot name=...> 与机型一致，12 个腿关节（Go2-W 另含 4 轮）
#   ② TF         ：world -> base 可查
#   ③ 规划闭环   ：收到 planning/bspline、状态进入 running、位移足够；
#                   场景有终点时检查到终点距离并等待 settle；有 obstacle_bbox(es)
#                   时检查最小间隙（阈值至少为机型圆柱半径 + 0.05 m）；
#                   有 min_yaw_change 时检查最终航向变化
#   ④ 关节动画   ：/joint_states 有 12 个腿关节（Go2-W 共 16）
#   ⑤ 碰撞几何   ：/self_inflation 的前后圆柱半径/偏移/高度与机型档案一致
#
# 用法：python3 sim_check_probe.py <机型名> <档案 yaml 路径>
# 退出码：0 全部通过，1 有失败项
# ============================================================================
import json
import math
import os
import re
import sys
import time
from collections import deque

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scan_planner_msgs.msg import Bspline
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "?"
PROFILE = sys.argv[2] if len(sys.argv) > 2 else ""

# 场景参数由 sim_check.sh 通过环境变量传入；缺省保持原来的烟测行为。
MOVE_TIMEOUT = float(os.environ.get("SCENARIO_TIMEOUT", "45.0"))
MIN_MOVE = float(os.environ.get("SCENARIO_MIN_MOVE", "1.0"))
GOAL_TOL = float(os.environ.get("SCENARIO_GOAL_TOL", "0.5"))
GOAL_SETTLE = float(os.environ.get("SCENARIO_GOAL_SETTLE", "1.0"))
_goal_x = os.environ.get("SCENARIO_GOAL_X", "")
_goal_y = os.environ.get("SCENARIO_GOAL_Y", "")
GOAL = (float(_goal_x), float(_goal_y)) if _goal_x and _goal_y else None
_bboxes = os.environ.get("SCENARIO_OBSTACLE_BBOXES", "")
OBSTACLE_BBOXES = []
if _bboxes:
    for item in _bboxes.split(";"):
        if item.strip():
            OBSTACLE_BBOXES.append(tuple(float(v) for v in item.split(",")))
if not OBSTACLE_BBOXES:
    _bbox = os.environ.get("SCENARIO_OBSTACLE_BBOX", "")
    if _bbox:
        OBSTACLE_BBOXES.append(tuple(float(v) for v in _bbox.split(",")))
MIN_CLEARANCE = float(os.environ.get("SCENARIO_MIN_CLEARANCE", "0.0"))
MIN_YAW_CHANGE = float(os.environ.get("SCENARIO_MIN_YAW_CHANGE", "0.0"))


def _opt_float(name):
    raw = os.environ.get(name, "")
    return float(raw) if raw not in ("", None) else None


# 降级场景：判定"已到达降级点"的距离阈值（FSM 的 status_goal_tol 是 0.20）
DEGRADE_ARRIVE_TOL = 0.35


def _read_degraded_point(log_path):
    """从运行日志里读最近一次降级点（world 系 xy）。读不到返回 None。"""
    if not log_path or not os.path.isfile(log_path):
        return None
    last = None
    try:
        with open(log_path, "r", errors="ignore") as fh:
            for line in fh:
                m = re.search(r"degraded to nearest reachable \[([-\d.]+), ([-\d.]+)", line)
                if m:
                    last = (float(m.group(1)), float(m.group(2)))
    except OSError:
        return None
    return last


# ---- 目标降级断言（仅当场景声明 expect_degrade 时才生效）----
EXPECT_DEGRADE = os.environ.get("SCENARIO_EXPECT_DEGRADE", "false").lower() == "true"
GOAL_TO_OBSTACLE = _opt_float("SCENARIO_GOAL_TO_OBSTACLE")
GOAL_DIST_MIN = _opt_float("SCENARIO_GOAL_DIST_MIN")
GOAL_DIST_MAX = _opt_float("SCENARIO_GOAL_DIST_MAX")
MAX_DEGRADE_COUNT = int(os.environ.get("SCENARIO_MAX_DEGRADE_COUNT", "1") or 1)
# 降级日志所在文件（由 sim_check.sh 传入），用于统计降级次数与连通性失败
SCAN_LOG = os.environ.get("SCENARIO_SCAN_LOG", "")


def bbox_distance(x, y, bbox):
    """点到矩形包围盒的二维距离；点在盒内返回 0。"""
    xmin, xmax, ymin, ymax = bbox
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


def path_turn(points):
    """累计路径转向角：相邻运动方向的变化量之和（弧度）。

    直线路径为 0；L 形 90° 路径约为 pi/2。
    这样不会把“最终航向回到初始值”的 L 形路径误判为没转弯。
    """
    total = 0.0
    prev_heading = None
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        dx, dy = x1 - x0, y1 - y0
        if math.hypot(dx, dy) < 0.005:
            continue
        heading = math.atan2(dy, dx)
        if prev_heading is not None:
            delta = math.atan2(math.sin(heading - prev_heading),
                               math.cos(heading - prev_heading))
            total += abs(delta)
        prev_heading = heading
    return total


def load_expectations(profile_path):
    """从机型档案读取碰撞几何的期望值。"""
    with open(profile_path, "r", encoding="utf-8") as handle:
        profile = yaml.safe_load(handle) or {}
    params = profile.get("scan_planner_node", {}).get("ros__parameters", {}) or {}
    return {
        "radius": float(params.get("grid_map.double_cylinder_radius", -1.0)),
        "offset": float(params.get("grid_map.double_cylinder_offset", -1.0)),
        "z_up": float(params.get("grid_map.obstacles_inflation_z_up", 0.0)),
        "z_down": float(params.get("grid_map.obstacles_inflation_z_down", 0.0)),
    }


class Probe(Node):
    def __init__(self):
        super().__init__("sim_check_probe")
        self.desc = None
        self.joints = None
        self.bsplines = 0
        self.states = []
        # self_inflation 每轮发布两条 marker（id=0 前 / id=1 后）。
        # 各留最近若干条，取“时间戳最接近”的一对来计算偏移，
        # 避免把不同时刻的前/后圆柱相减（机器人移动后会算出很大的偏移）。
        self.markers = {0: deque(maxlen=20), 1: deque(maxlen=20)}
        late = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description",
                                 lambda m: setattr(self, "desc", m.data), late)
        self.create_subscription(String, "/local_planner/status",
                                 lambda m: self.states.append(m.data), late)
        self.create_subscription(Bspline, "planning/bspline",
                                 self._on_bspline, 10)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "joints", m), 10)
        self.create_subscription(Marker, "/self_inflation", self._on_marker, late)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def _on_bspline(self, _msg):
        self.bsplines += 1

    def _on_marker(self, msg):
        self.markers.setdefault(msg.id, deque(maxlen=20)).append(msg)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def base_pose(self):
        try:
            t = self.tf.lookup_transform("world", "base", rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except Exception:
            return None

    def base_xy(self):
        pose = self.base_pose()
        return pose[:2] if pose else None

    def wait_for_base(self, timeout):
        """等 TF 就绪并返回第一帧 base 位姿 (x, y, yaw)。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.base_pose()
            if pose:
                return pose
        return None

    def saw_state(self, state):
        for raw in self.states:
            try:
                if json.loads(raw).get("state") == state:
                    return True
            except Exception:
                pass
        return False


def main():
    expect = load_expectations(PROFILE)
    rclpy.init()
    p = Probe()
    results = []

    # 先等 TF 就绪并记录初始位置：越早记录越好，避免机器人已经走完短路。
    p0 = p.wait_for_base(15.0)

    # ① 模型身份 + 12 个腿关节
    p.spin_for(6.0)
    ok1 = bool(p.desc) and "<robot" in p.desc
    model_name = ""
    if ok1:
        match = re.search(r'<robot[^>]*name="([^"]+)"', p.desc)
        model_name = match.group(1) if match else ""
    leg_joints = len(re.findall(r'name="[A-Z]{2}_(?:hip|thigh|calf)_joint"', p.desc or ""))
    wheel_joints = len(re.findall(
        r'<joint name="[A-Z]{2}_foot_joint" type="continuous"', p.desc or ""))
    expected_wheels = 4 if ROBOT == "go2w" else 0
    ok1 = (ok1 and model_name == ROBOT and leg_joints == 12
           and wheel_joints == expected_wheels)
    detail1 = f"name={model_name}, 腿关节={leg_joints}"
    if expected_wheels or wheel_joints:
        detail1 += f", 轮关节={wheel_joints}"
    results.append(("① 模型身份",
                    f"OK  {detail1}" if ok1
                    else f"FAIL {detail1}"))

    # ② TF world -> base
    results.append(("② TF world->base",
                    f"OK  base=({p0[0]:.2f}, {p0[1]:.2f})" if p0
                    else "FAIL 查不到 world->base"))

    # ③ 规划闭环：收到轨迹 + 状态 running + 实际位移；
    #    场景给了终点时，还要求最终到终点距离 <= 阈值。
    ok3 = False
    detail3 = "FAIL 无 TF，无法判断"
    if p0:
        deadline = time.time() + MOVE_TIMEOUT
        p1 = p0
        moved = 0.0
        goal_dist = None
        path_xy = []
        reached_since = None
        prev_pose = None
        while time.time() < deadline:
            rclpy.spin_once(p, timeout_sec=0.05)
            p1 = p.base_pose()
            if not p1:
                continue
            path_xy.append((p1[0], p1[1]))
            moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if GOAL is not None:
                goal_dist = math.hypot(p1[0] - GOAL[0], p1[1] - GOAL[1])
                # 降级场景不能用"距目标近"当结束条件：目标本身不可达，
                #   机器人要走到降级点才算完，而 goal_tol 可能比降级点还远，
                #   会在半路就误判"已到达"（实测 go2w 在 1.71 m 处就停了观察，
                #   降级根本没来得及发生）。改为"位置不再变化"的停稳判定。
                if EXPECT_DEGRADE:
                    # 降级场景：目标本身不可达，不能拿"距目标近"当结束条件。
                    #   判据改为「已经降级成功 且 到达了降级点」：
                    #     · 日志里出现 degraded 说明降级已发生；
                    #     · 之后距降级点的距离进入 status_goal_tol 量级即算到达。
                    #   若一直没降级，就让 MOVE_TIMEOUT 兜底（此时断言会失败，
                    #   这正是我们想要的信号：该降级却没降级）。
                    deg_pt = _read_degraded_point(SCAN_LOG)
                    if deg_pt is not None:
                        d_deg = math.hypot(p1[0] - deg_pt[0], p1[1] - deg_pt[1])
                        if d_deg <= DEGRADE_ARRIVE_TOL:
                            if reached_since is None:
                                reached_since = time.time()
                            if time.time() - reached_since >= GOAL_SETTLE:
                                break
                        else:
                            reached_since = None
                    prev_pose = p1
                elif goal_dist <= GOAL_TOL:
                    if reached_since is None:
                        reached_since = time.time()
                    # 到达阈值后再观察一小段时间，确认不是“冲过阈值又离开”
                    if time.time() - reached_since >= GOAL_SETTLE:
                        break
                else:
                    reached_since = None
            elif moved >= MIN_MOVE:
                break
        ok3 = p.bsplines > 0 and moved >= MIN_MOVE and p.saw_state("running")
        if GOAL is not None:
            ok3 = ok3 and goal_dist is not None and goal_dist <= GOAL_TOL
        clearance = None
        effective_min_clearance = MIN_CLEARANCE
        if expect["radius"] > 0.0:
            # 路径间隙至少覆盖机型圆柱半径，再加一点余量
            effective_min_clearance = max(
                effective_min_clearance, expect["radius"] + 0.05)
        if OBSTACLE_BBOXES:
            if path_xy:
                clearance = min(
                    bbox_distance(x, y, box)
                    for x, y in path_xy for box in OBSTACLE_BBOXES)
                ok3 = ok3 and clearance >= effective_min_clearance
            else:
                ok3 = False
        yaw_change = None
        if MIN_YAW_CHANGE > 0.0:
            # 路径转向角按相邻运动方向变化累计
            yaw_change = path_turn(path_xy)
            ok3 = ok3 and yaw_change >= MIN_YAW_CHANGE
        detail3 = (f"轨迹={p.bsplines}, 位移={moved:.2f} m, running={p.saw_state('running')}")
        if GOAL is not None:
            detail3 += f", goal_dist={goal_dist:.2f} m" if goal_dist is not None else ", goal_dist=?"
        if OBSTACLE_BBOXES:
            detail3 += (f", clearance={clearance:.2f}/{effective_min_clearance:.2f} m"
                        if clearance is not None else ", clearance=?")
        if MIN_YAW_CHANGE > 0.0:
            detail3 += (f", path_turn={yaw_change:.2f}/{MIN_YAW_CHANGE:.2f} rad"
                        if yaw_change is not None else ", path_turn=?")
        detail3 = ("OK  " if ok3 else "FAIL ") + detail3
    results.append(("③ 规划闭环", detail3))

    # ③b 目标降级：仅当场景声明 expect_degrade 时检查。
    #   目的：把"降级确实发生了、且落在合理位置"变成可判定的断言，
    #   而不是只看 goal_dist 一个数字（它无法区分"降级成功"与"卡在半路"）。
    if EXPECT_DEGRADE:
        ok_deg = True
        parts = []
        # (1) 日志证据：必须真的出现降级，且次数不超过上限（幂等）
        degrade_cnt = None
        not_connected_cnt = None
        if SCAN_LOG and os.path.isfile(SCAN_LOG):
            try:
                with open(SCAN_LOG, "r", errors="ignore") as fh:
                    text = fh.read()
                degrade_cnt = text.count("degraded to nearest reachable")
                not_connected_cnt = text.count("not connected to robot")
            except OSError:
                pass
        if degrade_cnt is None:
            parts.append("FAIL 读不到运行日志，无法确认降级是否发生")
            ok_deg = False
        else:
            if degrade_cnt < 1:
                parts.append(f"FAIL 降级次数={degrade_cnt}（应至少 1 次：目标不可达却没降级）")
                ok_deg = False
            elif degrade_cnt > MAX_DEGRADE_COUNT:
                parts.append(f"FAIL 降级次数={degrade_cnt} > {MAX_DEGRADE_COUNT}（幂等失效，落点会漂移）")
                ok_deg = False
            else:
                parts.append(f"降级={degrade_cnt} 次")
            if not_connected_cnt:
                parts.append(f"A*不通={not_connected_cnt} 次")
                ok_deg = False
        # (2) 几何证据：停稳后到原目标的距离应落在 [下界, 上界]。
        #     下界默认取"目标到障碍表面距离"（不能停在障碍里），
        #     上界默认取下界 + 1.0 m（不能离降级半径之外还报成功）。
        # 下界优先用场景显式值；否则用 goal_to_obstacle（障碍半宽）+ 机型半径。
        #   不同机型圆柱半径不同（a2=0.25 / go2=0.234 / go2w=0.277），
        #   固定值会让部分机型误判。
        if GOAL_DIST_MIN is not None:
            lo = GOAL_DIST_MIN
        elif GOAL_TO_OBSTACLE is not None:
            lo = GOAL_TO_OBSTACLE + expect["radius"]
        else:
            lo = None
        hi = GOAL_DIST_MAX
        if hi is None and lo is not None:
            hi = lo + 1.0
        if lo is not None and hi is not None and GOAL is not None:
            # 停车后静置取样：避免把"路过目标附近"当成"停稳"
            p.spin_for(3.0)
            final = p.base_pose()
            if not final:
                parts.append("FAIL 取不到停稳位置")
                ok_deg = False
            else:
                d_final = math.hypot(final[0] - GOAL[0], final[1] - GOAL[1])
                within = lo - 1e-3 <= d_final <= hi + 1e-3
                parts.append(f"停稳距目标={d_final:.2f} m，期望[{lo:.2f}, {hi:.2f}]")
                if not within:
                    parts.append("(越界：要么停在障碍里，要么没走到可达点)")
                    ok_deg = False
        detail_deg = (", ".join(parts) if parts else "无可用断言")
        results.append(("③b 目标降级", ("OK  " if ok_deg else "FAIL ") + detail_deg))

    # ④ 关节动画
    p.spin_for(2.0)
    names = list(p.joints.name) if p.joints else []
    leg = [n for n in names if n.endswith(("_hip_joint", "_thigh_joint", "_calf_joint"))]
    wheel = [n for n in names if n.endswith("_foot_joint")]
    ok4 = len(leg) == 12 and len(wheel) == expected_wheels
    detail4 = f"{len(leg)} 个腿关节"
    if expected_wheels or wheel:
        detail4 += f" + {len(wheel)} 个轮关节"
    results.append(("④ /joint_states",
                    f"OK  {detail4}" if ok4
                    else f"FAIL 收到 {len(names)} 个关节 {names[:4]}"))

    # ⑤ 碰撞几何：前后两个圆柱的半径/偏移/高度（取时间戳最接近的一对）
    ok5 = False
    detail5 = "FAIL 没收到完整的前/后 self_inflation marker"
    best = None
    for front in p.markers.get(0, []):
        for rear in p.markers.get(1, []):
            dt = abs((front.header.stamp.sec - rear.header.stamp.sec) +
                     (front.header.stamp.nanosec - rear.header.stamp.nanosec) * 1e-9)
            if best is None or dt < best[0]:
                best = (dt, front, rear)
    if best and best[0] <= 0.02:
        _, front, rear = best
        radius = front.scale.x / 2.0
        offset = math.hypot(front.pose.position.x - rear.pose.position.x,
                            front.pose.position.y - rear.pose.position.y) / 2.0
        height = front.scale.z
        want_h = expect["z_up"] + expect["z_down"]
        ok5 = (abs(radius - expect["radius"]) < 0.02 and
               abs(offset - expect["offset"]) < 0.02 and
               abs(height - want_h) < 0.05)
        detail5 = (f"OK  r={radius:.3f}/{expect['radius']:.3f}, "
                   f"off={offset:.3f}/{expect['offset']:.3f}, "
                   f"h={height:.3f}/{want_h:.3f}")
        if not ok5:
            detail5 = "FAIL " + detail5
    elif best:
        detail5 = f"FAIL 前后 marker 时间差过大 ({best[0]*1000:.0f} ms)"
    results.append(("⑤ self_inflation 几何", detail5))

    print(f"  ---- {ROBOT} 自检 ----")
    for name, msg in results:
        print(f"    {name:24s} {msg}")
    bad = [n for n, m in results if m.startswith("FAIL")]
    print(f"    => {'全部通过' if not bad else '失败项: ' + ', '.join(bad)}")
    p.destroy_node()
    rclpy.shutdown()
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
