#!/usr/bin/env python3
# ============================================================================
# 文件：tools/sim_check_probe.py —— 仿真闭环自检探针（被 tools/sim_check.sh 调用）
#
# 在仿真栈已经拉起后，检查：
#   ① 模型身份   ：/robot_description 的 <robot name=...> 与机型一致，12 个腿关节（Go2-W 另含 4 轮）
#   ② TF         ：world -> base 可查
#   ③ 规划闭环   ：收到 planning/bspline、状态进入 running、位移足够；
#                   场景有终点时检查到终点距离并等待 settle，有 obstacle_bbox 时
#                   检查最小间隙（阈值至少为机型圆柱半径 + 0.05 m）
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
_bbox = os.environ.get("SCENARIO_OBSTACLE_BBOX", "")
OBSTACLE_BBOX = tuple(float(v) for v in _bbox.split(",")) if _bbox else None
MIN_CLEARANCE = float(os.environ.get("SCENARIO_MIN_CLEARANCE", "0.0"))


def bbox_distance(x, y, bbox):
    """点到矩形包围盒的二维距离；点在盒内返回 0。"""
    xmin, xmax, ymin, ymax = bbox
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


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

    def base_xy(self):
        try:
            t = self.tf.lookup_transform("world", "base", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def wait_for_base(self, timeout):
        """等 TF 就绪并返回第一帧 base 位置（用于记录运动的起点）。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pos = self.base_xy()
            if pos:
                return pos
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
        while time.time() < deadline:
            rclpy.spin_once(p, timeout_sec=0.05)
            p1 = p.base_xy()
            if not p1:
                continue
            path_xy.append(p1)
            moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if GOAL is not None:
                goal_dist = math.hypot(p1[0] - GOAL[0], p1[1] - GOAL[1])
                if goal_dist <= GOAL_TOL:
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
        if OBSTACLE_BBOX is not None:
            if path_xy:
                clearance = min(bbox_distance(x, y, OBSTACLE_BBOX)
                                for x, y in path_xy)
                ok3 = ok3 and clearance >= effective_min_clearance
            else:
                ok3 = False
        detail3 = (f"轨迹={p.bsplines}, 位移={moved:.2f} m, running={p.saw_state('running')}")
        if GOAL is not None:
            detail3 += f", goal_dist={goal_dist:.2f} m" if goal_dist is not None else ", goal_dist=?"
        if OBSTACLE_BBOX is not None:
            detail3 += (f", clearance={clearance:.2f}/{effective_min_clearance:.2f} m"
                        if clearance is not None else ", clearance=?")
        detail3 = ("OK  " if ok3 else "FAIL ") + detail3
    results.append(("③ 规划闭环", detail3))

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
