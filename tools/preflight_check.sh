#!/usr/bin/env bash
# ============================================================================
# preflight_check.sh —— 按 START 之前 / 卡住时的"体检"：一眼看出卡在哪一环
#
#   bash tools/preflight_check.sh            # 全部检查
#   bash tools/preflight_check.sh --quick    # 只查进程与门控（最快）
#
# 只读：不改状态、不发任何指令。
# ============================================================================
WS=${WS:-$HOME/scan_planner_github_ws}
QUICK=0
[ "${1:-}" = "--quick" ] && QUICK=1

# 注意：ROS 的 setup.bash 里有未定义变量，必须在 set -u 之前 source
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" 2>/dev/null
set -u

echo "============================================================"
echo " scan_planner 启动前体检    $(date '+%F %T')"
echo "============================================================"

bfail=0
ok()  { printf "  [ OK ] %s\n" "$*"; }
bad() { printf "  [FAIL] %s\n" "$*"; bfail=$((bfail+1)); }
wr()  { printf "  [WARN] %s\n" "$*"; }

cnt_proc() { local n; n=$(pgrep -fc "$1" 2>/dev/null); echo "${n:-0}"; }

# ---------------- 1) 进程（重复实例会让 RViz 圆柱闪烁） ----------------
echo "-- 1) 关键进程 --"
for pat in scan_planner_node closed_loop_controller; do
  n=$(cnt_proc "[l]ib/scan_planner/$pat")
  if [ "$n" -eq 1 ]; then ok "$pat ×1"
  elif [ "$n" -eq 0 ]; then bad "$pat 没在跑 → bash $WS/nav_bringup.sh start"
  else bad "$pat 有 $n 个实例 → 先全部杀掉再启动（重复实例会让自膨胀圆柱闪烁/尺寸乱跳）"
  fi
done
for pat in global_planner map_localization; do
  n=$(cnt_proc "$pat")
  if [ "$n" -ge 1 ]; then ok "$pat 在跑"; else wr "$pat 没在跑（navi_mode=3 需要它产出 /global_path）"; fi
done

if [ "$QUICK" -eq 0 ]; then
  # ---------------- 2) 话题 / 状态 / 地图（rclpy 一次采样） ----------------
  echo "-- 2) 话题与状态 --"
  tmp=$(mktemp)
  python3 - >"$tmp" 2>/dev/null <<'PY'
import rclpy, time, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

qb = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
ql = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
rclpy.init(); n = Node("preflight"); L = {}
n.create_subscription(Bool, "/motion_enabled", lambda m: L.__setitem__("m", m.data), 20)
n.create_subscription(String, "/local_planner/status", lambda m: L.__setitem__("s", m.data), ql)
n.create_subscription(Bool, "/planning/go2_execution_frozen", lambda m: L.__setitem__("f", m.data), 10)
n.create_subscription(Path, "/global_path", lambda m: L.__setitem__("gp", m), 10)
n.create_subscription(Odometry, "/lio/robo/odom", lambda m: L.__setitem__("o", m), 5)
n.create_subscription(PointCloud2, "/grid_map/occupancy_inflate", lambda m: L.__setitem__("p", m), qb)
n.create_subscription(PoseStamped, "/goal_3d", lambda m: L.__setitem__("g", m), ql)
n.create_subscription(PoseStamped, "/goal_3d", lambda m: L.__setitem__("g", m), 10)
t = time.time() + 8.0
while time.time() < t:
    rclpy.spin_once(n, timeout_sec=0.05)

# 1) 门控
m = L.get("m")
if m is True:
    print("  [ OK ] /motion_enabled = true（门控已开）")
else:
    print("  [FAIL] /motion_enabled = %s → 按遥控器 START；或临时：ros2 topic pub --once "
          "/motion_enabled std_msgs/msg/Bool \"{data: true}\"" % m)

# 2) 规划器状态
s = L.get("s")
if s and '"state":"running"' in s:
    print("  [ OK ] /local_planner/status = %s" % s)
elif s and '"state":"stopped"' in s:
    print("  [WARN] /local_planner/status = %s（门控关，或收到过 pause 还没 resume）" % s)
else:
    print("  [WARN] /local_planner/status = %s" % (s or "(没收到)"))

# 3) 控制器冻结
f = L.get("f")
if f is True:
    print("  [WARN] execution_frozen = true（控制器判卡住/未使能 ⇒ 轨迹时钟被冻结，车不会动）")
else:
    print("  [ OK ] execution_frozen = %s" % f)

o = L.get("o")
# 4) 全局路径 + 与目标的一致性
gp = L.get("gp")
if gp is None:
    print("  [FAIL] 没收到 /global_path → 重发目标（RViz 2D Goal Pose / 原任务入口）")
elif not gp.poses:
    print("  [FAIL] /global_path 是空的（0 点）→ 全局规划器找不到通路，重发目标或检查它的地图")
else:
    L2 = 0.0
    for i in range(len(gp.poses) - 1):
        a, b = gp.poses[i].pose.position, gp.poses[i+1].pose.position
        L2 += math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
    p1 = gp.poses[-1].pose.position
    print("  [ OK ] /global_path %d 点，长 %.1f m，终点 (%.2f, %.2f, %.2f)"
          % (len(gp.poses), L2, p1.x, p1.y, p1.z))
    g = L.get("g")
    if g is None:
        print("  [WARN] 没收到 /goal_3d → 跳过“路径终点 vs 目标”检查")
    else:
        gx, gy, gz = g.pose.position.x, g.pose.position.y, g.pose.position.z
        d_end = math.hypot(p1.x - gx, p1.y - gy)
        if d_end > 1.0:
            print("  [FAIL] 路径终点 (%.2f,%.2f) 与目标 (%.2f,%.2f) 相差 %.2f m"
                  " ⇒ 这条 /global_path 不是通到当前目标的（会带着车往错方向走）"
                  % (p1.x, p1.y, gx, gy, d_end))
        else:
            print("  [ OK ] 路径终点与目标一致（差 %.2f m）" % d_end)
        if o is not None:
            p = o.pose.pose.position
            straight = math.hypot(gx - p.x, gy - p.y)
            if straight > 1.0 and L2 / straight > 2.5:
                print("  [WARN] 路径长度是直线的 %.1f 倍（%.1f m vs %.1f m）"
                      " ⇒ 大概率在绕圈，局部目标会先朝反方向走" % (L2 / straight, L2, straight))
            elif straight > 1.0:
                print("  [ OK ] 路径长度/直线 = %.1f 倍（%.1f m vs %.1f m）"
                      % (L2 / straight, L2, straight))

# 5) 位姿 + 是否被膨胀层包住
if o is None:
    print("  [FAIL] 没收到 /lio/robo/odom → 定位（super_lio/map_localization）没起")
else:
    p = o.pose.pose.position
    v = math.hypot(o.twist.twist.linear.x, o.twist.twist.linear.y)
    print("  [ OK ] odom 位姿 (%.2f, %.2f, %.2f)   |v| = %.3f m/s" % (p.x, p.y, p.z, v))
    pc = L.get("p")
    if pc is None:
        print("  [WARN] 没收到 /grid_map/occupancy_inflate（跳过“是否被障碍包住”检查）")
    else:
        try:
            from sensor_msgs_py import point_cloud2 as pc2
            cell = 0.1
            grid = set()
            for x, y, z in pc2.read_points(pc, field_names=("x", "y", "z"), skip_nans=True):
                grid.add((round(x / cell), round(y / cell), round(z / cell)))
            cx, cy, cz = round(p.x / cell), round(p.y / cell), round(p.z / cell)
            hit = 0
            for dx in (-2, -1, 0, 1, 2):
                for dy in (-2, -1, 0, 1, 2):
                    if math.hypot(dx, dy) * cell > 0.25:
                        continue
                    for dz in (-1, 0, 1):
                        if (cx + dx, cy + dy, cz + dz) in grid:
                            hit += 1
            if hit == 0:
                print("  [ OK ] 机器人自身周围无膨胀格 ⇒ 它不在障碍里")
            else:
                print("  [FAIL] 机器人自身周围有 %d 个膨胀格 ⇒ 它正被“障碍”包住，"
                      "规划必然失败（常见于重定位/重启定位后，或贴墙）" % hit)
        except Exception as e:
            print("  [WARN] 膨胀地图检查异常：%s" % e)

n.destroy_node()
if rclpy.ok():
    rclpy.shutdown()
PY
  cat "$tmp"
  pyfail=$(grep -c '\[FAIL\]' "$tmp" 2>/dev/null); pyfail=${pyfail:-0}
  rm -f "$tmp"
  bfail=$((bfail + pyfail))
fi

echo "------------------------------------------------------------"
if [ "$bfail" -gt 0 ]; then
  echo " 结论：有 $bfail 项 FAIL —— 先修它们，再按 START"
else
  echo " 结论：没有 FAIL。若仍不走，按顺序看 WARN，并检查 WARN 项里的提示"
fi
echo "============================================================"
