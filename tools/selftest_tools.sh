#!/usr/bin/env bash
# ============================================================================
# tools/selftest_tools.sh —— 工具自测（语法 + 解析 + 导出）
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  [[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
  set -u
fi

bash -n tools/clock_sync_test/record_run.sh tools/record_raw_bag.sh
python3 -m py_compile \
  tools/log_query.py \
  tools/clock_sync_test/analyze_run.py \
  tools/view_stair_log.py

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

cat > "$TMP/run.log" <<'EOF'
[scan_planner_node-1] [DEBUG] [1.000] [scan_planner_node]: [replan-geom] rid=1 ok=0 robot=(1.00,2.00,yaw=0) goal=(5.00,6.00) d_goal=5.00 start=(1.00,2.00) target=(3.00,4.00) d_target=2.00 bearing=0deg traj_dir=(1.00,0.00) traj_dur=2.00
[scan_planner_node-1] [INFO] [1.010] [scan_planner_node]: [fsm] event=state from=EXEC_TRAJ to=REPLAN_TRAJ trigger=FSM
[scan_planner_node-1] [WARN] [1.100] [scan_planner_node]: [fsm] event=replan_failed rid=42 traj_id=17 reason=astar_no_path replan_fail=3/20
[scan_planner_node-1] [WARN] [1.100] [scan_planner_node]: [astar] event=search_failed reason=SEARCH_ERR rid=42 seg=1 expanded=10 blocked=5(50%) time=0.20s pool=±4.00m start=(1.00,2.00,0.50) adj_s=0.00m end=(3.00,4.00,0.50) adj_e=0.00m
[scan_planner_node-1] [WARN] [1.200] [scan_planner_node]: [collision] ahead=0.50s pos=(2.00,3.00,0.50) self_occ=0
[scan_planner_node-1] [INFO] [2.000] [scan_planner_node]: [fsm] event=mission_done reason=GOAL_REACHED rid=42
EOF

python3 tools/log_query.py summary "$TMP/run.log" > "$TMP/summary.txt"
grep -q "replan_failed: 1" "$TMP/summary.txt"
grep -q "astar_no_path" "$TMP/summary.txt"

python3 tools/log_query.py last-failure "$TMP/run.log" > "$TMP/last.txt"
grep -q "rid=42" "$TMP/last.txt"
grep -q "mission_done" "$TMP/last.txt"

python3 tools/log_query.py export "$TMP/run.log" --out "$TMP/events.jsonl" >/dev/null
test -s "$TMP/events.jsonl"

mkdir -p "$TMP/run/analysis"
cp "$TMP/run.log" "$TMP/run/run.log"
printf '1.500000\tblock1\n' > "$TMP/run/events.tsv"
python3 tools/clock_sync_test/analyze_run.py "$TMP/run" --no-bag --no-plot --json >/dev/null
test -s "$TMP/run/analysis/events.jsonl"
grep -q "replan_failed 次数: 1" "$TMP/run/analysis/summary.txt"
grep -q "user_event" "$TMP/run/analysis/events.csv"

mkdir -p "$TMP/root/current"
cp "$TMP/run.log" "$TMP/root/current/run.log"
CLOCK_SYNC_ROOT="$TMP/root" python3 tools/log_query.py summary > "$TMP/default.txt"
grep -q "replan_failed: 1" "$TMP/default.txt"

if python3 -c 'import rclpy, visualization_msgs' >/dev/null 2>&1; then
  python3 - "$TMP/run.log" <<'PY'
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("view_stair_log", "tools/view_stair_log.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
robot, targets, astar, coll, goal = mod.parse_log(sys.argv[1])
assert len(robot) == 1, robot
assert len(targets) == 1, targets
assert len(astar) == 2, astar
assert len(coll) == 1, coll
assert goal == (5.0, 6.0), goal
PY
else
  echo "[selftest] 跳过 view_stair_log 解析测试（ROS Python 环境不可用）"
fi

bash tools/clock_sync_test/record_run.sh --help >/dev/null
bash tools/clock_sync_test/record_run.sh selftest --dry-run --robot a2 \
  --extra-topics "/demo/topic_a /demo/topic_b" > "$TMP/dry.txt"
grep -q "robot: a2" "$TMP/dry.txt"
grep -q "/demo/topic_a" "$TMP/dry.txt"
grep -q "/lio/robo/odom" "$TMP/dry.txt"

bash tools/record_raw_bag.sh --list > "$TMP/raw_list.txt"
grep -q "/lio/cloud_world" "$TMP/raw_list.txt"
grep -q "/grid_map/occupancy_inflate" "$TMP/raw_list.txt"
grep -q "/camera/color/image_raw" "$TMP/raw_list.txt"
bash tools/record_raw_bag.sh --list --no-camera --no-grid-map > "$TMP/raw_list2.txt"
if grep -q "/camera/color/image_raw" "$TMP/raw_list2.txt"; then
  echo "[selftest] --no-camera 未生效" >&2
  exit 1
fi

echo "[selftest] tools OK"
