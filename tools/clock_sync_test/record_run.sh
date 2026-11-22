#!/usr/bin/env bash
# ============================================================================
# record_run.sh —— G1/G2/G3 真机验证：一条命令录下日志 + 关键话题
#
# 你只负责操控机器人；本脚本负责：
#   1) 启动 ros2 bag record（带正确的 QoS 覆盖）
#   2) 启动规划栈（可选），把终端输出（含 cout 裸输出）写进 run.log
#   3) 归档生效参数、git 版本、事件标记
#   4) 结束后自动调用 analyze_run.py 出 CSV/图/摘要
#
# 用法：
#   bash record_run.sh <label>                    # 手动 Ctrl-C 停止
#   bash record_run.sh <label> --duration 120     # 120s 后自动停
#   bash record_run.sh <label> --no-launch        # 只录包（规划栈你自己起）
#   bash record_run.sh <label> --with-map         # 额外录 /grid_map/*
#   bash record_run.sh <label> --log-level debug  # 规划栈日志级别（默认 debug）
#   bash record_run.sh <label> --robot go2 --navi-mode 3 --controller-mode closed_loop
#   bash record_run.sh <label> --extra-topics "/lio/cloud_world /lio/imu/odom"
#   bash record_run.sh <label> --dry-run          # 只打印配置，不启动
#   bash record_run.sh <label> --quiet            # 不打印 banner / 不 tail
#   bash record_run.sh --event "block 1"          # 给当前 run 打时间戳标记
#   bash record_run.sh --list-events              # 查看当前 run 的事件
#
# 产物：${CLOCK_SYNC_ROOT:-$HOME/clock_sync_runs}/<label>_<时间>/
#   meta.txt  run.log  bag/  bag_record.log  events.tsv  roslog/
#   params_planner.yaml  params_controller.yaml
#   analysis/  (traj_clock.csv / ctl_clock.csv / events.csv / summary.txt /
#              log_summary.txt / plots/)
# ============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROOT="${CLOCK_SYNC_ROOT:-$HOME/clock_sync_runs}"
ODOM_TOPIC="${ODOM_TOPIC:-}"
STORAGE="${BAG_STORAGE:-sqlite3}"
LOG_LEVEL="${LOG_LEVEL:-debug}"
ROBOT="${ROBOT:-a2}"
NAVI_MODE="${NAVI_MODE:-3}"
CONTROLLER_MODE="${CONTROLLER_MODE:-closed_loop}"
MAX_LOG_MB="${MAX_LOG_MB:-200}"
QUIET="${QUIET:-0}"
ROBOT_PROFILE="$REPO/src/planner/plan_manage/config/robots/${ROBOT}.yaml"

usage() {
  sed -n '2,42p' "$0" | sed 's/^# \{0,1\}//'
}

die() { echo "[error] $*" >&2; exit 1; }

# ---- --list-events 快路径 ----
if [[ "${1:-}" == "--list-events" ]]; then
  cur="$ROOT/current"
  [[ -L "$cur" && -d "$cur" ]] || die "没有正在进行的 run（$cur）"
  [[ -f "$cur/events.tsv" ]] && cat "$cur/events.tsv" || echo "（当前 run 还没有事件）"
  exit 0
fi

# ---- --event 快路径：只给“正在进行的 run”追加一条带时间戳的标记 ----
if [[ "${1:-}" == "--event" ]]; then
  msg="${2:-}"
  [[ -n "$msg" ]] || { echo '用法: bash record_run.sh --event "说明"' >&2; exit 2; }
  cur="$ROOT/current"
  [[ -L "$cur" && -d "$cur" ]] || { echo "没有正在进行的 run（$cur）" >&2; exit 1; }
  printf '%s\t%s\n' "$(date +%s.%N)" "$msg" >> "$cur/events.tsv"
  echo "[event] $(date +%H:%M:%S)  $msg"
  exit 0
fi

LABEL=""; DURATION=""; NO_LAUNCH=0; WITH_MAP=0; DO_ANALYZE=1; DRY_RUN=0
EXTRA_TOPICS=()
LAUNCH_EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)  DURATION="${2:?缺少秒数}"; shift 2;;
    --no-launch) NO_LAUNCH=1; shift;;
    --with-map)  WITH_MAP=1; shift;;
    --no-analyze) DO_ANALYZE=0; shift;;
    --storage)   STORAGE="${2:?缺少 storage}"; shift 2;;
    --log-level) LOG_LEVEL="${2:?缺少日志级别}"; shift 2;;
    --robot)     ROBOT="${2:?缺少机型}"; shift 2;;
    --navi-mode) NAVI_MODE="${2:?缺少 navi_mode}"; shift 2;;
    --controller-mode) CONTROLLER_MODE="${2:?缺少控制器模式}"; shift 2;;
    --extra-topics)
      read -r -a _extra <<< "${2:?缺少话题列表}"
      EXTRA_TOPICS+=("${_extra[@]}"); shift 2;;
    --max-log-mb) MAX_LOG_MB="${2:?缺少 MB}"; shift 2;;
    --dry-run)   DRY_RUN=1; shift;;
    --quiet)     QUIET=1; shift;;
    -h|--help)   usage; exit 0;;
    --)          shift; LAUNCH_EXTRA+=("$@"); break;;
    -*)          echo "未知参数: $1" >&2; usage; exit 2;;
    *)           LABEL="$1"; shift;;
  esac
done
[[ -n "$LABEL" ]] || { echo "缺少 <label>" >&2; usage; exit 2; }
ROBOT_PROFILE="$REPO/src/planner/plan_manage/config/robots/${ROBOT}.yaml"
[[ "$NAVI_MODE" =~ ^[123]$ ]] || die "navi-mode 必须是 1/2/3"
[[ "$CONTROLLER_MODE" == "open_loop" || "$CONTROLLER_MODE" == "closed_loop" ]] || \
  die "controller-mode 必须是 open_loop/closed_loop"
[[ -f "$ROBOT_PROFILE" ]] || die "找不到机型档案: $ROBOT_PROFILE"

# ---- 机型档案里的真机话题（body_pose/sensor_pose/cloud） ----
PROFILE_TOPICS=()
if command -v python3 >/dev/null 2>&1; then
  mapfile -t PROFILE_TOPICS < <(python3 - "$ROBOT_PROFILE" <<'PY'
import sys
try:
    import yaml
    data = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
    topics = data.get("robot_topics", {}) or {}
    for key in ("body_pose", "sensor_pose", "cloud"):
        value = topics.get(key)
        if value:
            print(value)
except Exception:
    pass
PY
  )
fi
if [[ -z "$ODOM_TOPIC" ]]; then
  ODOM_TOPIC="${PROFILE_TOPICS[0]:-/lio/robo/odom}"
fi

# ---- 话题列表：内置 + 机型档案 + 用户追加（去重） ----
declare -A _seen=()
TOPICS=()
add_topic() {
  local t="$1"
  [[ -n "$t" && -z "${_seen[$t]:-}" ]] || return 0
  _seen[$t]=1
  TOPICS+=("$t")
}
for t in /tf /tf_static "$ODOM_TOPIC" /loc/robo/odom /loc/status \
         /global_path /planning/bspline /local_path /cmd_vel \
         /motion_enabled /planning/go2_execution_frozen \
         /local_planner/status /local_planner/diagnostics /self_inflation; do
  add_topic "$t"
done
for t in "${PROFILE_TOPICS[@]}" "${EXTRA_TOPICS[@]}"; do add_topic "$t"; done
if [[ $WITH_MAP -eq 1 ]]; then
  add_topic /grid_map/occupancy
  add_topic /grid_map/occupancy_inflate
fi

# ---- 预检 ----
[[ -f /opt/ros/humble/setup.bash ]] || die "找不到 /opt/ros/humble/setup.bash"
[[ -f "$REPO/install/setup.bash" ]] || \
  echo "[warn] 没找到 $REPO/install/setup.bash，请确认已编译" >&2
if command -v df >/dev/null 2>&1; then
  avail_mb=$(df -Pm "$ROOT" 2>/dev/null | awk 'NR==2 {print $4}')
  if [[ -n "$avail_mb" && "$avail_mb" -lt 2048 ]]; then
    echo "[warn] $ROOT 剩余空间 ${avail_mb}MB，可能不足以录制" >&2
  fi
fi

if [[ $DRY_RUN -eq 1 ]]; then
  echo "robot: $ROBOT"
  echo "navi_mode: $NAVI_MODE"
  echo "controller_mode: $CONTROLLER_MODE"
  echo "log_level: $LOG_LEVEL"
  echo "odom_topic: $ODOM_TOPIC"
  echo "topics: ${TOPICS[*]}"
  echo "launch: ros2 launch scan_planner run.launch.py is_real_world:=true robot:=$ROBOT navi_mode:=$NAVI_MODE controller_mode:=$CONTROLLER_MODE log_level:=$LOG_LEVEL ${LAUNCH_EXTRA[*]:-}"
  exit 0
fi

RUN_DIR="$ROOT/${LABEL}_$(date +%Y%m%d_%H%M%S_%3N)"
mkdir -p "$RUN_DIR/analysis" "$RUN_DIR/roslog"
ln -sfn "$RUN_DIR" "$ROOT/current"

# ---- 环境 ----
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# 先 source 其他工作空间（定位/全局规划），再 source 本仓库的 install，
# 保证“调试分支的 scan_planner”优先级最高。
#   EXTRA_SETUP=~/nav_ws/install/setup.bash
if [[ -n "${EXTRA_SETUP:-}" && -f "${EXTRA_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${EXTRA_SETUP}"
  echo "[env] 已 source EXTRA_SETUP: $EXTRA_SETUP"
fi
if [[ -f "$REPO/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$REPO/install/setup.bash"
else
  echo "[warn] 没找到 $REPO/install/setup.bash，请确认已编译并 source"
fi
set -u

# ---- meta / 参数快照 ----
{
  echo "label: $LABEL"
  echo "start: $(date -Is)"
  echo "repo: $REPO"
  echo "branch: $(git -C "$REPO" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
  echo "head: $(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "odom_topic: $ODOM_TOPIC"
  echo "storage: $STORAGE"
  echo "duration: ${DURATION:-manual}"
  echo "with_map: $WITH_MAP"
  echo "log_level: $LOG_LEVEL"
  echo "robot: $ROBOT"
  echo "navi_mode: $NAVI_MODE"
  echo "controller_mode: $CONTROLLER_MODE"
  echo "max_log_mb: $MAX_LOG_MB"
} > "$RUN_DIR/meta.txt"
for f in common.yaml "robots/${ROBOT}.yaml"; do
  [[ -f "$REPO/src/planner/plan_manage/config/$f" ]] && \
    cp "$REPO/src/planner/plan_manage/config/$f" "$RUN_DIR/" 2>/dev/null || true
done

if [[ "$QUIET" -eq 0 ]]; then
  echo "================================================================"
  echo " run dir : $RUN_DIR"
  echo " robot   : $ROBOT  navi_mode=$NAVI_MODE  controller=$CONTROLLER_MODE"
  echo " odom    : $ODOM_TOPIC"
  echo " 话题    : ${TOPICS[*]}"
  echo " 停止    : Ctrl-C（或 --duration 到点）"
  echo " 打标记  : bash $0 --event \"block 1\""
  echo "----------------------------------------------------------------"
  echo " 建议依次做："
  echo "   1) 正常巡航 60~90s"
  echo "   2) 挡住车 2~5s，松手（mark: block1 / release1）"
  echo "   3) 长时卡住 >=5s（mark: block_long / release_long）"
  echo "   4) 让它跑到目标收尾"
  echo "================================================================"
fi

BAG_PID=""; LAUNCH_PID=""; TAIL_PID=""; WATCH_PID=""; PARAM_PID=""

cleanup() {
  trap - INT TERM EXIT
  [[ -n "$WATCH_PID" ]] && kill "$WATCH_PID" 2>/dev/null || true
  [[ -n "$PARAM_PID" ]] && kill "$PARAM_PID" 2>/dev/null || true
  [[ -n "$TAIL_PID"  ]] && kill "$TAIL_PID"  2>/dev/null || true

  if [[ -n "$LAUNCH_PID" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  if [[ -n "$BAG_PID" ]]; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi

  echo "end: $(date -Is)" >> "$RUN_DIR/meta.txt"
  echo "[done] 数据已保存：$RUN_DIR"

  if [[ "$DO_ANALYZE" -eq 1 && -f "$SCRIPT_DIR/analyze_run.py" ]]; then
    echo "[analyze] 正在生成 CSV / 图 / 摘要 ..."
    python3 "$SCRIPT_DIR/analyze_run.py" "$RUN_DIR" || \
      echo "[analyze] 分析失败（可稍后手动跑：python3 $SCRIPT_DIR/analyze_run.py $RUN_DIR）"
    LOG_QUERY="$(cd "$SCRIPT_DIR/.." && pwd)/log_query.py"
    if [[ -f "$LOG_QUERY" && -f "$RUN_DIR/run.log" ]]; then
      {
        echo "===== log_query summary ====="
        python3 "$LOG_QUERY" summary "$RUN_DIR/run.log"
        echo
        echo "===== log_query last-failure ====="
        python3 "$LOG_QUERY" last-failure "$RUN_DIR/run.log"
      } > "$RUN_DIR/analysis/log_summary.txt" 2>&1 || true
      echo "[analyze] 失败链摘要 -> analysis/log_summary.txt"
    fi
  fi

  # 日志过大时压缩，避免长期运行占满磁盘
  if [[ -f "$RUN_DIR/run.log" && "$MAX_LOG_MB" -gt 0 ]]; then
    size_mb=$(( $(stat -c%s "$RUN_DIR/run.log" 2>/dev/null || echo 0) / 1024 / 1024 ))
    if [[ "$size_mb" -ge "$MAX_LOG_MB" ]]; then
      gzip -f "$RUN_DIR/run.log" && echo "[done] run.log 已压缩为 run.log.gz"
    fi
  fi
  exit 0
}
trap cleanup INT TERM EXIT

# ---- 开始录包 ----
ros2 bag record -s "$STORAGE" -o "$RUN_DIR/bag" "${TOPICS[@]}" \
  --qos-profile-overrides-path "$SCRIPT_DIR/bag_qos.yaml" \
  > "$RUN_DIR/bag_record.log" 2>&1 &
BAG_PID=$!
sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "[error] ros2 bag record 启动失败，见 $RUN_DIR/bag_record.log"
  exit 1
fi
echo "[bag] recording -> $RUN_DIR/bag"

# ---- 启动规划栈（可选）----
if [[ $NO_LAUNCH -eq 0 ]]; then
  LAUNCH_CMD=(ros2 launch scan_planner run.launch.py
              is_real_world:=true robot:="$ROBOT" navi_mode:="$NAVI_MODE"
              controller_mode:="$CONTROLLER_MODE" log_level:="$LOG_LEVEL")
  # 用户在 "--" 后传入的 launch 参数优先级最高
  LAUNCH_CMD+=("${LAUNCH_EXTRA[@]}")
  ROS_LOG_DIR="$RUN_DIR/roslog" stdbuf -oL -eL "${LAUNCH_CMD[@]}" \
    > "$RUN_DIR/run.log" 2>&1 &
  LAUNCH_PID=$!
  if [[ "$QUIET" -eq 0 ]]; then
    tail -n +1 -f "$RUN_DIR/run.log" &
    TAIL_PID=$!
  fi
  # 5s 后抓一次“生效参数”，便于核对 traj_clock_max_lead / time_lead 等
  (
    sleep 5
    for _ in 1 2 3 4 5; do
      if timeout 5 ros2 param dump /scan_planner_node > "$RUN_DIR/params_planner.yaml" 2>/dev/null; then break; fi
      sleep 2
    done
    for _ in 1 2 3 4 5; do
      if timeout 5 ros2 param dump /closed_loop_controller > "$RUN_DIR/params_controller.yaml" 2>/dev/null; then break; fi
      sleep 2
    done
  ) &
  PARAM_PID=$!
  echo "[launch] 规划栈已启动（输出见 run.log）"
else
  echo "[no-launch] 请自行启动规划栈，并把输出重定向到：$RUN_DIR/run.log"
  echo "           例如： ros2 launch scan_planner run.launch.py robot:=$ROBOT navi_mode:=$NAVI_MODE log_level:=$LOG_LEVEL ... 2>&1 | tee $RUN_DIR/run.log"
fi

if [[ -n "$DURATION" ]]; then
  ( sleep "$DURATION" ) &
  WATCH_PID=$!
  sleep "$DURATION"
  echo "[auto-stop] ${DURATION}s 到，结束录制"
else
  while true; do sleep 1; done
fi

exit 0
