#!/usr/bin/env bash
# ============================================================================
# tools/record_raw_bag.sh —— 原始数据一键录包
#
# 默认录制：
#   /tf /tf_static
#   /lio/robo/odom /loc/robo/odom /lio/imu/odom /lio/cloud_world
#   /camera/color/image_raw
#   /cmd_vel
#   /global_path /planning/bspline /local_path
#   /grid_map/occupancy /grid_map/occupancy_inflate
#
# 用法：
#   tools/record_raw_bag.sh                    # 录到 ~/bags/raw_<时间>
#   tools/record_raw_bag.sh --duration 120     # 120s 后自动停
#   tools/record_raw_bag.sh --out /data/run1
#   tools/record_raw_bag.sh --no-camera
#   tools/record_raw_bag.sh --no-grid-map
#   tools/record_raw_bag.sh --extra-topics "/a /b"
#   tools/record_raw_bag.sh --list             # 只打印将录制的话题
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STORAGE="${BAG_STORAGE:-sqlite3}"
OUT="${BAG_OUT:-}"
DURATION=""
CHECK=1
STRICT=0
WITH_CAMERA=1
WITH_GRID_MAP=1
WITH_LOC=1
LIST_ONLY=0
EXTRA_TOPICS=()
QOS_FILE=""

usage() {
  sed -n '2,28p' "$0" | sed 's/^# \{0,1\}//'
}

die() { echo "[error] $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out)         OUT="${2:?缺少目录}"; shift 2;;
    --duration)    DURATION="${2:?缺少秒数}"; shift 2;;
    --storage)     STORAGE="${2:?缺少 storage}"; shift 2;;
    --extra-topics)
      read -r -a _extra <<< "${2:?缺少话题列表}"
      EXTRA_TOPICS+=("${_extra[@]}"); shift 2;;
    --no-camera)   WITH_CAMERA=0; shift;;
    --no-grid-map) WITH_GRID_MAP=0; shift;;
    --no-loc)      WITH_LOC=0; shift;;
    --no-check)    CHECK=0; shift;;
    --strict)      STRICT=1; shift;;
    --list)        LIST_ONLY=1; shift;;
    -h|--help)     usage; exit 0;;
    *)             die "未知参数: $1（--help 查看用法）";;
  esac
done

[[ -n "$OUT" ]] || OUT="$HOME/bags/raw_$(date +%Y%m%d_%H%M%S)"
[[ -z "$DURATION" || "$DURATION" =~ ^[0-9]+$ ]] || die "--duration 必须是整数秒"

TOPICS=(/tf /tf_static /lio/robo/odom)
if [[ "$WITH_LOC" -eq 1 ]]; then TOPICS+=(/loc/robo/odom); fi
TOPICS+=(/lio/imu/odom /lio/cloud_world)
if [[ "$WITH_CAMERA" -eq 1 ]]; then TOPICS+=(/camera/color/image_raw); fi
TOPICS+=(/cmd_vel /global_path /planning/bspline /local_path /local_planner/diagnostics)
if [[ "$WITH_GRID_MAP" -eq 1 ]]; then
  TOPICS+=(/grid_map/occupancy /grid_map/occupancy_inflate)
fi
TOPICS+=("${EXTRA_TOPICS[@]}")

if [[ "$LIST_ONLY" -eq 1 ]]; then
  echo "out: $OUT"
  echo "storage: $STORAGE"
  echo "topics: ${TOPICS[*]}"
  exit 0
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi
command -v ros2 >/dev/null 2>&1 || die "找不到 ros2"

# 按话题生成 QoS；未列出的 extra 话题默认 best_effort
QOS_FILE="$(mktemp /tmp/raw_record_qos.XXXXXX.yaml)"
cleanup() { [[ -n "$QOS_FILE" && -f "$QOS_FILE" ]] && rm -f "$QOS_FILE"; }
trap cleanup EXIT

cat > "$QOS_FILE" <<'EOF'
/tf:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 100

/tf_static:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 1

/lio/robo/odom:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 50

/loc/robo/odom:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 50

/lio/imu/odom:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 50

/lio/cloud_world:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10

/camera/color/image_raw:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10

/cmd_vel:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 20

/global_path:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 1

/planning/bspline:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 10

/local_path:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 10

/grid_map/occupancy:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10

/grid_map/occupancy_inflate:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10

/local_planner/diagnostics:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 10
EOF

for t in "${EXTRA_TOPICS[@]}"; do
  printf '\n%s:\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 10\n' "$t" >> "$QOS_FILE"
done

if [[ "$CHECK" -eq 1 ]]; then
  topics_now="$(ros2 topic list 2>/dev/null || true)"
  missing=()
  for t in "${TOPICS[@]}"; do
    grep -qxF "$t" <<< "$topics_now" || missing+=("$t")
  done
  if [[ ${#missing[@]} -gt 0 ]]; then
    echo "[warn] 当前未发现话题: ${missing[*]}" >&2
    echo "[warn] 可能是驱动/规划器未启动，或话题名不同；继续录制..." >&2
    if [[ "$STRICT" -eq 1 ]]; then
      die "--strict 模式下退出"
    fi
  fi
fi

if [[ -e "$OUT" ]]; then
  die "输出目录已存在: $OUT"
fi
mkdir -p "$(dirname "$OUT")"

echo "================================================================"
echo " output : $OUT"
echo " storage: $STORAGE"
echo " topics : ${TOPICS[*]}"
echo " qos    : $QOS_FILE"
[[ -n "$DURATION" ]] && echo " duration: ${DURATION}s"
echo " 停止   : Ctrl-C"
echo "================================================================"

if [[ -n "$DURATION" ]]; then
  timeout -s INT "$DURATION" ros2 bag record -s "$STORAGE" -o "$OUT" \
    "${TOPICS[@]}" --qos-profile-overrides-path "$QOS_FILE" || true
else
  ros2 bag record -s "$STORAGE" -o "$OUT" \
    "${TOPICS[@]}" --qos-profile-overrides-path "$QOS_FILE" || true
fi

if [[ -d "$OUT" ]]; then
  echo "[done] bag: $OUT"
  ros2 bag info "$OUT" || true
else
  die "录包目录未生成: $OUT"
fi
