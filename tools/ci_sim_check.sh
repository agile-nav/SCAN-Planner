#!/usr/bin/env bash
# ============================================================================
# 文件：tools/ci_sim_check.sh —— CI 中的仿真场景入口
#
# 前置：已经 colcon build 并 source install/setup.bash。
# 用法：
#   tools/ci_sim_check.sh          # 只跑 empty_3m 烟测
#   tools/ci_sim_check.sh --full   # 跑全部回归场景
# ============================================================================
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS}"

source /opt/ros/humble/setup.bash
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
else
  echo "install/setup.bash 不存在，请先 colcon build" >&2
  exit 1
fi

./tools/sim_check.sh --scenario empty_3m

if [ "${1:-}" = "--full" ]; then
  for scenario in straight_10m corridor_obstacle narrow_passage turning multi_obstacle; do
    ./tools/sim_check.sh --scenario "${scenario}"
  done
fi
