#!/usr/bin/env bash
# ============================================================================
# 文件：tools/astar_bench/astar_bench.sh —— A* 行为基准入口
#
# 【做什么】
#   编译并运行 tools/astar_bench/main.cpp，汇总 expanded 分布与失败率。
#   用于"改动前后"对比，不做断言、不影响 colcon test。
#
# 【用法】
#   tools/astar_bench/astar_bench.sh                    # 默认 corridor_obstacle
#   tools/astar_bench/astar_bench.sh <pcd> [mode] [n]
#     mode: road(默认) | crossing
#
# 【前置】
#   已执行过 colcon build（需要 install/ 下已安装 path_searching 和 plan_env）
# ============================================================================
# 注意：不要开 set -u —— ROS 的 setup.bash 会引用未定义变量而中断
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC_DIR="${WS}/tools/astar_bench"
BUILD_DIR="${WS}/build/astar_bench"

PCD="${1:-${WS}/tools/scenarios/corridor_obstacle.pcd}"
MODE="${2:-road}"
N="${3:-20}"

# 相对路径转绝对（cmake 的构建目录在别处）
case "${PCD}" in
  /*) : ;;
  *) PCD="${WS}/${PCD}" ;;
esac

if [ ! -f "${WS}/install/path_searching/share/path_searching/cmake/path_searchingConfig.cmake" ]; then
  echo "错误：install/ 下找不到 path_searching，请先运行 colcon build" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release >/dev/null || exit 1
cmake --build "${BUILD_DIR}" -j"$(nproc)" >/dev/null || exit 1

# 运行；把 expanded 从 verbose 日志里抽出来做汇总
OUT="$("${BUILD_DIR}/astar_bench" "${PCD}" "${MODE}" "${N}" 2>&1)"

echo "${OUT}" | grep -E "^#" || true

echo "# expanded 分布："
echo "${OUT}" \
  | grep -oE "expanded=[0-9]+" \
  | cut -d= -f2 \
  | sort -n \
  | awk '{a[NR]=$1; s+=$1}
         END {
           if (NR == 0) { print "  (无数据：A* 未产生 verbose 日志)"; exit }
           printf "  n=%d 中位=%d 均值=%d p90=%d 最大=%d\n",
                  NR, a[int((NR+1)/2)], s/NR, a[int(NR*0.9)], a[NR]
         }'
