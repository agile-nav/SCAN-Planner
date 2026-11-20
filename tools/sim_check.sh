#!/usr/bin/env bash
# ============================================================================
# 文件：tools/sim_check.sh —— 三机型仿真自检
#
# 逐个机型拉起“参考路径”仿真栈（is_real_world:=false navi_mode:=3 robot:=<机型>），
# 然后用 tools/sim_check_probe.py 做闭环自检：
#   ① 模型身份与 12 腿关节   ② TF world->base
#   ③ 规划闭环（收到轨迹 + running + 实际位移）
#   ④ /joint_states 12 关节  ⑤ self_inflation 半径/偏移/高度与档案一致
#
# 用法：
#   tools/sim_check.sh              # 三个机型都跑（a2 go2 go2w）
#   tools/sim_check.sh a2           # 只跑一个
#   USE_GPU=true tools/sim_check.sh # 用 OpenGL 后端（需 -DUSE_GPU=ON 构建）
# 日志：log/sim_check/<机型>.log
# ============================================================================
# 注意：不要开 set -u —— ROS 的 setup.bash 会引用未定义变量而中断

ROBOTS=("$@")
[ ${#ROBOTS[@]} -eq 0 ] && ROBOTS=(a2 go2 go2w)

# 渲染后端：USE_GPU=true 时用 opengl_render_node（需先 -DUSE_GPU=ON 构建，并有可用 GL 3.3 上下文）
USE_GPU="${USE_GPU:-false}"

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${TOOLS_DIR}/.." && pwd)"
LOGDIR="${WS}/log/sim_check"
mkdir -p "${LOGDIR}"

source /opt/ros/humble/setup.bash
# 本工作区 + 依赖工作区（local_sensing / mockamap 等未必在本工作区里）
for s in "${WS}/install/setup.bash" "$HOME/nav_ws/install/setup.bash"; do
  [ -f "$s" ] && source "$s"
done

SCAN_SHARE="$(ros2 pkg prefix scan_planner)/share/scan_planner"
# 自检用 3 m 平地短路（见 tools/sim_check_path.yaml），不使用 10 m 楼梯示例路径
REFERENCE_PATH="${TOOLS_DIR}/sim_check_path.yaml"

# 清掉 ros2 CLI 守护进程的旧话题缓存：连续拉起/停止多个仿真栈时，
# 旧 daemon 可能返回过期话题列表，导致“就绪”判断误判。
ros2 daemon stop >/dev/null 2>&1 || true

LAUNCH_PAT="[r]os2 launch scan_planner run.launch.py"
LAUNCH_PID=""
stop_stack() {
  # 用进程组 Kill：ros2 launch 被 SIGKILL 后，子节点会变成孤儿进程，
  # 下一个机型启动时就会出现“robot_description 串台”（例如 go2w 读到 go2）。
  if [ -n "${LAUNCH_PID}" ]; then
    kill -INT -"${LAUNCH_PID}" 2>/dev/null
    # 等进程组真正退出，最多 10 s；不要固定 sleep 后直接 SIGKILL，
    # 否则子节点容易变成孤儿，Fast-DDS 也会留下半关闭的 participant。
    for _ in $(seq 1 20); do
      kill -0 -"${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -9 -"${LAUNCH_PID}" 2>/dev/null
    LAUNCH_PID=""
  fi
  # 兜底：清掉可能残留的本工作区节点，并等它们真正消失
  pkill -INT -f "${WS}/[i]nstall/" 2>/dev/null
  pkill -INT -f "/opt/ros/humble/lib/[r]obot_state_publisher" 2>/dev/null
  for _ in $(seq 1 20); do
    pgrep -f "${WS}/[i]nstall/" >/dev/null 2>&1 || break
    sleep 0.5
  done
  # 重置 CLI daemon，避免下一轮就绪判断读到上一轮的话题缓存
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
}
trap 'stop_stack; exit 130' INT TERM

fail_total=0
for robot in "${ROBOTS[@]}"; do
  echo "=================================================================="
  echo "机型 ${robot}：拉起仿真栈…"
  LOG="${LOGDIR}/${robot}.log"
  PROFILE="${SCAN_SHARE}/config/robots/${robot}.yaml"
  if [ ! -f "${PROFILE}" ]; then
    echo "  !! 找不到机型档案 ${PROFILE}"
    fail_total=$((fail_total + 1))
    continue
  fi
  setsid ros2 launch scan_planner run.launch.py \
      is_real_world:=false navi_mode:=3 robot:="${robot}" use_gpu:="${USE_GPU}" \
      reference_path_file:="${REFERENCE_PATH}" obstacle_number:=0 >"${LOG}" 2>&1 &
  LAUNCH_PID=$!

  ready=0
  for _ in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q '^/quad_0/body_pose$'; then ready=1; break; fi
    sleep 1
  done
  if [ "${ready}" -eq 0 ]; then
    echo "  !! 45 秒内仿真栈未就绪，日志尾部："
    tail -6 "${LOG}" | sed 's/^/     /'
    stop_stack
    fail_total=$((fail_total + 1))
    continue
  fi
  echo "  仿真栈就绪，开始闭环检查"
  if ! python3 "${TOOLS_DIR}/sim_check_probe.py" "${robot}" "${PROFILE}"; then
    fail_total=$((fail_total + 1))
  fi
  stop_stack
done

echo "=================================================================="
if [ "${fail_total}" -eq 0 ]; then
  echo "全部机型自检通过 ✅"
else
  echo "有 ${fail_total} 个机型存在失败项 ❌（日志见 ${LOGDIR}/）"
fi
exit "${fail_total}"
