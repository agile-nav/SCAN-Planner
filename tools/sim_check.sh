#!/usr/bin/env bash
# ============================================================================
# 文件：tools/sim_check.sh —— 三机型仿真自检
#
# 逐个机型拉起“参考路径”仿真栈（is_real_world:=false navi_mode:=3 robot:=<机型>），
# 然后用 tools/sim_check_probe.py 做闭环自检：
#   ① 模型身份（12 腿；Go2-W 另含 4 轮）   ② TF world->base
#   ③ 规划闭环（轨迹 + running + 位移 + 到终点距离 + 障碍间隙）
#   ④ /joint_states（12 腿；Go2-W 16 关节）  ⑤ self_inflation 与档案一致
#
# 用法：
#   tools/sim_check.sh                              # 三机型 + empty_3m
#   tools/sim_check.sh a2                            # 只跑一个（兼容旧用法）
#   tools/sim_check.sh --scenario straight_10m       # 三机型 + 10 m 直线
#   tools/sim_check.sh --scenario straight_10m a2    # 指定机型
#   tools/sim_check.sh --scenario straight_10m --robot go2w
#   USE_GPU=true tools/sim_check.sh                  # GPU 后端
#
# 场景文件：tools/scenarios/<scenario>.yaml
# 日志：log/sim_check/<scenario>_<机型>.log
# ============================================================================
# 注意：不要开 set -u —— ROS 的 setup.bash 会引用未定义变量而中断

SCENARIO="${SCENARIO:-empty_3m}"
ROBOTS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --scenario)
      SCENARIO="$2"; shift 2 ;;
    --scenario=*)
      SCENARIO="${1#*=}"; shift ;;
    --robot)
      ROBOTS+=("$2"); shift 2 ;;
    --robot=*)
      ROBOTS+=("${1#*=}"); shift ;;
    -h|--help)
      sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
      exit 0 ;;
    -*)
      echo "未知参数: $1" >&2; exit 2 ;;
    *)
      ROBOTS+=("$1"); shift ;;
  esac
done
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

# 场景名只允许字母/数字/下划线/短横线：既防路径穿越，也让日志名可控。
case "${SCENARIO}" in
  ""|*[!A-Za-z0-9_-]*)
    echo "非法场景名: '${SCENARIO}'（只允许 A-Z a-z 0-9 _ -）" >&2
    exit 2 ;;
esac

SCENARIO_FILE="${TOOLS_DIR}/scenarios/${SCENARIO}.yaml"
if [ ! -f "${SCENARIO_FILE}" ]; then
  echo "找不到场景文件: ${SCENARIO_FILE}" >&2
  echo "可用场景: $(ls "${TOOLS_DIR}/scenarios"/*.yaml 2>/dev/null | xargs -n1 basename | sed 's/\.yaml$//' | paste -sd' ' -)" >&2
  exit 2
fi

# 不用 eval：scenario_env.py 输出 "key<TAB>value"，这里逐行 case 赋值。
# 白名单在 Python 侧（ALLOWED_KEYS），值里的换行/制表符也会被拒绝。
while IFS=$'\t' read -r key value; do
  case "${key}" in
    name)                SCENARIO_NAME="${value}" ;;
    path_file)           PATH_FILE="${value}" ;;
    navi_mode)           NAVI_MODE="${value}" ;;
    obstacle_number)     OBSTACLE_NUMBER="${value}" ;;
    use_pcd_map)         USE_PCD_MAP="${value}" ;;
    pcd_map_file)        PCD_MAP_FILE="${value}" ;;
    timeout)             TIMEOUT="${value}" ;;
    min_move)            MIN_MOVE="${value}" ;;
    goal_x)              GOAL_X="${value}" ;;
    goal_y)              GOAL_Y="${value}" ;;
    goal_tol)            GOAL_TOL="${value}" ;;
    goal_settle)         GOAL_SETTLE="${value}" ;;
    obstacle_bbox)       OBSTACLE_BBOX="${value}" ;;
    obstacle_bboxes)     OBSTACLE_BBOXES="${value}" ;;
    min_clearance)       MIN_CLEARANCE="${value}" ;;
    min_yaw_change)      MIN_YAW_CHANGE="${value}" ;;
    expect_degrade)      EXPECT_DEGRADE="${value}" ;;
    goal_to_obstacle)    GOAL_TO_OBSTACLE="${value}" ;;
    goal_dist_min)       GOAL_DIST_MIN="${value}" ;;
    goal_dist_max)       GOAL_DIST_MAX="${value}" ;;
    max_degrade_count)   MAX_DEGRADE_COUNT="${value}" ;;
    map_size_x)          MAP_SIZE_X="${value}" ;;
    map_size_y)          MAP_SIZE_Y="${value}" ;;
    map_size_z)          MAP_SIZE_Z="${value}" ;;
    "")                  ;;
    *)                   echo "未知场景键: ${key}" >&2; exit 2 ;;
  esac
done < <(python3 "${TOOLS_DIR}/scenario_env.py" --tsv "${SCENARIO_FILE}") || exit 2
if [ -z "${PATH_FILE}" ]; then
  echo "场景 ${SCENARIO} 缺少 path_file" >&2
  exit 2
fi
case "${PATH_FILE}" in
  /*) REFERENCE_PATH="${PATH_FILE}" ;;
  *)  REFERENCE_PATH="${TOOLS_DIR}/${PATH_FILE}" ;;
esac
if [ ! -f "${REFERENCE_PATH}" ]; then
  echo "找不到参考路径文件: ${REFERENCE_PATH}" >&2
  exit 2
fi
if [ -n "${PCD_MAP_FILE}" ]; then
  case "${PCD_MAP_FILE}" in
    /*) ;;
    *)  PCD_MAP_FILE="${WS}/${PCD_MAP_FILE}" ;;
  esac
  if [ ! -f "${PCD_MAP_FILE}" ]; then
    echo "找不到 PCD 地图文件: ${PCD_MAP_FILE}" >&2
    exit 2
  fi
fi
echo "场景: ${SCENARIO_NAME} (path=${PATH_FILE}, goal=(${GOAL_X}, ${GOAL_Y}), min_move=${MIN_MOVE}, timeout=${TIMEOUT}s)"

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
  LOG="${LOGDIR}/${SCENARIO}_${robot}.log"
  PROFILE="${SCAN_SHARE}/config/robots/${robot}.yaml"
  if [ ! -f "${PROFILE}" ]; then
    echo "  !! 找不到机型档案 ${PROFILE}"
    fail_total=$((fail_total + 1))
    continue
  fi
  LAUNCH_ARGS=(
    is_real_world:=false
    navi_mode:="${NAVI_MODE}"
    robot:="${robot}"
    use_gpu:="${USE_GPU}"
    reference_path_file:="${REFERENCE_PATH}"
    obstacle_number:="${OBSTACLE_NUMBER}"
    use_pcd_map:="${USE_PCD_MAP}"
  )
  [ -n "${PCD_MAP_FILE}" ] && LAUNCH_ARGS+=("pcd_map_file:=${PCD_MAP_FILE}")
  [ -n "${MAP_SIZE_X}" ] && LAUNCH_ARGS+=("map_size_x:=${MAP_SIZE_X}")
  [ -n "${MAP_SIZE_Y}" ] && LAUNCH_ARGS+=("map_size_y:=${MAP_SIZE_Y}")
  [ -n "${MAP_SIZE_Z}" ] && LAUNCH_ARGS+=("map_size_z:=${MAP_SIZE_Z}")

  setsid ros2 launch scan_planner run.launch.py "${LAUNCH_ARGS[@]}" >"${LOG}" 2>&1 &
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
  if ! SCENARIO_TIMEOUT="${TIMEOUT}" SCENARIO_MIN_MOVE="${MIN_MOVE}" \
       SCENARIO_GOAL_X="${GOAL_X}" SCENARIO_GOAL_Y="${GOAL_Y}" \
       SCENARIO_GOAL_TOL="${GOAL_TOL}" \
       SCENARIO_GOAL_SETTLE="${GOAL_SETTLE}" \
       SCENARIO_OBSTACLE_BBOX="${OBSTACLE_BBOX}" \
       SCENARIO_OBSTACLE_BBOXES="${OBSTACLE_BBOXES}" \
       SCENARIO_MIN_CLEARANCE="${MIN_CLEARANCE}" \
       SCENARIO_MIN_YAW_CHANGE="${MIN_YAW_CHANGE}" \
       SCENARIO_EXPECT_DEGRADE="${EXPECT_DEGRADE}" \
       SCENARIO_GOAL_TO_OBSTACLE="${GOAL_TO_OBSTACLE}" \
       SCENARIO_GOAL_DIST_MIN="${GOAL_DIST_MIN}" \
       SCENARIO_GOAL_DIST_MAX="${GOAL_DIST_MAX}" \
       SCENARIO_MAX_DEGRADE_COUNT="${MAX_DEGRADE_COUNT}" \
       SCENARIO_SCAN_LOG="${LOG}" \
       python3 "${TOOLS_DIR}/sim_check_probe.py" "${robot}" "${PROFILE}"; then
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
