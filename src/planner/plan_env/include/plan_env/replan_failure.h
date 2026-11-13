// ============================================================================
// 文件：include/plan_env/replan_failure.h
//
// 局部规划"失败/error"的原因枚举，用于 /local_planner/status 的 reason 字段。
// header-only：只在各包里 include 使用，不产生额外编译单元。
// ============================================================================
#ifndef PLAN_ENV_REPLAN_FAILURE_H
#define PLAN_ENV_REPLAN_FAILURE_H

namespace scan_planner
{
enum class ReplanFailure
{
  NONE = 0,
  CLOSE_TO_GOAL,                  // 起点距局部目标 <0.2m（到点，非错误）
  ASTAR_INIT_ERR,                 // A* 起点/终点问题（INIT_ERR）
  ASTAR_NO_PATH,                  // A* 搜不到路/超时（SEARCH_ERR）
  NO_FREE_TARGET,                 // 局部目标被障碍占据且找不到空闲替代
  TF_UNAVAILABLE,                 // 参考 TF 不可用/过旧
  GLOBAL_TRAJ_FAILED,             // 全局参考轨迹生成失败
  TARGET_OCCUPIED,                // 整条全局路径被占（目标不可达）
  PARAMETERIZE_FAILED,            // B 样条拟合失败
  START_IN_OBSTACLE,              // 优化后前 3 个控制点仍在障碍
  OPTIMIZE_FAILED,                // 优化未收敛/重启耗尽/求解器错误
  REFINE_UNSAFE,                  // 细化后仍碰撞
  DYNAMIC_INFEASIBLE,             // 速度/加速度超限
  EMERGENCY_SAFETY,               // 碰撞且来不及重规划 → 急停
  EMERGENCY_TF_TIMEOUT,           // 参考 TF 超时急停
  EMERGENCY_TF_JUMP_FAIL,         // TF 跳变后重规划失败急停
  EMERGENCY_TF_MISSING_COLLISION, // TF 缺失期间又检测到碰撞急停
  MAX_REPLAN_FAIL,                // 连续失败达上限
  UNKNOWN
};

// 不影响机器狗正常运动的“非错误”原因：不计入 error 状态、也不算重规划失败次数。
//   CLOSE_TO_GOAL —— 起点到局部目标 <0.2 m，是“到点”而不是故障。
inline bool replanFailureIsBenign(ReplanFailure f)
{
  return f == ReplanFailure::CLOSE_TO_GOAL;
}

// “必须人工干预”的硬错误（→ error）：被堵死 / 目标不可达 / 碰撞急停 / 连续失败超限。
// A* 无路（astar_no_path）不在此列：它常由局部地图/搜索池的瞬时情况引起，按可自恢复处理，归 failed。
// 其余（优化、拟合、TF/定位类、超限等可自恢复的）归 failed。
// 【当前策略】不再区分“必须人工干预”的硬错误：
// 全部原因都归入软错误 ⇒ 状态只报 failed（不再有 error、也不再锁存）。
// 保留本函数（而非删除分级），以备将来需要时把某些原因重新划回 error。
inline bool replanFailureNeedsIntervention(ReplanFailure)
{
  return false;
}
inline const char *replanFailureToString(ReplanFailure f)
{
  switch (f)
  {
  case ReplanFailure::NONE:
    return "";
  case ReplanFailure::CLOSE_TO_GOAL:
    return "close_to_goal";
  case ReplanFailure::ASTAR_INIT_ERR:
    return "astar_init_err";
  case ReplanFailure::ASTAR_NO_PATH:
    return "astar_no_path";
  case ReplanFailure::NO_FREE_TARGET:
    return "no_free_target";
  case ReplanFailure::TF_UNAVAILABLE:
    return "tf_unavailable";
  case ReplanFailure::GLOBAL_TRAJ_FAILED:
    return "global_traj_failed";
  case ReplanFailure::TARGET_OCCUPIED:
    return "target_occupied";
  case ReplanFailure::PARAMETERIZE_FAILED:
    return "parameterize_failed";
  case ReplanFailure::START_IN_OBSTACLE:
    return "start_in_obstacle";
  case ReplanFailure::OPTIMIZE_FAILED:
    return "optimize_failed";
  case ReplanFailure::REFINE_UNSAFE:
    return "refine_unsafe";
  case ReplanFailure::DYNAMIC_INFEASIBLE:
    return "dynamic_infeasible";
  case ReplanFailure::EMERGENCY_SAFETY:
    return "emergency_safety";
  case ReplanFailure::EMERGENCY_TF_TIMEOUT:
    return "emergency_tf_timeout";
  case ReplanFailure::EMERGENCY_TF_JUMP_FAIL:
    return "emergency_tf_jump_fail";
  case ReplanFailure::EMERGENCY_TF_MISSING_COLLISION:
    return "emergency_tf_missing_collision";
  case ReplanFailure::MAX_REPLAN_FAIL:
    return "max_replan_fail";
  case ReplanFailure::UNKNOWN:
    return "unknown";
  }
  return "unknown";
}
} // namespace scan_planner

#endif // PLAN_ENV_REPLAN_FAILURE_H
