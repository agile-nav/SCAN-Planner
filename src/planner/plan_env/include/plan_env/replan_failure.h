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
  STUCK_GIVEUP,                   // 卡死：重试与后撤都用尽，放弃本段
  EMERGENCY_EXIT,                 // 急停后无法自动恢复，退出并放弃任务
  MEMORY_ERROR,                   // 重规划期间内存分配失败（std::bad_alloc）
  UNKNOWN
};

// 不影响机器狗正常运动的“非错误”原因：不算重规划失败次数。
//   CLOSE_TO_GOAL —— 起点到局部目标 <0.2 m，是“到点”而不是故障。
inline bool replanFailureIsBenign(ReplanFailure f)
{
  return f == ReplanFailure::CLOSE_TO_GOAL;
}

// 注意：失败原因不再用于推导状态话题。状态只由事件决定：
//   · failed —— 任务终结且未成功（重试/后撤/连续失败全部用尽）
//   · error  —— 进程内终态，只在“连最小分配都失败”这类确定性证据下置位
// 能自恢复的故障一律只体现为 running + problem:true，
// 具体原因由 /local_planner/diagnostics 的 failures{} / last_reason 提供。
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
  case ReplanFailure::STUCK_GIVEUP:
    return "stuck_giveup";
  case ReplanFailure::EMERGENCY_EXIT:
    return "emergency_exit";
  case ReplanFailure::MEMORY_ERROR:
    return "memory_error";
  case ReplanFailure::UNKNOWN:
    return "unknown";
  }
  return "unknown";
}
} // namespace scan_planner

#endif // PLAN_ENV_REPLAN_FAILURE_H
