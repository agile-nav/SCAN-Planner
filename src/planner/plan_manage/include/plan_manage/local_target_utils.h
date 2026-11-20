#ifndef PLAN_MANAGE_LOCAL_TARGET_UTILS_H
#define PLAN_MANAGE_LOCAL_TARGET_UTILS_H

// ============================================================================
// 文件：include/plan_manage/local_target_utils.h
//
// getLocalTarget 的纯算法部分：不依赖 ROS、TF、栅格地图和 FSM 状态。
// 这样可以用 gtest 直接覆盖“前视距离超图 / 前视点被占 / 兜底点太近”等分支。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include <Eigen/Core>

namespace scan_planner
{
using PositionFn = std::function<Eigen::Vector3d(double)>;
using AcceptFn = std::function<bool(const Eigen::Vector3d &, double)>;

// 前视距离夹紧：不能超过局部地图可信范围（min(range) - 机器人半径）。
// 保持原 getLocalTarget 的行为：只有 usable > min_horizon 时才夹紧。
inline double clampLocalHorizon(double requested, double range_x, double range_y,
                                double radius, double min_horizon = 0.5)
{
  double horizon = requested;
  const double usable = std::min(range_x, range_y) - radius;
  if (usable > min_horizon)
    horizon = std::min(horizon, usable);
  return std::max(min_horizon, horizon);
}

struct PathProjection
{
  double t_proj{0.0};
  double min_dist{std::numeric_limits<double>::infinity()};
};

// 沿全局路径找到离起点最近的采样时刻，避免轨迹回绕时取错段。
inline PathProjection projectOntoPath(const PositionFn &path, double duration,
                                      double t_step, const Eigen::Vector3d &start)
{
  PathProjection result;
  if (!path || t_step <= 0.0 || duration <= 0.0)
    return result;
  for (double t = 0.0; t < duration; t += t_step)
  {
    const double dist = (path(t) - start).norm();
    if (dist < result.min_dist)
    {
      result.min_dist = dist;
      result.t_proj = t;
    }
  }
  return result;
}

struct LocalTarget
{
  bool found{false};
  double t{0.0};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
};

// 从 t_proj 沿路径前进 horizon 弧长，取该点作为局部目标。
// 若走完整条路径仍不够 horizon，返回 found=false，point=end_point，t=duration。
inline LocalTarget findForwardTarget(const PositionFn &path, double duration,
                                     double t_proj, double horizon, double t_step,
                                     const Eigen::Vector3d &end_point)
{
  LocalTarget result;
  result.t = duration;
  result.point = end_point;
  if (!path || t_step <= 0.0 || duration <= t_proj)
    return result;

  double total_dist = 0.0;
  Eigen::Vector3d prev = path(t_proj);
  for (double t = t_proj; t < duration; t += t_step)
  {
    const Eigen::Vector3d pos = path(t);
    total_dist += (pos - prev).norm();
    if (total_dist >= horizon)
    {
      result.found = true;
      result.t = t;
      result.point = pos;
      return result;
    }
    prev = pos;
  }
  return result;
}

// 替代点不能贴着机器人；全局终点例外（即使很近也是合法到达）。
inline bool targetFarEnough(const Eigen::Vector3d &point, double t, double duration,
                            const Eigen::Vector3d &start, double min_target_dist)
{
  if (t >= duration - 1e-3)
    return true;
  return (point - start).head<2>().norm() >= min_target_dist;
}

// 前视点被占时，向前/向后逐步找可接受的替代点。
// 向后搜索不能越过 t_proj；返回 found=false 表示 no_free_target。
inline LocalTarget findFallbackTarget(const PositionFn &path, double duration,
                                      double t_proj, double target_t, double t_step,
                                      const AcceptFn &acceptable,
                                      const Eigen::Vector3d &end_point)
{
  LocalTarget result;
  result.t = target_t;
  result.point = end_point;
  if (!path || !acceptable || t_step <= 0.0)
    return result;

  for (double dt = 0.0; dt <= duration; dt += t_step)
  {
    const double t_forward = target_t + dt;
    if (t_forward <= duration)
    {
      const Eigen::Vector3d pt = path(t_forward);
      if (acceptable(pt, t_forward))
      {
        result.found = true;
        result.t = t_forward;
        result.point = pt;
        return result;
      }
    }

    const double t_backward = target_t - dt;
    if (t_backward >= std::max(0.0, t_proj))
    {
      const Eigen::Vector3d pt = path(t_backward);
      if (acceptable(pt, t_backward))
      {
        result.found = true;
        result.t = t_backward;
        result.point = pt;
        return result;
      }
    }
  }
  return result;
}
}  // namespace scan_planner

#endif  // PLAN_MANAGE_LOCAL_TARGET_UTILS_H
