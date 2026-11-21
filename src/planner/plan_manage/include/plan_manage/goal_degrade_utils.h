#ifndef PLAN_MANAGE_GOAL_DEGRADE_UTILS_H
#define PLAN_MANAGE_GOAL_DEGRADE_UTILS_H

// ============================================================================
// 文件：include/plan_manage/goal_degrade_utils.h
//
// 目标降级里的纯几何部分：以 goal 为圆心、由近到远生成候选点。
//   不依赖 ROS / 地图 / A*，便于用 gtest 直接覆盖"环数、点密度、顺序"。
//   真正的"这个点能不能站 / 走不走得过去"由调用方判定（判据进 accept 回调）。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <functional>

#include <Eigen/Core>

namespace scan_planner
{

using DegradeAcceptFn = std::function<bool(const Eigen::Vector3d &)>;

// 以 goal 为中心、由近到远扫描同心环，返回第一个被 accept 接受的点。
//   · 第 0 环就是 goal 本身
//   · 第 k 环半径为 k*step，取 max(8, 8k) 个点（保证点密度与半径成正比，
//     避免远处环上相邻点间距过大而漏掉唯一可行点）
//   · z 沿用 goal 高度：地面机器人降级只需换水平位置
//   · max_radius <= 0 或 step <= 0 → 直接返回 false（不扫描）
//
// 返回 true 表示找到，结果写入 out。
inline bool findNearestAcceptedOnRings(const Eigen::Vector3d &goal, double max_radius, double step,
                                       const DegradeAcceptFn &accept, Eigen::Vector3d &out)
{
  if (!(max_radius > 0.0) || !(step > 0.0))
    return false;

  if (accept(goal))
  {
    out = goal;
    return true;
  }

  const int k_max = std::max(1, static_cast<int>(std::ceil(max_radius / step)));
  for (int k = 1; k <= k_max; ++k)
  {
    const int samples = std::max(8, 8 * k);
    for (int i = 0; i < samples; ++i)
    {
      const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples);
      Eigen::Vector3d cand = goal;
      cand(0) += std::cos(ang) * step * k;
      cand(1) += std::sin(ang) * step * k;
      if (!accept(cand))
        continue;
      out = cand;
      return true;
    }
  }
  return false;
}

} // namespace scan_planner

#endif // PLAN_MANAGE_GOAL_DEGRADE_UTILS_H
