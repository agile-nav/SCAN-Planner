#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "plan_manage/goal_degrade_utils.h"

using scan_planner::findNearestAcceptedOnRings;

namespace
{
constexpr double kStep = 0.05;

// 只在"距离圆点超过 min_dist"的区域接受候选点，用来模拟障碍
scan_planner::DegradeAcceptFn beyondRadius(double cx, double cy, double min_dist)
{
  return [cx, cy, min_dist](const Eigen::Vector3d &p) {
    return std::hypot(p(0) - cx, p(1) - cy) > min_dist;
  };
}
} // namespace

// 目标本身可行 → 直接用 goal，不扫描任何环
TEST(GoalDegradeUtils, AcceptsGoalItself)
{
  const Eigen::Vector3d goal(1.0, 2.0, 0.25);
  Eigen::Vector3d out;
  int calls = 0;
  const bool ok = findNearestAcceptedOnRings(
      goal, 1.0, kStep,
      [&](const Eigen::Vector3d &) {
        ++calls;
        return true;
      },
      out);
  EXPECT_TRUE(ok);
  EXPECT_EQ(calls, 1); // 只问了 goal 自己
  EXPECT_NEAR((out - goal).norm(), 0.0, 1e-12);
}

// 目标被占 → 返回最近的可行点，且距离不超过半径
TEST(GoalDegradeUtils, FindsNearestOutsideObstacle)
{
  const Eigen::Vector3d goal(0.0, 0.0, 0.25);
  Eigen::Vector3d out;
  const double blocked_r = 0.30;
  const bool ok =
      findNearestAcceptedOnRings(goal, 1.0, kStep, beyondRadius(0.0, 0.0, blocked_r), out);
  ASSERT_TRUE(ok);
  const double d = std::hypot(out(0), out(1));
  // 必须位于障碍圈之外，且足够接近边界（不能跳到很远的环上）
  EXPECT_GT(d, blocked_r);
  EXPECT_LT(d, blocked_r + 2.0 * kStep);
  EXPECT_NEAR(out(2), goal(2), 1e-12); // z 沿用目标高度
}

// 半径内全部被占 → 找不到就返回 false（不能悄悄返回一个远处的点）
TEST(GoalDegradeUtils, ReturnsFalseWhenNothingWithinRadius)
{
  const Eigen::Vector3d goal(0.0, 0.0, 0.25);
  Eigen::Vector3d out;
  EXPECT_FALSE(findNearestAcceptedOnRings(
      goal, 0.5, kStep,
      [](const Eigen::Vector3d &) {
        return false;
      },
      out));
}

// 非正半径 / 非正步长 → 直接失败，不做任何扫描
TEST(GoalDegradeUtils, RejectsNonPositiveRadiusOrStep)
{
  const Eigen::Vector3d goal(0.0, 0.0, 0.25);
  Eigen::Vector3d out;
  int calls = 0;
  auto accept = [&](const Eigen::Vector3d &) {
    ++calls;
    return true;
  };
  EXPECT_FALSE(findNearestAcceptedOnRings(goal, 0.0, kStep, accept, out));
  EXPECT_FALSE(findNearestAcceptedOnRings(goal, -1.0, kStep, accept, out));
  EXPECT_FALSE(findNearestAcceptedOnRings(goal, 1.0, 0.0, accept, out));
  EXPECT_EQ(calls, 0);
}

// 点密度：第 k 环取 max(8, 8k) 个点，保证远处环不会因点数不足而漏检
TEST(GoalDegradeUtils, RingDensityGrowsWithRadius)
{
  const Eigen::Vector3d goal(0.0, 0.0, 0.25);
  const double R = 0.3; // k_max = 6

  double max_ang_gap = 0.0;
  Eigen::Vector3d unused;
  const bool ok = findNearestAcceptedOnRings(
      goal, R, kStep,
      [](const Eigen::Vector3d &) {
        return false;
      },
      unused);
  EXPECT_FALSE(ok);

  // 统计相邻候选点之间的最大角间距
  const int k_max = static_cast<int>(std::ceil(R / kStep));
  for (int k = 1; k <= k_max; ++k)
  {
    const int samples = std::max(8, 8 * k);
    const double gap = 2.0 * M_PI / samples;
    if (gap > max_ang_gap)
      max_ang_gap = gap;
  }
  // 最大角间距应来自最小半径的环（8 个点 → π/4）
  EXPECT_NEAR(max_ang_gap, M_PI / 4.0, 1e-12);
  // 弧长间距在任何环上都不超过 8 个格子的宽度（0.4 m 量级）
  for (int k = 1; k <= k_max; ++k)
  {
    const int samples = std::max(8, 8 * k);
    const double arc = (2.0 * M_PI * kStep * k) / samples;
    EXPECT_LT(arc, 0.42);
  }
}

// 候选点的顺序必须"由近到远"：遍历顺序不能先给远处环
TEST(GoalDegradeUtils, VisitsRingsInIncreasingRadius)
{
  const Eigen::Vector3d goal(0.0, 0.0, 0.25);
  double last_r = -1.0;
  bool monotonic = true;
  Eigen::Vector3d out;
  findNearestAcceptedOnRings(
      goal, 0.3, kStep,
      [&](const Eigen::Vector3d &p) {
        const double r = std::hypot(p(0), p(1));
        if (r + 1e-9 < last_r)
          monotonic = false;
        last_r = r;
        return false;
      },
      out);
  EXPECT_TRUE(monotonic);
}
