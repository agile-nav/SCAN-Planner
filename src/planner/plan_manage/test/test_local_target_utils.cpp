#include <gtest/gtest.h>

#include "plan_manage/local_target_utils.h"

namespace
{
using scan_planner::AcceptFn;
using scan_planner::PositionFn;
using scan_planner::clampLocalHorizon;
using scan_planner::findFallbackTarget;
using scan_planner::findForwardTarget;
using scan_planner::projectOntoPath;
using scan_planner::targetFarEnough;

PositionFn linePath(const Eigen::Vector3d &start = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d &direction = Eigen::Vector3d::UnitY(),
                    double speed = 1.0)
{
  // 先把方向单位化再捕获，避免 Eigen 惰性表达式引用临时量。
  const Eigen::Vector3d unit = direction.normalized();
  return [start, unit, speed](double t) {
    return start + unit * speed * t;
  };
}

TEST(LocalTargetUtils, ClampsHorizonToLocalMap)
{
  EXPECT_NEAR(clampLocalHorizon(7.5, 5.0, 5.0, 0.25), 4.75, 1e-9);
  EXPECT_NEAR(clampLocalHorizon(7.5, 6.0, 6.0, 0.25), 5.75, 1e-9);
  EXPECT_NEAR(clampLocalHorizon(3.0, 5.0, 5.0, 0.25), 3.0, 1e-9);
}

TEST(LocalTargetUtils, ProjectsOntoNearestPathPoint)
{
  const auto path = linePath();
  const auto projection = projectOntoPath(path, 10.0, 0.05,
                                          Eigen::Vector3d(0.0, 3.0, 0.0));
  EXPECT_NEAR(projection.t_proj, 3.0, 0.06);
  EXPECT_NEAR(projection.min_dist, 0.0, 0.06);
}

TEST(LocalTargetUtils, ForwardTargetUsesArcLength)
{
  const auto path = linePath();
  const auto result = findForwardTarget(path, 10.0, 0.0, 4.5, 0.05,
                                        Eigen::Vector3d(0.0, 10.0, 0.0));
  ASSERT_TRUE(result.found);
  EXPECT_NEAR(result.t, 4.5, 0.06);
  EXPECT_NEAR(result.point.y(), 4.5, 0.06);
}

TEST(LocalTargetUtils, ForwardTargetFallsBackToEnd)
{
  const auto path = linePath();
  const auto end = Eigen::Vector3d(0.0, 10.0, 0.0);
  const auto too_far = findForwardTarget(path, 10.0, 0.0, 20.0, 0.05, end);
  EXPECT_FALSE(too_far.found);
  EXPECT_NEAR(too_far.t, 10.0, 1e-9);
  EXPECT_EQ(too_far.point, end);

  const auto near_end = findForwardTarget(path, 10.0, 8.0, 4.5, 0.05, end);
  EXPECT_FALSE(near_end.found);
  EXPECT_NEAR(near_end.t, 10.0, 1e-9);
}

TEST(LocalTargetUtils, FallbackRejectsTooClosePointExceptGoal)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  EXPECT_FALSE(targetFarEnough(Eigen::Vector3d(0.0, 0.2, 0.0), 1.0, 10.0,
                               start, 0.4));
  EXPECT_TRUE(targetFarEnough(Eigen::Vector3d(0.0, 0.5, 0.0), 1.0, 10.0,
                              start, 0.4));
  // 全局终点即使很近也允许
  EXPECT_TRUE(targetFarEnough(Eigen::Vector3d(0.0, 0.2, 0.0), 10.0, 10.0,
                              start, 0.4));
}

TEST(LocalTargetUtils, FallbackSearchesForwardThenBackward)
{
  const auto path = linePath();
  const auto end = Eigen::Vector3d(0.0, 10.0, 0.0);

  AcceptFn forward = [](const Eigen::Vector3d &, double t) {
    return t > 6.0 && t < 6.2;
  };
  const auto f = findFallbackTarget(path, 10.0, 0.0, 5.0, 0.1, forward, end);
  ASSERT_TRUE(f.found);
  EXPECT_GT(f.t, 6.0);
  EXPECT_LT(f.t, 6.2);

  AcceptFn backward = [](const Eigen::Vector3d &, double t) {
    return t > 4.0 && t < 4.5;
  };
  const auto b = findFallbackTarget(path, 10.0, 0.0, 5.0, 0.1, backward, end);
  ASSERT_TRUE(b.found);
  EXPECT_GT(b.t, 4.0);
  EXPECT_LT(b.t, 4.5);
}

TEST(LocalTargetUtils, FallbackReportsNoFreeTarget)
{
  const auto path = linePath();
  const auto end = Eigen::Vector3d(0.0, 10.0, 0.0);
  AcceptFn none = [](const Eigen::Vector3d &, double) { return false; };
  const auto result = findFallbackTarget(path, 10.0, 0.0, 5.0, 0.1, none, end);
  EXPECT_FALSE(result.found);
}

TEST(LocalTargetUtils, FallbackAllowsCloseGlobalGoal)
{
  const auto path = linePath();
  const auto end = Eigen::Vector3d(0.0, 10.0, 0.0);
  const Eigen::Vector3d start(0.0, 9.8, 0.0);
  AcceptFn end_only = [&](const Eigen::Vector3d &pt, double t) {
    return t >= 10.0 - 1e-3 &&
           scan_planner::targetFarEnough(pt, t, 10.0, start, 0.4);
  };
  const auto result = findFallbackTarget(path, 10.0, 0.0, 9.9, 0.1, end_only, end);
  ASSERT_TRUE(result.found);
  EXPECT_NEAR(result.t, 10.0, 1e-9);
}
}  // namespace
