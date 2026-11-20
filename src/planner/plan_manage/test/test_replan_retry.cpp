#include <gtest/gtest.h>

#include "plan_manage/replan_retry.h"

using scan_planner::replanRetryAllowed;
using scan_planner::replanRetryInterval;

TEST(ReplanRetry, BackoffSequence)
{
  EXPECT_NEAR(replanRetryInterval(0, 0.2, 1.0), 0.0, 1e-9);
  EXPECT_NEAR(replanRetryInterval(1, 0.2, 1.0), 0.2, 1e-9);
  EXPECT_NEAR(replanRetryInterval(2, 0.2, 1.0), 0.4, 1e-9);
  EXPECT_NEAR(replanRetryInterval(3, 0.2, 1.0), 0.8, 1e-9);
  EXPECT_NEAR(replanRetryInterval(4, 0.2, 1.0), 1.0, 1e-9);
  EXPECT_NEAR(replanRetryInterval(10, 0.2, 1.0), 1.0, 1e-9);
}

TEST(ReplanRetry, MinZeroMeansNoWait)
{
  EXPECT_NEAR(replanRetryInterval(1, 0.0, 1.0), 0.0, 1e-9);
  EXPECT_NEAR(replanRetryInterval(5, 0.0, 1.0), 0.0, 1e-9);
}

TEST(ReplanRetry, MaxZeroMeansNoCap)
{
  // max<=0 不封顶：0.1 * 2^3 = 0.8
  EXPECT_NEAR(replanRetryInterval(4, 0.1, 0.0), 0.8, 1e-9);
}

TEST(ReplanRetry, AllowedGate)
{
  // 关闭退避 -> 永远允许
  EXPECT_TRUE(replanRetryAllowed(false, 0.0, 5, 0.2, 1.0));
  // 第 1 次失败后等 0.2s
  EXPECT_FALSE(replanRetryAllowed(true, 0.19, 1, 0.2, 1.0));
  EXPECT_TRUE(replanRetryAllowed(true, 0.20, 1, 0.2, 1.0));
  // 第 2 次失败后等 0.4s
  EXPECT_FALSE(replanRetryAllowed(true, 0.39, 2, 0.2, 1.0));
  EXPECT_TRUE(replanRetryAllowed(true, 0.40, 2, 0.2, 1.0));
  // 封顶 1.0s
  EXPECT_FALSE(replanRetryAllowed(true, 0.99, 10, 0.2, 1.0));
  EXPECT_TRUE(replanRetryAllowed(true, 1.00, 10, 0.2, 1.0));
}
