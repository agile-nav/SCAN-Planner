// ============================================================================
// 文件：test/test_uniform_bspline.cpp —— B 样条的单元测试
//
// 【这个测试检查什么】
//   用最简单的例子验证 B 样条的数学实现是否正确：
//   1. 控制点排在一条直线上时，求值结果应该落在预期的点上；
//   2. 求导得到的速度应该是那条直线的斜率；
//   3~6. lengthenTime() 必须是"纯时间拉伸"：
//        · 总时长精确放大 ratio 倍（任意控制点数）
//        · 曲线几何逐点不变（法向偏差 < 1e-6）
//        · 速度精确降为 1/ratio
//        · 多次拉伸可组合
//      这四条曾经全部失败：旧实现只拉伸 [u_(5), u_(rows-6)]，把首尾各 5 个
//      节点留成原间距，节点向量变非均匀 → 曲线形状被改掉、速度也降不到位。
//
// 【怎么跑】  colcon test --packages-select bspline_opt
// ============================================================================
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>

// 测试 1：控制点在 x 方向等距排列时，检查 evaluateDeBoorT 求出的位置对不对。
TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

// 测试 2：检查 getDerivative() 求出的速度是否等于那条直线的斜率（x 方向应为 2.0）。
TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = scan_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}

namespace
{
// 构造一条"形状有起伏"的曲线，用来区分"改参数化"与"改形状"。
Eigen::MatrixXd makeCurvedControlPoints(int cols)
{
  Eigen::MatrixXd points(3, cols);
  for (int i = 0; i < cols; ++i)
  {
    const double s = static_cast<double>(i) / static_cast<double>(cols - 1);
    points(0, i) = s * 5.0;
    points(1, i) = std::sin(s * M_PI);
    points(2, i) = 0.3;
  }
  return points;
}
}  // namespace

// 测试 3：lengthenTime(r) 必须把总时长精确放大 r 倍。
//   旧实现固定拉伸 [u_(5), u_(rows-6)]，小控制点数下只拉到 1.2~1.5 倍；
//   新实现对任意控制点数都精确。
TEST(UniformBspline, LengthenTimeScalesDurationExactly)
{
  for (int cols : {9, 12, 15, 24, 40})
  {
    for (double ratio : {1.2, 1.6, 2.0})
    {
      scan_planner::UniformBspline spline(makeCurvedControlPoints(cols), 3, 0.1);
      const double before = spline.getTimeSum();
      ASSERT_GT(before, 0.0);

      spline.lengthenTime(ratio);
      const double after = spline.getTimeSum();

      EXPECT_NEAR(after / before, ratio, 1e-9)
          << "cols=" << cols << " ratio=" << ratio
          << " before=" << before << " after=" << after;
    }
  }
}

// 测试 4：lengthenTime 只允许改时间轴，不允许改曲线几何。
//   取拉伸后的曲线，量它到【原始整条曲线】的最近距离（法向偏差）：
//   真正的"纯时间拉伸"该值是 0；若只缩放内部节点会留下 p 个未缩放的
//   首尾幻影节点，节点向量变非均匀 → 曲线形状被改掉。
TEST(UniformBspline, LengthenTimePreservesGeometry)
{
  const int cols = 15;
  scan_planner::UniformBspline original(makeCurvedControlPoints(cols), 3, 0.1);
  const double dur0 = original.getTimeSum();

  // 原始曲线密集采样，作为"路径"真值
  std::vector<Eigen::Vector3d> ref;
  constexpr int kRefSamples = 2000;
  for (int i = 0; i <= kRefSamples; ++i)
    ref.push_back(original.evaluateDeBoorT(dur0 * i / kRefSamples));

  const double ratio = 1.6;
  scan_planner::UniformBspline stretched(makeCurvedControlPoints(cols), 3, 0.1);
  stretched.lengthenTime(ratio);
  const double dur1 = stretched.getTimeSum();

  double max_normal_deviation = 0.0;  // 到原曲线的最近距离：衡量形状是否改变
  double max_same_progress_deviation = 0.0;  // 同一归一化进度：衡量位置是否漂移
  constexpr int kSamples = 400;
  for (int i = 0; i <= kSamples; ++i)
  {
    const double s = static_cast<double>(i) / kSamples;
    const Eigen::Vector3d pt = stretched.evaluateDeBoorT(s * dur1);

    // 法向偏差 = 该点到【原始整条曲线】的最近距离（先对 ref 取 min，再对采样取 max）
    double nearest = std::numeric_limits<double>::max();
    for (const auto &q : ref)
      nearest = std::min(nearest, (pt - q).norm());
    max_normal_deviation = std::max(max_normal_deviation, nearest);

    max_same_progress_deviation =
        std::max(max_same_progress_deviation, (pt - original.evaluateDeBoorT(s * dur0)).norm());
  }

  // 几何必须逐点一致：1e-6 m 已经远大于浮点误差(1e-15)，也远小于任何真实形状改变
  EXPECT_LT(max_normal_deviation, 1e-6) << "拉伸改变了曲线形状（法向偏差过大）";
  EXPECT_LT(max_same_progress_deviation, 1e-6) << "同进度位置漂移过大";
}

// 测试 5：拉伸后速度必须精确降为 1/ratio。
//   这是"欠拉伸"问题在物理量上的直接体现：只缩放内部节点时，
//   速度只有降到约 0.68 而不是 0.5。
TEST(UniformBspline, LengthenTimeScalesVelocityByInverseRatio)
{
  const int cols = 24;
  const double ratio = 2.0;

  scan_planner::UniformBspline original(makeCurvedControlPoints(cols), 3, 0.1);
  scan_planner::UniformBspline stretched(makeCurvedControlPoints(cols), 3, 0.1);
  stretched.lengthenTime(ratio);

  auto max_speed = [](scan_planner::UniformBspline &spline) {
    auto vel = spline.getDerivative();
    const double dur = vel.getTimeSum();
    double vmax = 0.0;
    for (int i = 0; i <= 200; ++i)
      vmax = std::max(vmax, vel.evaluateDeBoorT(dur * i / 200.0).norm());
    return vmax;
  };

  const double v_before = max_speed(original);
  const double v_after = max_speed(stretched);

  ASSERT_GT(v_before, 1e-6);
  EXPECT_NEAR(v_after / v_before, 1.0 / ratio, 1e-6)
      << "v_before=" << v_before << " v_after=" << v_after;
}

// 测试 6：连续的多次拉伸必须可组合（时间轴线性可加）。
TEST(UniformBspline, LengthenTimeIsComposable)
{
  scan_planner::UniformBspline spline(makeCurvedControlPoints(15), 3, 0.1);
  const double dur0 = spline.getTimeSum();

  spline.lengthenTime(1.3);
  spline.lengthenTime(1.5);

  EXPECT_NEAR(spline.getTimeSum() / dur0, 1.3 * 1.5, 1e-9);
}
