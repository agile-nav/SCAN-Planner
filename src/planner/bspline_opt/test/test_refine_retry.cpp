// ============================================================================
// 文件：test/test_refine_retry.cpp
//
// 【测什么】
//   BsplineOptimizer::refine_optimize() 的"不安全就加重贴合权重再试一次"逻辑。
//
// 【背景】
//   原实现写成：
//       int iter_count = 0;
//       do {
//           ...
//           iter_count++;
//       } while (!flag_safe && iter_count <= 0);
//   iter_count 在判断前已自增到 1，`1 <= 0` 恒假 —— 重试从未发生，
//   `lambda4_ *= 2` 随即被还原，等于死代码。
//
// 【怎么验证「重试真的发生了」】
//   直接观察 lambda4 翻倍是没有意义的（它在函数出口就被还原）。
//   改为观察 lastRefineRounds()：
//       1 = 首轮即安全（没有重试）
//       2 = 首轮不安全，用加倍后的 lambda4 又跑了一轮
//
//   为了让首轮"不安全"，构造一条附近有障碍的轨迹；为了让重试"有意义"，
//   参考点取在无障碍处，使贴合项能把轨迹拉回安全区。
//
// 【怎么跑】
//   colcon test --packages-select bspline_opt
// ============================================================================
#include <gtest/gtest.h>

#include <memory>

#include <Eigen/Core>
#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>

namespace
{

// 建一张只用于碰撞查询的地图：全部已知空闲，除了一条 x 方向的障碍带。
//   GridMap::initMap 需要节点，但 refine_optimize 只用到 getResolution /
//   getInflateOccupancy，因此这里走公开的 setOccupancy 接口构造场景。
struct RefineFixture
{
  std::shared_ptr<rclcpp::Node> node;
  GridMap::Ptr map;
  std::unique_ptr<scan_planner::BsplineOptimizer> opt;
};

RefineFixture makeFixture(const std::string &name)
{
  auto node = std::make_shared<rclcpp::Node>(name);

  auto setd = [&](const std::string &n, double v) { node->declare_parameter<double>(n, v); };
  auto seti = [&](const std::string &n, int v) { node->declare_parameter<int>(n, v); };
  auto setb = [&](const std::string &n, bool v) { node->declare_parameter<bool>(n, v); };
  auto sets = [&](const std::string &n, const std::string &v) { node->declare_parameter<std::string>(n, v); };
  auto setv = [&](const std::string &n, const std::vector<double> &v) {
    node->declare_parameter<std::vector<double>>(n, v);
  };

  setd("grid_map.resolution", 0.05);
  setd("grid_map.sliding_map_size_x", 20.0);
  setd("grid_map.sliding_map_size_y", 20.0);
  setd("grid_map.sliding_map_size_z", 5.0);
  setd("grid_map.local_update_range_x", 10.0);
  setd("grid_map.local_update_range_y", 10.0);
  setd("grid_map.local_update_range_z", 2.5);
  setd("grid_map.obstacles_inflation_z_up", 0.1);
  setd("grid_map.obstacles_inflation_z_down", 0.4);
  setd("grid_map.double_cylinder_radius", 0.25);
  setd("grid_map.double_cylinder_offset", 0.18);
  setb("grid_map.tilt_collision_enable", false);
  setd("grid_map.self_filter_radius", 0.0);
  setb("grid_map.map_sliding_en", false);
  setd("grid_map.map_sliding_thresh", 0.2);
  setd("grid_map.fx", 387.229);
  setd("grid_map.fy", 387.229);
  setd("grid_map.cx", 321.046);
  setd("grid_map.cy", 243.450);
  setd("grid_map.depth_filter_maxdist", 3.0);
  setd("grid_map.depth_filter_mindist", 0.3);
  seti("grid_map.depth_filter_margin", 1);
  setd("grid_map.k_depth_scaling_factor", 1000.0);
  seti("grid_map.skip_pixel", 2);
  setd("grid_map.p_hit", 0.85);
  setd("grid_map.p_miss", 0.30);
  setd("grid_map.p_min", 0.12);
  setd("grid_map.p_max", 0.98);
  setd("grid_map.p_occ", 0.80);
  setd("grid_map.max_ray_length", 5.0);
  setd("grid_map.vis_height", 0.3);
  setb("grid_map.show_occ_time", false);
  sets("grid_map.frame_id", "world");
  sets("grid_map.sliding_map_frame_id", "sliding_map");
  setd("grid_map.ground_height", 0.0);
  sets("grid_map.sensor_type", "lidar");
  setb("grid_map.cloud_is_world", true);
  setb("grid_map.need_extrinsic", true);
  setv("grid_map.lidar_extrinsic_xyz", {0.0, 0.0, 0.0});
  setv("grid_map.lidar_extrinsic_rpy", {0.0, 0.0, 0.0});
  setv("grid_map.depth_extrinsic_xyz", {0.0, 0.0, 0.0});
  setv("grid_map.depth_extrinsic_rpy", {0.0, 0.0, 0.0});

  // 优化器参数
  setd("optimization.lambda_smooth", 1.0);
  setd("optimization.lambda_collision", 1.0);
  setd("optimization.lambda_feasibility", 0.1);
  setd("optimization.lambda_fitness", 1.0);
  setd("optimization.dist0", 0.2);
  setd("optimization.max_vel", 0.75);
  setd("optimization.max_acc", 1.5);
  seti("optimization.order", 3);
  setb("optimization.rebound_fallback_enable", true);

  auto map = std::make_shared<GridMap>();
  map->initMap(node.get());

  auto opt = std::make_unique<scan_planner::BsplineOptimizer>();
  opt->setParam(node.get());
  opt->setEnvironment(map);
  opt->setLogger(node->get_logger());

  return RefineFixture{node, map, std::move(opt)};
}

// 把一块区域标成已知空闲
void markFree(GridMap &map, double x0, double x1, double y0, double y1, double z)
{
  for (double x = x0; x <= x1 + 1e-9; x += 0.05)
    for (double y = y0; y <= y1 + 1e-9; y += 0.05)
      map.setOccupancy(Eigen::Vector3d(x, y, z), 0);
}

// 用一条沿 x 的直线构造控制点与参考点
Eigen::MatrixXd straightLine(int cols, double y, double z, double dx)
{
  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(3, cols);
  for (int i = 0; i < cols; ++i)
  {
    p(0, i) = i * dx;
    p(1, i) = y;
    p(2, i) = z;
  }
  return p;
}

}  // namespace

// ① 无障碍：首轮就安全，不应触发重试
TEST(RefineRetry, SafeOnFirstRound)
{
  auto fx = makeFixture("refine_retry_safe");
  fx.map->resetBuffer();
  markFree(*fx.map, -5.0, 10.0, -2.0, 2.0, 0.30);

  Eigen::MatrixXd cps = straightLine(10, 0.0, 0.30, 0.20);

  // 参考点取在原轨迹上，贴合项无冲突
  fx.opt->ref_pts_.clear();
  for (int i = 0; i < cps.cols(); ++i)
    fx.opt->ref_pts_.push_back(cps.col(i));

  Eigen::MatrixXd out;
  const bool ok = fx.opt->BsplineOptimizeTrajRefine(cps, 0.10, out);

  EXPECT_TRUE(ok);
  EXPECT_EQ(fx.opt->lastRefineRounds(), 1)
      << "无障碍时应首轮通过，不应进入重试";
}

// ② 参考点安全、初始控制点落在障碍上：首轮不安全 → 应重试一轮
TEST(RefineRetry, RetriesOnceWhenUnsafe)
{
  auto fx = makeFixture("refine_retry_unsafe");

  // 场景：y=0 一带是障碍，y=1.5 一带空闲。
  fx.map->resetBuffer();
  markFree(*fx.map, -5.0, 10.0, -2.0, 2.0, 0.30);
  // 沿 x 造一条宽障碍带（覆盖初始控制点所在 y=0.95 附近）
  for (double x = -5.0; x <= 10.0; x += 0.05)
    for (double y = 0.80; y <= 1.10; y += 0.05)
      fx.map->setOccupied(Eigen::Vector3d(x, y, 0.30));

  // 初始控制点穿过障碍带
  Eigen::MatrixXd cps = straightLine(10, 0.95, 0.30, 0.20);

  // 参考点放在安全区（y=1.5），贴合项在加权后应把轨迹拉出障碍
  fx.opt->ref_pts_.clear();
  for (int i = 0; i < cps.cols(); ++i)
  {
    Eigen::Vector3d p = cps.col(i);
    p(1) = 1.5;                       // 安全位置
    fx.opt->ref_pts_.push_back(p);
  }

  Eigen::MatrixXd out;
  fx.opt->BsplineOptimizeTrajRefine(cps, 0.10, out);

  // 关键断言：必须走过 2 轮（首轮不安全 + 加权重试）
  EXPECT_EQ(fx.opt->lastRefineRounds(), 2)
      << "首轮不安全时必须触发一次重试；若为 1 说明重试逻辑仍是死代码";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
