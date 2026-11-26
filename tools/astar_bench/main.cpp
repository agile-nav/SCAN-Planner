// ============================================================================
// 文件：tools/astar_bench/main.cpp —— A* 行为基准（不是测试）
//
// 【它做什么】
//   在真实障碍地图（tools/scenarios/*.pcd）上反复调用 AstarSearch，
//   统计 expanded 分布与失败率，用于"改动前后"对比。
//
// 【为什么不做断言】
//   expanded 不是稳定量：换地图、微调起终点、甚至堆在代价相同时的任意
//   弹出顺序都会让它浮动。把阈值写死会让测试偶发失败，最后被人注释掉。
//   需要守护性能时，用输出里的数字人工比对。
//
// 【用法】
//   tools/astar_bench/astar_bench.sh                      # 默认 corridor + 两组用例
//   tools/astar_bench/astar_bench.sh <pcd> <mode> <n>
//     mode: road | crossing
//
// 【输出】
//   case i ret=... path_pts=... start=(...) end=(...)
//   expanded=<n> ...（由 AstarSearch 的 verbose 日志打印）
//   # SUMMARY ok=.. fail=..
// ============================================================================
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>

namespace
{

// ---------------------------------------------------------------------------
// 建一个最小 GridMap：所有参数显式声明，不依赖任何 yaml。
//   圆柱参数与真机 a2.yaml 保持一致（r=0.25, offset=0.18），这样 expanded
//   的统计结果对真机有参考意义。
// ---------------------------------------------------------------------------
std::shared_ptr<rclcpp::Node> makeMapNode()
{
  auto node = std::make_shared<rclcpp::Node>("astar_bench");

  auto setd = [&](const std::string &n, double v) { node->declare_parameter<double>(n, v); };
  auto seti = [&](const std::string &n, int v) { node->declare_parameter<int>(n, v); };
  auto setb = [&](const std::string &n, bool v) { node->declare_parameter<bool>(n, v); };
  auto sets = [&](const std::string &n, const std::string &v) {
    node->declare_parameter<std::string>(n, v);
  };
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

  return node;
}

// 解析 PCD：仅支持本仓库 tools/scenarios 下的 ascii xyz 格式
std::vector<Eigen::Vector3d> loadPcd(const std::string &path)
{
  std::vector<Eigen::Vector3d> pts;
  std::ifstream f(path);
  if (!f)
  {
    fprintf(stderr, "无法打开 PCD: %s\n", path.c_str());
    return pts;
  }

  std::string line;
  bool in_data = false;
  while (std::getline(f, line))
  {
    if (!in_data)
    {
      if (line.rfind("DATA", 0) == 0)
        in_data = true;
      continue;
    }
    std::istringstream ss(line);
    double x, y, z;
    if (ss >> x >> y >> z)
      pts.emplace_back(x, y, z);
  }
  return pts;
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  const std::string pcd = (argc > 1) ? argv[1] : "tools/scenarios/corridor_obstacle.pcd";
  const std::string mode = (argc > 2) ? argv[2] : "road";
  const int repeats = (argc > 3) ? std::atoi(argv[3]) : 20;

  auto node = makeMapNode();
  auto map = std::make_shared<GridMap>();
  map->initMap(node.get());

  const double z = 0.30;
  const double res = map->getResolution();

  // 铺一层 20×20 m 已知空闲，再把 PCD 障碍压上去。
  map->resetBuffer();
  for (double x = -10.0; x <= 10.0; x += res)
    for (double y = -10.0; y <= 10.0; y += res)
      map->setOccupancy(Eigen::Vector3d(x, y, z), 0);

  const auto pts = loadPcd(pcd);
  printf("# pcd=%s pts=%zu mode=%s repeats=%d\n", pcd.c_str(), pts.size(),
         mode.c_str(), repeats);
  for (const auto &p : pts)
    map->setOccupied(Eigen::Vector3d(p(0), p(1), z));

  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(160, 160, 100));
  astar->setLogger(node->get_logger());

  int ok = 0, fail = 0;
  if (mode == "road")
  {
    // 沿参考路径 (-5.5,5.5) → (-5.5,-4.5) 分段搜索
    for (int i = 0; i < repeats; ++i)
    {
      const double y0 = 5.5 - 0.5 * (i % 4) - 0.1;
      const double y1 = y0 - 0.6;
      astar->setDiagContext(static_cast<uint64_t>(1000 + i), 0, true);
      const ASTAR_RET r =
          astar->AstarSearch(res, Eigen::Vector3d(-5.5, y0, z), Eigen::Vector3d(-5.5, y1, z));
      if (r == ASTAR_RET::SUCCESS)
        ++ok;
      else
        ++fail;
    }
  }
  else  // crossing：横穿带缺口的墙
  {
    for (int i = 0; i < repeats; ++i)
    {
      const double y = 1.0 - 0.60 + 0.03 * (i % 41);
      astar->setDiagContext(static_cast<uint64_t>(2000 + i), 0, true);
      const ASTAR_RET r =
          astar->AstarSearch(res, Eigen::Vector3d(-8.0, y, z), Eigen::Vector3d(-3.0, y, z));
      if (r == ASTAR_RET::SUCCESS)
        ++ok;
      else
        ++fail;
    }
  }

  printf("# SUMMARY ok=%d fail=%d total=%d rate=%.1f%%\n", ok, fail, repeats,
         repeats > 0 ? 100.0 * fail / repeats : 0.0);

  rclcpp::shutdown();
  return 0;
}
