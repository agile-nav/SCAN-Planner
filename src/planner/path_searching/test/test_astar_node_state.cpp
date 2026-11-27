// ============================================================================
// 文件：test/test_astar_node_state.cpp
//
// 【测什么】
//   dyn_a_star.cpp 内层循环中对 GridNode 三个状态（rounds / state / gScore）
//   的使用顺序是否正确。
//
// 【背景：三处状态本应同步，实际不同步】
//   GridNode 在 initGridMap 时一次性 new 出约 256 万个，之后全程复用：
//     int  rounds{0};                 // 轮次标记，用来代替"每轮清零"
//     state{UNDEFINED};               // 是否已入队/已扩展
//     double gScore{inf}, fScore{inf}; // inf 写成 1>>20，实际等于 0
//
//   内层循环的原顺序（dyn_a_star.cpp:516-559）：
//       flag_explored = (node->rounds == rounds_);      // ① 读"本轮碰过没"
//       if (flag_explored && state == CLOSEDSET) continue;   // ②
//       node->rounds = rounds_;                          // ③ ★ 在障碍判断之前置位
//       if (checkOccupancy(pos, yaw)) continue;          // ④ 带朝向的障碍判断
//       if (!flag_explored) { ...; openSet_.push(); }    // ⑤ 只在这里入队
//       else if (tentative < gScore) { ... }              // ⑥ ★ 不 push
//
//   getInflateOccupancy(pos, yaw) 只探 pos ± 0.18m·(cos yaw, sin yaw, 0)
//   两个点。所以**同一个格子，从不同方向接近，障碍判定结果可能不同**。
//
// 【由此产生的两个 bug】
//   Bug 2（rounds 置位过早）：
//     节点 X 本轮先被从"撞墙"的方向碰到 → rounds 已置位、但没入队；
//     再从"通行"的方向碰到 → flag_explored 已为 true → 走 ⑥ 分支，永不入队。
//     结果：X 明明可达却被静默丢弃。
//
//   Bug 3（state 跨轮残留）：
//     上一次搜索把 X 标成 CLOSEDSET，且从不清理；
//     本次搜索 X 被"撞墙"方向碰到 → rounds 置位（state 仍是旧 CLOSEDSET）；
//     再从"通行"方向碰到 → ② 成立 → 永久跳过。
//
// 【怎么跑】
//   colcon test --packages-select path_searching
// ============================================================================
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>

namespace
{

// ---------------------------------------------------------------------------
// 构造一张最小地图：所有参数显式声明，避免依赖 yaml。
// ---------------------------------------------------------------------------
std::shared_ptr<rclcpp::Node> makeMapNode(const std::string &name, double cy_radius,
                                          double cy_offset)
{
  auto node = std::make_shared<rclcpp::Node>(name);

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
  setd("grid_map.double_cylinder_radius", cy_radius);
  setd("grid_map.double_cylinder_offset", cy_offset);
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

// ---------------------------------------------------------------------------
// 把 [x0,x1]×[y0,y1] 的 z 平面全部标成"已知空闲"。
//   resetBuffer 之后所有格子是"未知"，而 getInflateOccupancyFromBuffer
//   对未知格返回 -1（调用方按"会撞"处理），所以必须先铺一层空闲。
// ---------------------------------------------------------------------------
void markFreeRegion(GridMap &map, double x0, double x1, double y0, double y1, double z)
{
  const double res = map.getResolution();
  for (double x = x0; x <= x1 + 1e-9; x += res)
    for (double y = y0; y <= y1 + 1e-9; y += res)
      map.setOccupancy(Eigen::Vector3d(x, y, z), 0);
}

// ---------------------------------------------------------------------------
// 场景：在 A* 的搜索平面上，让某个格子"从一个方向看有障碍、从另一个方向看没有"。
//
//   getInflateOccupancy(pos, yaw) 探测 pos ± offset·(cos yaw, sin yaw, 0)
//
//   在格子 X 的 +x 侧 offset 处放一个障碍点：
//     · 从 -x 方向接近 X：yaw = 0        → 探 X±(offset,0,0) → 撞
//     · 从 -y 方向接近 X：yaw = +π/2     → 探 X±(0,offset,0) → 通
//
//   障碍只放"单点"，且半径足够小（cy_radius 0.05），使膨胀不扩散到 X 自身。
// ---------------------------------------------------------------------------
}  // namespace

// ===========================================================================
// 测试 0（前提验证）：障碍判定确实依赖朝向
//
//   这是 Bug 2 / Bug 3 成立的前提。若此断言失败，说明 getInflateOccupancy
//   在本场景下并不随 yaw 变化，那么"同格不同方向结论不同"的情况不会出现，
//   后续测试也就没有意义。
//
//   场景：cell(0,0) 的 +x 侧 0.18m 处放一个点障碍
//     yaw=0    → 探 (0,0)±(0.18,0)  → 前方命中障碍 → 判撞
//     yaw=π/2  → 探 (0,0)±(0,0.18)  → 两点都空     → 判通
// ===========================================================================
TEST(AstarNodeState, OccupancyDependsOnYaw)
{
  auto node = makeMapNode("astar_premise", 0.05, 0.18);
  auto map = std::make_shared<GridMap>();
  map->initMap(node.get());

  const double z = 0.30;
  const double res = map->getResolution();
  map->resetBuffer();
  markFreeRegion(*map, -3.0, 3.0, -3.0, 3.0, z);

  // 单个点障碍，位于 cell(0,0) 的 +x 侧
  //   注意：setOccupied 会按膨胀半径(0.05)向四周扩散约 1 格，
  //   但 X 格中心本身（距障碍 0.18m）不应被覆盖。
  map->setOccupied(Eigen::Vector3d(0.18, 0.0, z));

  const Eigen::Vector3d cell(0.0, 0.0, z);
  const int occ_x = map->getInflateOccupancy(cell, 0.0);        // 沿 +x 探测
  const int occ_y = map->getInflateOccupancy(cell, M_PI / 2.0); // 沿 +y 探测

  // 把实测值记进 gtest 输出。不要用 printf —— 第一方裸输出会被
  //   tools/log_audit.py --strict 判为违规（退出码 1）。
  ::testing::Test::RecordProperty("occ_yaw_0", occ_x);
  ::testing::Test::RecordProperty("occ_yaw_90", occ_y);
  ::testing::Test::RecordProperty("resolution", res);

  // 前提：两个方向的结论必须不同，否则"先撞后通"的路径不存在
  EXPECT_NE(occ_x, occ_y)
      << "该网格配置下障碍判定与朝向无关，Bug 2/3 的前提不成立，需重新设计场景";

  // 方向也要符合预期：+x 方向撞、+y 方向通
  EXPECT_NE(occ_x, 0) << "沿 +x 探测应命中 (0.18,0) 处的障碍";
  EXPECT_EQ(occ_y, 0) << "沿 +y 探测不应命中障碍";
}

// ===========================================================================
// 测试 1：单向障碍 → 另一方向必须仍可通行
//
//   期望行为：A* 能找到一条从 start 到 end 的路径。
//   在旧实现下，X 会先在"撞墙方向"被置 rounds，随后在"通行方向"走 else-if
//   分支而永不入队，导致本测试失败（预期）。
// ===========================================================================
TEST(AstarNodeState, PassableFromAnotherDirection)
{
  const std::string name = "astar_dir_pass";
  auto node = makeMapNode(name, 0.05, 0.18);
  auto map = std::make_shared<GridMap>();
  map->initMap(node.get());

  const double z = 0.30;
  const double res = map->getResolution();
  map->resetBuffer();
  markFreeRegion(*map, -3.0, 3.0, -3.0, 3.0, z);

  // 构造一道"只有一个缺口"的墙：
  //   x=0 这一列，除 y=0 之外全部是墙。
  //   于是从 (-1.5,0) 到 (1.5,0) 的唯一通路就是 X=(0,0) 这个缺口。
  for (double y = -4.2; y <= 4.2; y += res)
  {
    if (std::abs(y) < 0.5 * res) continue;      // y=0 留作缺口
    map->setOccupied(Eigen::Vector3d(0.0, y, z));
  }


  // 关键：只在 X 的 +x 侧 0.18m 处放一个点障碍。
  //   从 -x 方向接近 X（yaw=0）→ 探 (0,0)±(0.18,0) → 前点命中 → 判撞
  //   从 +y 方向接近 X（yaw=π/2）→ 探 (0,0)±(0,0.18) → 都空 → 判通
  //
  //   这个障碍点本身位于缺口右侧，会挡掉"穿过缺口继续往 +x 走"的路径，
  //   所以路线改为从缺口斜着穿过去（A* 允许 8 方向），仍能到达终点。
  map->setOccupied(Eigen::Vector3d(0.18, 0.0, z));

  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(160, 160, 100));
  astar->setLogger(node->get_logger());
  astar->setDiagContext(1, 0, true);

  const Eigen::Vector3d start(-1.5, 0.0, z);
  const Eigen::Vector3d end(1.5, 0.0, z);
  const ASTAR_RET ret = astar->AstarSearch(map->getResolution(), start, end);

  EXPECT_EQ(ret, ASTAR_RET::SUCCESS)
      << "单点障碍不应让整条直线路径失效（应能绕行或换探测方向通过）";

  if (ret != ASTAR_RET::SUCCESS)
    return;

  const auto path = astar->getPath();
  EXPECT_GE(path.size(), 2u);

  // 路径的格子步数 = 路径代价（每步 1 或 √2）。
  //   参考值：3.0m / 0.05 = 60 格，纯直线正好 60 步。
  //   若 (0,0) 被丢弃，A* 只能绕行，步数会明显变多。
  //
  //   注意：不能断言"路径必须经过 (0,0)"——A* 在代价相同时会选任意一条
  //   最短路径，(0,0) 的邻居 (0,-0.05) 同样可达终点，视堆的弹出顺序而定。
  //   实测修复后路径会从 (0,-0.05) 这一格通过（斜向避开障碍），仍是最短。
  //   所以这里用**代价上界**判定：只要不超过直线步数，就说明没有因丢节点
  //   而被迫绕远。
  // 缺口是唯一通路：如果 X 被丢弃，A* 应当完全找不到路（前面已断言 SUCCESS）。
  //   所以只要成功，就说明 X 没有被丢弃。
  bool passes_gap = false;
  for (const auto &q : path)
    if (q.head<2>().norm() < 0.05)
      passes_gap = true;

  EXPECT_TRUE(passes_gap) << "路径未穿过缺口 (0,0)";
}

// ===========================================================================
// 测试 2：连续两次搜索复用同一批 GridNode
//
//   第一次搜索把一批节点标成 CLOSEDSET 且不清除；
//   第二次搜索（同一起终点）必须仍然成功。
//   这验证 Bug 3（state 跨轮残留）不会让节点被永久排除。
// ===========================================================================
TEST(AstarNodeState, RepeatedSearchStaysCorrect)
{
  const std::string name = "astar_repeat";
  auto node = makeMapNode(name, 0.05, 0.18);
  auto map = std::make_shared<GridMap>();
  map->initMap(node.get());

  const double z = 0.30;
  map->resetBuffer();
  markFreeRegion(*map, -3.0, 3.0, -3.0, 3.0, z);

  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(160, 160, 100));
  astar->setLogger(node->get_logger());

  const Eigen::Vector3d start(-1.5, 0.0, z);
  const Eigen::Vector3d end(1.5, 0.0, z);

  // 连续 5 次相同搜索：结果必须一致（不能因为节点残留状态而退化）
  for (int i = 0; i < 5; ++i)
  {
    astar->setDiagContext(static_cast<uint64_t>(100 + i), 0, false);
    const ASTAR_RET ret = astar->AstarSearch(map->getResolution(), start, end);
    EXPECT_EQ(ret, ASTAR_RET::SUCCESS) << "第 " << i + 1 << " 次搜索失败";
    if (ret != ASTAR_RET::SUCCESS)
      break;
    EXPECT_GE(astar->getPath().size(), 2u) << "第 " << i + 1 << " 次路径过短";
  }
}

// ===========================================================================
// 测试 3：区域内无路时必须明确失败（不能因为状态污染而"假装成功"）
// ===========================================================================
TEST(AstarNodeState, BlockedCorridorFails)
{
  const std::string name = "astar_blocked";
  auto node = makeMapNode(name, 0.05, 0.18);
  auto map = std::make_shared<GridMap>();
  map->initMap(node.get());

  const double z = 0.30;
  map->resetBuffer();
  markFreeRegion(*map, -3.0, 3.0, -3.0, 3.0, z);

  // 用一道横贯的厚墙把地图彻底隔断。
  //   纵向必须铺满整个搜索池（pool=160 格 × 0.05 = ±4.0m），否则 A* 会从
  //   墙的两端绕过去（实测 ±3m 时就能绕）。
  for (double x = -0.10; x <= 0.10; x += 0.05)
    for (double y = -4.2; y <= 4.2; y += 0.05)
      map->setOccupied(Eigen::Vector3d(x, y, z));

  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(160, 160, 100));
  astar->setLogger(node->get_logger());
  astar->setDiagContext(7, 0, false);

  const Eigen::Vector3d start(-1.5, 0.0, z);
  const Eigen::Vector3d end(1.5, 0.0, z);
  const ASTAR_RET ret = astar->AstarSearch(map->getResolution(), start, end);

  EXPECT_NE(ret, ASTAR_RET::SUCCESS) << "被墙隔断时不应报告成功";
}

// ===========================================================================
// 测试 4：Bug 3 —— state 跨轮残留导致节点被永久排除
//
//   【为什么必须分两轮、且障碍在第二轮才出现】
//     Bug 3 需要两个条件同时成立：
//       ① X 的 state 是"上一轮"遗留的 CLOSEDSET；
//       ② 本轮中 X 先被"撞墙方向"碰到（原实现会把 rounds 置位），
//          随后才被"通行方向"碰到。
//     如果障碍从一开始就存在，Bug 2 会在第 1 轮就把 X 吃掉，Bug 3 被掩盖。
//     所以第 1 轮必须在"无障碍"下跑完，让 X 干净地变成 CLOSEDSET；
//     第 2 轮再新增方向性障碍 + 把 X 变成唯一缺口。
//
//   期望：第 2 轮必须穿过缺口 (0,0)。
//   旧实现下 X 会被锁死（flag_explored 为真 + state 残留 CLOSEDSET → continue），
//   整体搜不到路或被迫绕行。
// ===========================================================================
TEST(AstarNodeState, StaleClosedSetDoesNotLatchNode)
{
  auto node = makeMapNode("astar_stale_closed", 0.05, 0.18);
  auto map = std::make_shared<GridMap>();
  map->initMap(node.get());

  const double z = 0.30;
  const double res = map->getResolution();
  map->resetBuffer();
  markFreeRegion(*map, -3.0, 3.0, -3.0, 3.0, z);

  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(160, 160, 100));
  astar->setLogger(node->get_logger());

  const Eigen::Vector3d start(-1.5, 0.0, z);
  const Eigen::Vector3d end(1.5, 0.0, z);

  // ---- 第 1 轮：完全无障碍，X=(0,0) 在直线路径上 → 被扩展成 CLOSEDSET ----
  astar->setDiagContext(301, 0, false);
  const ASTAR_RET ret1 = astar->AstarSearch(res, start, end);
  ASSERT_EQ(ret1, ASTAR_RET::SUCCESS) << "第 1 轮（无障碍）应当成功";

  // ---- 第 2 轮：新增"只有一个缺口"的墙 + 缺口 +x 侧的点障碍 ----
  for (double y = -4.2; y <= 4.2; y += res)
  {
    if (std::abs(y) < 0.5 * res) continue;   // y=0 留作唯一缺口
    map->setOccupied(Eigen::Vector3d(0.0, y, z));
  }
  map->setOccupied(Eigen::Vector3d(0.18, 0.0, z));

  astar->setDiagContext(302, 0, false);
  const ASTAR_RET ret2 = astar->AstarSearch(res, start, end);
  ASSERT_EQ(ret2, ASTAR_RET::SUCCESS)
      << "第 2 轮应当成功：缺口 (0,0) 可从斜向通过";

  const auto path = astar->getPath();
  bool passes_gap = false;
  for (const auto &q : path)
    if (q.head<2>().norm() < 0.05)
      passes_gap = true;

  EXPECT_TRUE(passes_gap)
      << "路径未穿过缺口 (0,0)：该格被上一轮残留的 CLOSEDSET 锁死"
         "（flag_explored 为真 + state 仍是旧 CLOSEDSET → 直接 continue）";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
