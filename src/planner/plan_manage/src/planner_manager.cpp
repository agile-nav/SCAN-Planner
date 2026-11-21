// ============================================================================
// 文件：src/planner/plan_manage/src/planner_manager.cpp
//
// 【这个文件是干什么的】
//   规划总管 SCANPlannerManager 的实现代码。整个系统“怎么算出一条轨迹”
//   都写在这里。最核心的函数是 reboundReplan()。
//
// 【reboundReplan() 里的五步流水线】（详细注释见函数内部）
//   1. 造一条初始猜测路径（多项式）
//   2. 把 z 拉平（四足走平地，高度按弧长线性变化）
//   3. 把一串点拟合成 B 样条（得到控制点）
//   4. 找出会碰障碍的段，用 A* 算出“往哪边推”的方向
//   5. 用 L-BFGS 优化控制点，得到平滑、无碰撞、满足速度限制的轨迹
//   最后：要是超速了就把时间拉长；仍不安全就整条丢弃不发布。
//
// 【建议阅读顺序】
//   initPlanModules() → reboundReplan() → updateTrajInfo() → 其它辅助函数
// ============================================================================
// #include <fstream>
// 依赖：本包的头文件（声明）+ std::chrono（计时，用来统计各阶段耗时）
#include <plan_manage/planner_manager.h>
#include <chrono>
#include <thread>
#include <plan_manage/goal_degrade_utils.h>

namespace scan_planner
{
  // 匿名命名空间：下面的函数只在本文件内部使用，外部看不见
  namespace
  {
    // 把路径上每个点的高度 z 重新赋值：从 start_z 到 target_z 按“走过的水平距离”线性变化。
    // 目的：四足在平地上行走，高度不该忽高忽低，统一成一条平缓的参考高度。
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      // 情况1：路径是空的 → 直接返回，防止后面越界访问
      if (points.empty())
        return;

      // 情况2：只有一个点 → 高度直接设成起点高度
      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      // 第一步：为每个点累计“从起点走到它，水平方向一共走了多远”
      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      // 逐点向右累加：本点到上一点的水平距离（只取 x、y，不管高度）
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      // 第二步：整条路径的水平总长度（最后一项就是总和）
      const double total_xy_length = accumulated_xy_length.back();
      // 第三步：按“已走的比例”给每个点重新指定高度
      for (size_t i = 0; i < points.size(); ++i)
      {
        // ratio = 这个点走完全程的百分之几（0~1）。总长为 0 时退化成“按序号平均分”
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        // 高度 = 起点高度 + 比例 × (目标高度 - 起点高度)，也就是线性插值
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      // 最后把首尾两个点的高度强制对齐，消除浮点误差
      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  // 构造函数：目前不需要做任何事
  SCANPlannerManager::SCANPlannerManager() {}

  // 析构函数：只打印一行调试信息
  SCANPlannerManager::~SCANPlannerManager() {}

  // 初始化：读参数（最大速度/加速度等）、创建栅格地图并初始化、
  // 创建 B 样条优化器并注入地图、创建 A* 并设定搜索池大小。
  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    // 记住节点指针，后面读参数、发日志都要用它
    node_ = node;
    /* read algorithm parameters */
    // 小工具 lambda：读取一个 double 参数（没声明过就先声明，再取值）
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    // 依次读取全部参数。名字对应 common.yaml 里的 manager.* 和 optimization.*
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);
    pp_.max_time_stretch_ = get_double("optimization.max_time_stretch", 2.0);
    pp_.max_head_mismatch_ = get_double("optimization.max_head_mismatch", 0.30);

    // 轨迹编号从 0 开始
    local_data_.traj_id_ = 0;
    // 创建栅格地图对象；initMap 里会读参数、订阅传感器话题、建定时器
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    // 创建 B 样条优化器，并把参数、地图、A* 全部注入给它
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    // A* 搜索池按 XY 边长 + Z 深度参数化，默认保持本分支原来的 160×160×100。
    // 池子以"当前这一段的中点"为中心，物理覆盖 = 边长 × 分辨率：
    //   160 格 × 0.05 m = 8 m（XY 每侧 ±4.0 m）；100 格 × 0.05 m = 5 m（Z 每侧 ±2.5 m）。
    // 池子偏小时 A* 会以 SEARCH_ERR 搜不到路（日志里 "A-star failed ... (ret=2)"）。
    // 内存 ≈ XY² × Z × 48 B：160×160×100 ≈ 123 MB；160×160×160 ≈ 197 MB。
    {
      if (!node->has_parameter("manager.a_star_pool_size"))
        node->declare_parameter<int>("manager.a_star_pool_size", 160);
      if (!node->has_parameter("manager.a_star_pool_z"))
        node->declare_parameter<int>("manager.a_star_pool_z", 100);
      const int pool_xy = std::max(32, static_cast<int>(node->get_parameter("manager.a_star_pool_size").as_int()));
      const int pool_z = std::max(32, static_cast<int>(node->get_parameter("manager.a_star_pool_z").as_int()));
      RCLCPP_INFO(node->get_logger(),
                  "[Astar] pool=%dx%dx%d  覆盖=±%.2f(x) ±%.2f(y) ±%.2f(z) m  约 %.0f MB",
                  pool_xy, pool_xy, pool_z,
                  pool_xy * grid_map_->getResolution() * 0.5,
                  pool_xy * grid_map_->getResolution() * 0.5,
                  pool_z * grid_map_->getResolution() * 0.5,
                  pool_xy * pool_xy * pool_z * 48.0 / 1e6);
      bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(pool_xy, pool_xy, pool_z));
    }

    // 保存可视化对象（在 RViz 里画轨迹、目标点）
    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  // ==========================================================================
  // 【全系统最核心的函数】reboundReplan()：重新规划一条局部轨迹。
  //   rebound 意为“弹回/绕开”，即让轨迹绕开障碍物。
  //
  // 输入：起点(位置/速度/加速度)、局部目标(位置/速度)、两个开关标志。
  // 返回：true = 规划成功（轨迹会被发布并执行）；false = 失败（本轮放弃）。
  //
  // 内部按 5 个 STEP 顺序执行（下面每步都有注释）。
  // 只要任何一步判断为“不安全/不可行”，就返回 false，绝不发布坏轨迹。
  // ==========================================================================
  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    // 下面几行只是调试打印：每次调用打一条分隔线并显示起终点
    last_failure_ = ReplanFailure::NONE;
    static int count = 0;
    RCLCPP_INFO(node_->get_logger(),
                "[rebo replan] #%d rid=%llu start=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) "
                "goal=(%.2f,%.2f,%.2f) target_vel=(%.2f,%.2f,%.2f)",
                count++, static_cast<unsigned long long>(diag_rid_),
                start_pt(0), start_pt(1), start_pt(2), start_vel(0), start_vel(1), start_vel(2),
                local_target_pt(0), local_target_pt(1), local_target_pt(2),
                local_target_vel(0), local_target_vel(1), local_target_vel(2));

    // 【提前退出】起点离目标不到 0.2 米 → 已经到地方了，没必要再规划
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      RCLCPP_INFO(node_->get_logger(), "[diag] rid=%llu event=close_to_goal start=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f)",
                  static_cast<unsigned long long>(diag_rid_), start_pt(0), start_pt(1), start_pt(2),
                  local_target_pt(0), local_target_pt(1), local_target_pt(2));
      last_failure_ = ReplanFailure::CLOSE_TO_GOAL;
      // 记一次“连续失败”，用于触发退避策略
      continuous_failures_count_++;
      return false;
    }

    // 开始计时（分别统计 准备/优化/细化 三个阶段）
    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    // ================= STEP 1：准备一条初始路径 =================
    /*** STEP 1: INIT ***/
    // ts = 相邻控制点的时间间隔。离目标很近(x5)时放大时间，避免速度爆表
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    // point_set：初始路径离散出的一串点；start_end_derivatives：起点终点速度/加速度
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    // flag_first_call：是否第一次规划；flag_force_polynomial：是否强制改用多项式
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    // 用 do-while 包住：万一初始路径不合理，就设 flag_regenerate 重来一次
    do
    {
      // 每轮开始先清空上一轮留下的数据
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 【情况 A】用多项式造初始路径：第一次规划 / 目标变了 / 被强制要求
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        // gl_traj 是一条多项式轨迹，作为后续优化的“初始猜测”
        PolynomialTraj gl_traj;

        // 起点到目标的直线距离
        double dist = (start_pt - local_target_pt).norm();
        // 估算走完这段需要多少秒（按最大速度、最大加速度做梯形速度估计）
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        // A1 普通情况：用一段五次多项式，直接从起点平滑连到目标
        if (!flag_randomPolyTraj)
        {
          // STEP 1：造“初始猜测”。用一个五次多项式从起点平滑连到局部目标。
          // 生成单段五次多项式：满足起点终点的位置/速度/加速度约束
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        // A2 兜底情况：连续失败多次后，插一个随机中间点，换一条不同的绕行路线
        else
        {
          // 构造一个水平方向单位向量（垂直于起点→目标方向）
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          // 再构造一个竖直方向单位向量
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          // 在中点附近随机偏一个点：失败次数越多，偏移幅度越小（收敛）
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          // 用 [起点, 随机中点, 目标] 三个点做 min-snap 多项式
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          // 两段各占一半时间
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        // 下面把多项式“离散”成一串点（点距要合适，不能太稀）
        double t;
        bool flag_too_far;
        // 先放大 1.5 倍，进循环后逐步缩小，并检查相邻点距是否合格
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          // 按 ts 步长依次采样每个时刻的位置
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            // 若相邻两点距离 > 1.5 倍控制点间距，说明采样太稀 → 缩小 ts 重来
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        // 一直重试，直到点距合格、并且点数不少于 7 个
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        // 循环退出时 t 多走了一步，减回去，得到真正的最后一个时刻
        t -= ts;
        // 记录 4 个边界量：起点速度、终点速度、起点加速度、终点加速度
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      // 【情况 B】沿用上一条轨迹：保留已走过的部分，只把尾部接到新目标上
      else // Initial path generated from previous trajectory.
      {

        double t;
        // 上一条轨迹已经执行了多少秒
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

        // 把上一条轨迹“从当前位置往后”按弧长参数化（方便按固定间距重采样）
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        // 沿着上一条轨迹往后采样
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            // 累加相邻点距离，得到弧长表
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        // 同样把多走的一步退回来
        t -= ts;

        // 如果上一条轨迹剩下太短、够不到目标，就再补一段多项式
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          // 从轨迹末端“接”一段多项式到目标，速度和加速度保持连续
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          // 把新补的这段也加进点集
          for (t = ts; t < poly_time; t += ts)
          {
            // 防御性检查：弧长表不能为空（为空说明前面的采样出了问题）
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              // 打日志并放弃本轮规划
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        // 下面把弧长表按“固定间距”重新采样，得到分布均匀的点集
        double sample_length = 0;
        // 目标点间距（先放大 1.5 倍，循环里再逐步缩小）
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        // 重复采样，直到点数不少于 7 个
        do
        {
          // 每次把间距缩小一点，点会变多
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          // 沿着弧长表，每隔 cps_dist 米取一个点
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            // 找到当前长度落在哪两个采样点之间
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              // 在这两点之间按比例线性插值，得到一个均匀分布的点
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          // 最后补上目标点本身
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        // 边界导数：起点用当前轨迹的速度/加速度，终点速度用局部目标速度，加速度为 0
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        // 若初始路径异常长（超过规划视野 3 倍），说明接头失败了
        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          // 强制改用多项式重新生成一次
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // STEP 2：把初始路径上每个点的高度 z 拉平成一条平缓参考。
    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    // ctrl_pts：B 样条的控制点。优化器后面就是调整这几个点
    Eigen::MatrixXd ctrl_pts;
    // STEP 3：把上面那串点拟合（参数化）成一条 B 样条，得到“控制点” ctrl_pts。
    // B 样条是后续优化的载体：优化器只调整这些控制点。
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    if (ctrl_pts.size() == 0)
    {
      continuous_failures_count_++;
      last_failure_ = ReplanFailure::PARAMETERIZE_FAILED;
      return false;
    }

    // a_star_paths：每个碰撞段对应的 A* 绕行路线（只用来确定“往哪边推”）
    vector<vector<Eigen::Vector3d>> a_star_paths;
    // STEP 4：检查哪些控制点落在障碍里，对每个碰撞段跑一次 A*，
    // 算出“该往哪个方向把控制点顶出去”。返回的 a_star_paths 只用于指示方向。
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    // 记录“准备初始路径”阶段的耗时
    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // 把初始路径和 A* 路线画到 RViz，方便观察
    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    // 重置计时器，开始统计优化阶段耗时
    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    // STEP 5：后端优化（L-BFGS）。同时优化平滑度、避障、速度/加速度限制。
    // flag_step_1_success = 优化是否收敛成功。
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    RCLCPP_INFO(node_->get_logger(), "[diag] rid=%llu stage=opt event=step1 ok=%d t_init=%.3f",
                static_cast<unsigned long long>(diag_rid_), static_cast<int>(flag_step_1_success), t_init);
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      last_failure_ = bspline_optimizer_rebound_->lastFailureReason();
      if (last_failure_ == ReplanFailure::NONE)
        last_failure_ = ReplanFailure::OPTIMIZE_FAILED;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // 用控制点构造一条三次 B 样条——这就是候选轨迹
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    // STEP 6：可行性检查。把物理限制告诉轨迹，然后看它有没有超速/超加速度。
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    // ratio：超限倍数（下面 checkFeasibility 会填出具体值）
    double ratio;
    bool flag_step_2_success = true;
    // checkFeasibility 返回 false 表示超限了；ratio 告诉我们超了几倍。
    if (!pos.checkFeasibility(ratio, false))
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[diag] rid=%llu stage=feas event=static_infeasible ratio=%.3f v_lim=%.2f a_lim=%.2f tol=%.2f",
                  static_cast<unsigned long long>(diag_rid_), ratio, pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

      // 【P21】时间拉长有上限。ratio 过大说明几何路径本身不可执行，
      //   无限拉长只会得到“永远追不上”的慢轨迹，直接拒绝让 FSM 重试。
      if (pp_.max_time_stretch_ > 0.0 && ratio > pp_.max_time_stretch_)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[diag] rid=%llu stage=feas event=stretch_rejected ratio=%.3f max=%.2f",
                    static_cast<unsigned long long>(diag_rid_), ratio, pp_.max_time_stretch_);
        last_failure_ = ReplanFailure::DYNAMIC_INFEASIBLE;
        continuous_failures_count_++;
        return false;
      }

      // 时间拉长后重新拟合出的控制点
      Eigen::MatrixXd optimal_control_points;
      // 超限就“重新分配时间”：把整条轨迹的时间拉长 ratio 倍（走慢一点），
      // 再做一次小优化，让它既不超限、又尽量贴近原来那条轨迹。
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      // 细化成功 → 用细化后的轨迹替换原来的
      if (flag_step_2_success)
      {
        pos = UniformBspline(optimal_control_points, 3, ts);
        // 【P21】细化后的起点必须仍然贴着规划起点，否则控制器会长期 t_proj=0
        const double head_err = (pos.evaluateDeBoorT(0.0).head<2>() - start_pt.head<2>()).norm();
        if (pp_.max_head_mismatch_ > 0.0 && head_err > pp_.max_head_mismatch_)
        {
          RCLCPP_WARN(node_->get_logger(),
                      "[diag] rid=%llu stage=feas event=head_mismatch err=%.2fm max=%.2f",
                      static_cast<unsigned long long>(diag_rid_), head_err, pp_.max_head_mismatch_);
          last_failure_ = ReplanFailure::REFINE_UNSAFE;
          continuous_failures_count_++;
          return false;
        }
      }
    }

    // 细化失败，或者轨迹仍然超速/超加速度 → 这条轨迹不安全
    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[diag] rid=%llu stage=feas event=%s ratio=%.3f v_lim=%.2f a_lim=%.2f tol=%.2f",
                  static_cast<unsigned long long>(diag_rid_),
                  flag_step_2_success ? "dynamic_infeasible" : "refine_unsafe",
                  ratio, pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
    last_failure_ = flag_step_2_success ? ReplanFailure::DYNAMIC_INFEASIBLE : ReplanFailure::REFINE_UNSAFE;
      RCLCPP_WARN(node_->get_logger(),
                  "[diag] rid=%llu stage=feas event=refine_unpublish reason=%s",
                  static_cast<unsigned long long>(diag_rid_), replanFailureToString(last_failure_));
      continuous_failures_count_++;
      return false;
    }

    // 记录“细化”阶段耗时
    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // 【P21】最终轨迹头部必须贴着规划起点。
    //   P20 拉控制点后即使优化“成功”，曲线起点也可能已经偏出 0.4 m，
    //   控制器会长期 t_proj=0，所以这里统一再检查一次。
    {
      const double head_err = (pos.evaluateDeBoorT(0.0).head<2>() - start_pt.head<2>()).norm();
      if (pp_.max_head_mismatch_ > 0.0 && head_err > pp_.max_head_mismatch_)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[diag] rid=%llu stage=feas event=final_head_mismatch err=%.2fm max=%.2f",
                    static_cast<unsigned long long>(diag_rid_), head_err, pp_.max_head_mismatch_);
        last_failure_ = ReplanFailure::REFINE_UNSAFE;
        continuous_failures_count_++;
        return false;
      }
    }

    // save planned results
    // 全部通过：把这条轨迹记录到 local_data_，本轮规划成功。
    // 全部检查通过：把最终轨迹写入 local_data_，随后会被发布执行
    updateTrajInfo(pos, node_->now());

    // 打印三个阶段的总耗时
    RCLCPP_INFO(node_->get_logger(),
                "[diag] rid=%llu stage=opt event=total t_init=%.3f t_opt=%.3f t_refine=%.3f total=%.3f",
                static_cast<unsigned long long>(diag_rid_), t_init, t_opt, t_refine, t_init + t_opt + t_refine);

    // success. YoY
    // 规划成功，把连续失败计数清零
    continuous_failures_count_ = 0;
    // 返回成功
    return true;
  }

  // 紧急停止：生成一条“所有控制点都在 stop_pos”的静止 B 样条，
  // 相当于让机器人立刻刹住不动。
  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    // 造 6 个控制点（三次 B 样条至少需要 4 个，这里给 6 个）
    Eigen::MatrixXd control_points(3, 6);
    // 把 6 个控制点全部放在停止位置 → 曲线退化成“待在原地不动”
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    // 把这条静止轨迹记录进 local_data_（这样控制器会立刻刹停）
    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  // 【P22】低速后撤：直线连接 start→target，固定时长（默认 3 s）。
  //   不经过 A*、L-BFGS、checkFeasibility，避免 0.35 m 短轨迹被 ratio≈2.9 判死。
  bool SCANPlannerManager::EscapeBackup(const Eigen::Vector3d &start,
                                        const Eigen::Vector3d &target,
                                        double duration)
  {
    constexpr int kPts = 8;  // 三次 B 样条，8 个控制点
    const double ts = std::max(0.05, duration / double(kPts - 3));
    Eigen::MatrixXd control_points(3, kPts);
    for (int i = 0; i < kPts; ++i)
    {
      const double ratio = double(i) / double(kPts - 1);
      control_points.col(i) = start + ratio * (target - start);
    }

    // 直接写入 local_data_，随后由 FSM 发布；控制器只跟踪，不再做规划检查
    updateTrajInfo(UniformBspline(control_points, 3, ts), node_->now());
    return true;
  }

  // 用一串航点生成全局参考轨迹：
  // 太长的段会先加密打点，再按“距离/最大速度”估算各段耗时，最后用 min-snap 多项式连接。
  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    // 一个航点都没有 → 无法规划，直接失败
    if (waypoints.empty())
      return false;

    // 把“起点 + 所有航点”串成一条折线
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    // 先算出整条折线的总长度
    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    // inter_points：插点之后的点列表（避免某一段太长导致多项式剧烈震荡）
    vector<Eigen::Vector3d> inter_points;
    // 插点阈值：取“总长的 1/8”和“4 米”中较大的那个
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      // 先放入本段的起点
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      // 这一段太长了 → 在中间均匀插入若干点
      if (dist > dist_thresh)
      {
        // 需要分成几段
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          // 用线性插值算出第 j 个中间点
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    // 最后补上终点
    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    // 把点集写成 3×N 矩阵（每一列是一个点的 xyz）
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    // 估算每一段需要多少时间：距离 ÷ 最大速度
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    // 首尾两段再多给一倍时间，让起步和刹车更平缓
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // 用 min-snap 多项式把这些点连成一条光滑的全局轨迹
    PolynomialTraj gl_traj;
    // 3 个点以上 → 最小化加加速度(jerk)的多段多项式
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    // 恰好 2 个点 → 退化成单段五次多项式
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    // 存入 global_data_，作为后续“取局部目标”依据的全局参考轨迹
    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  // 用起点和终点直接生成全局参考轨迹（逻辑与上面类似）。
  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    // 只有起点和终点两个点
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    // 固定阈值：任何一段超过 4 米就插点
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    // 下面与 planGlobalTrajWaypoints 完全一样：写矩阵 → 估时间 → 生成多项式
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  // “细化”步骤：先把时间拉长（reparamBspline），再以拉长后的轨迹为参考，
  // 做一次带“贴合项”的小优化，得到最终轨迹。
  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    // t_inc：因为把时间拉长而多出来的秒数（要同步给全局时间）
    double t_inc;

    // 拉长时间后重新拟合出来的控制点
    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    // 第一步：把轨迹时间拉长 ratio 倍，并重新采样/拟合，得到新控制点
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);
    RCLCPP_INFO(node_->get_logger(), "[diag] rid=%llu stage=feas event=refine ratio=%.3f ts=%.3f",
                static_cast<unsigned long long>(diag_rid_), ratio, ts);

    // 用新控制点重建轨迹对象
    traj = UniformBspline(ctrl_pts, 3, ts);

    // 第二步：把新轨迹按固定时间间隔采样成“参考点” ref_pts_
    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    // 先清空上一次的参考点
    bspline_optimizer_rebound_->ref_pts_.clear();
    // 每隔 t_step 秒取一个点，作为细化优化的“目标形状”
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    // 第三步：以这些参考点为目标，再优化一次（贴合 + 平滑 + 可行性）
    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  // 把算好的轨迹写进 local_data_：开始时刻、时长、起点、位置样条，
  // 并顺手求出速度样条和加速度样条（对位置求导），轨迹编号 +1。
  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    // 记录开始时刻——控制器和状态机用它算“这条轨迹已经走了多久”
    local_data_.start_time_ = time_now;
    // 保存位置曲线
    local_data_.position_traj_ = position_traj;
    // 速度曲线 = 位置曲线求导
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    // 加速度曲线 = 速度曲线求导
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    // 记录起点位置
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    // 记录总时长
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    // 轨迹编号 +1（每次重规划都会变，方便区分是不是新轨迹）
    local_data_.traj_id_ += 1;
  }

  // 沿轨迹采样，逐个检查速度和加速度有没有超过物理限制（留一点容差）。
  // 任一处超限就返回 false，说明这条轨迹不能给机器人执行。
  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    // 由位置曲线求导，得到速度曲线和加速度曲线
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    // 轨迹总时长
    const double duration = position_traj.getTimeSum();
    // 采样间隔：限制在 0.01~0.05 秒之间（最多采 50 个点，够用又不慢）
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    // 速度上限（物理限制 + 允许的容差）
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    // 加速度上限（物理限制 + 允许的容差）
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    // 从 0 到总时长，按固定间隔逐个时刻检查
    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      // 防止浮点误差让 t 略微超过总时长
      const double tc = std::min(t, duration);
      // 取该时刻的速度大小
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      // 超速 → 打印警告并判定这条轨迹不可行
      if (vel.norm() > vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f rid=%llu",
                    tc, vel.norm(), vel_limit, static_cast<unsigned long long>(diag_rid_));
        return false;
      }

      // 取该时刻的加速度大小
      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      // 超加速度 → 同样判定不可行
      if (acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f rid=%llu",
                    tc, acc.norm(), acc_limit, static_cast<unsigned long long>(diag_rid_));
        return false;
      }
    }

    // 全程都没有超限 → 通过检查
    return true;
  }

  // 重新分配时间：把整条 B 样条的时间轴拉长 ratio 倍，然后重新采样点、重新拟合。
  // 效果：同样的空间路径，走得更慢，速度/加速度自然就降下来了。
  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    // 原来的总时长
    double time_origin = bspline.getTimeSum();
    // 段数 = 控制点数 - 3（三次 B 样条的固有关系）
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    // 第一步：把时间轴整体拉长 ratio 倍
    bspline.lengthenTime(ratio);
    // 拉长后的新总时长
    double duration = bspline.getTimeSum();
    // 新的时间间隔 dt = 总时长 ÷ 段数
    dt = duration / double(seg_num);
    // 多出来的时间量（用于和全局时间轴对齐）
    time_inc = duration - time_origin;

    // 第二步：按新的时间间隔重新采样出一串点
    vector<Eigen::Vector3d> point_set;
    // 均匀取点
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    // 第三步：用这些点重新拟合出新的控制点
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

  // 以 goal 为圆心由近到远找一个"既站得住、又真的走得过去"的点。
  //
  // 两级判据，各解决一个不同的问题：
  //  1) 本地快速筛（standable）：自身与 8 邻居都空闲。
  //     "能站"不等于"不会被围死" —— 目标那格可能空着、但整圈被障碍包住
  //     （实测出现过 occ=0 而周边 7/8 被占）。
  //     这一步很便宜（每点约 9 次格子查询），用来把候选压到 1 个。
  //  2) 连通性验证（A*）：候选点必须能从 start 真正走过去。
  //     "站得住"也不等于"去得了" —— 目标可能在一片被封死的空腔里。
  //     这一步贵（实测单次中位 17 ms），所以**只对最终候选跑一次**，
  //     而不是对每个环上的点都跑（1681 个候选 × 17 ms 是不可接受的）。
  bool SCANPlannerManager::findNearestReachableGoal(const Eigen::Vector3d &start,
                                                    const Eigen::Vector3d &goal,
                                                    double max_radius,
                                                    Eigen::Vector3d &result)
  {
    if (!grid_map_)
      return false;

    // 【重要】目标不在当前滑动地图内时**不能**判定为"不可达"：
    //   滑动地图只有 10 m 见方且跟随机器人，刚收到远处目标时目标必然在图外
    //   （实测 occ=-1 inMap=0）。这时应当保持原目标，等机器人走近、地图覆盖到
    //   目标附近后，再由后续轮次重新判断。
    if (!grid_map_->isInMap(goal))
      return false;

    const double step = std::max(1e-3, grid_map_->getResolution());

    // 第一级：这个点站得住吗 —— 圆盘判据。
    //
    //   **不能**用 getInflateOccupancy(p, yaw)：它只沿「假定朝向」前后各探
    //   0.18 m 两个点，侧向完全不查。而降级点到站时的朝向是未知的，
    //   那个 yaw 只能拿 atan2(goal-start) 估计，与真实到站朝向无关。
    //   实测后果（goal_in_obstacle 场景）：目标在 0.4 m 半宽的柱心，
    //   挑选出的「最近可站点」落在距柱心 0.156 m 处（柱体内部），因为
    //   该点 ±0.18 m 恰好探到了柱体外。机器人最终开进柱子里。
    //   用圆盘判据作真值对比时，该判据的漏报率是 48%。
    //
    //   圆盘判据：以 p 为圆心、机体圆柱半径内所有格子都必须空闲。
    //   开销实测（2% 障碍、0.25 m 半径、首次命中即返回）：0.035 ms；
    //   最坏情况（目标被彻底围死、扫完 1681 个候选点）1.68 ms。
    //   按 1.23 s 的调用间隔算，占比 < 0.15%，可以忽略。
    // 半径必须取「双圆柱偏移」，**不是**圆柱半径：
    //   膨胀层本身就是按 double_cylinder_radius 生成的（rebuildInflationOffsets），
    //   所以单格 inflate==0 已经表示「该格 0.25m 内无原始障碍」。
    //   若这里再用圆柱半径扫一圈，等价于要求 2×半径 内无障碍 —— 双重膨胀。
    //   实测后果：go2w(半径0.277) 需要 0.954m 净空，1.0m 降级半径内找不到任何点，
    //   降级永远失败；a2 也被逼到 1.0m 边缘。
    //
    //   但也不能刚好取 offset：降级点会被 getLocalTarget() 用
    //   getInflateOccupancy(p, yaw) 复核，而那个判据探的是 p ± offset·axis
    //   两个点。若圆盘半径恰好等于 offset，探测点正好落在圆盘边界上，
    //   它所在格子中心可能在圆盘之外（最多差半个格子的对角 = step*√2/2），
    //   于是"我们没检查过的格子"被对方判定占据。
    //   实测（goal_in_obstacle / go2）：降级点本身 occAt=0，
    //   但后探测点 occR=1 → getLocalTarget 报 no_free_target 拒绝它，
    //   降级反复失败、机器人停在半路。
    //   所以圆盘半径要多留一个格子，确保 offset 处及其所在格子都被覆盖。
    const double disk_r =
        std::max(0.0, grid_map_->getDoubleCylinderOffset() + step);
    const int disk_steps = static_cast<int>(std::ceil(disk_r / step));
    const double disk_r2 = disk_r * disk_r;
    auto standable = [&](const Eigen::Vector3d &p) {
      if (!grid_map_->isInMap(p))
        return false;
      // 圆盘覆盖的所有格子都必须是确定的空闲(0)；-1(出图) 同样判不可用
      for (int i = -disk_steps; i <= disk_steps; ++i)
      {
        for (int j = -disk_steps; j <= disk_steps; ++j)
        {
          const double dx = i * step;
          const double dy = j * step;
          if (dx * dx + dy * dy > disk_r2)
            continue;
          Eigen::Vector3d q = p;
          q(0) += dx;
          q(1) += dy;
          // 必须「已观测且空闲」：未知格(从没被雷达看到)不算可站。
          //   实测教训：目标柱体只有朝向机器人的一侧被观测到，
          //   远侧仍是未知;若把未知当空闲,圆盘判据会放过柱体内部的点。
          if (!grid_map_->isKnownFree(q))
            return false;
        }
      }
      return true;
    };

    // 由近到远找出最近的"站得住"的点（先不验证连通性）。
    //   同心环的几何细节在 goal_degrade_utils.h（纯函数，有单测覆盖）。
    Eigen::Vector3d candidate;
    if (!findNearestAcceptedOnRings(goal, max_radius, step, start, standable, candidate))
      return false;

    // 第二级：从机器人当前位置能真正走到这个候选点吗。
    //   有 A* 就验证；没有（未注入）则退化为旧的纯几何行为，避免直接不可用。
    //
    //   【z 必须取同一平面】A* 的搜索面按"起点→终点连线"插值 z：
    //   若一端 z=0.50（机体中心）、另一端 z=0.00（地面目标），搜索面会斜着
    //   穿过地面膨胀层（obstacles_inflation_z_down=0.4），实测必然
    //   SEARCH_ERR(no path inside pool)。现有的 bspline_optimizer 调用者
    //   传的是轨迹控制点的 z，两端本来就同高，所以没暴露这个问题。
    //   这里统一成 goal 的 z：只关心水平连通性。
    if (bspline_optimizer_rebound_ && bspline_optimizer_rebound_->a_star_)
    {
      Eigen::Vector3d astar_start = start;
      Eigen::Vector3d astar_goal = candidate;
      astar_start(2) = goal(2);
      astar_goal(2) = goal(2);
      const ASTAR_RET ret =
          bspline_optimizer_rebound_->a_star_->AstarSearch(step, astar_start, astar_goal);
      if (ret != ASTAR_RET::SUCCESS)
      {
        // 两种可能，都退回"沿全局轨迹截断"的老路（功能降级，不崩溃）：
        //   · SEARCH_ERR：最近的可站点确实走不过去（典型：目标在一片被封死的空腔里）
        //   · INIT_ERR  ：候选点出了 A* 搜索池。池子以 (start+goal)/2 为中心、
        //     半宽 = pool*res/2 = ±4.0 m，因此 start 与 goal 相距超过约 6 m 时
        //     （4.0 - 降级半径）就会出池。滑动地图 10x10 m，目标刚进图时
        //     距离可能到 7 m，所以这个区间是真实存在的。
        RCLCPP_WARN(node_->get_logger(),
                    "[degrade] nearest standable point (%.2f, %.2f) is not connected to "
                    "robot at (%.2f, %.2f) (A-star ret=%d, dist=%.2f m); skip degradation",
                    candidate(0), candidate(1), start(0), start(1), static_cast<int>(ret),
                    (candidate - start).head<2>().norm());
        return false;
      }
    }

    result = candidate;
    return true;
  }

} // namespace scan_planner
