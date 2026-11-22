
// ============================================================================
// 文件：src/planner/plan_manage/src/scan_replan_fsm.cpp
//
// 【这个文件是干什么的】
//   状态机 SCANReplanFSM 的全部实现代码（头文件里只是声明）。
//   状态机就是程序的大脑：它每 10 毫秒醒一次，看看当前处于哪个阶段，
//   然后执行那个阶段该做的事。
//
// 【最重要的一句话】
//   想看懂整个规划程序，看懂 execFSMCallback() 和 checkCollisionCallback()
//   这两个函数就够了，其余都是它们的辅助。
//
// 【建议阅读顺序】
//   1) init()                  —— 启动时做了什么准备
//   2) execFSMCallback()       —— 主循环：六个状态各自干什么
//   3) checkCollisionCallback()—— 怎么发现快要撞上
//   4) 其余辅助函数            —— 用到时再查
// ============================================================================
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/reference_path_utils.h>
#include <plan_manage/reference_frame_utils.h>
#include <plan_manage/replan_retry.h>
#include <plan_manage/local_target_utils.h>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <tf2/time.hpp>
#include <tf2/exceptions.hpp>

namespace
{
  // 小工具函数：读取一个 ROS 参数。
  // 如果参数还没声明过就先声明（用给定的默认值），然后返回它的值。
  // 这样可以避免“参数不存在”导致的报错。
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  // ==========================================================================
  // init()：程序启动时执行一次的全部准备工作。
  //   1. 把内部的标志位/计数器清零
  //   2. 从 YAML 参数文件读取所有配置（navi_mode、阈值、膨胀尺寸……）
  //   3. 创建可视化对象和规划总管，并让规划总管去建地图、建优化器
  //   4. 创建两个定时器（10ms 主循环 / 50ms 安全检查）
  //   5. 订阅话题：里程计、底盘冻结；再按 navi_mode 订阅目标点或外部路径
  //   6. 创建发布器：规划出的轨迹、调试数据、机器人外形
  // ==========================================================================
  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    // ---------- 第1步：把所有内部状态重置为初始值 ----------
    node_ = node;
    // TF 缓冲与监听器：navi_mode=3 需要把参考坐标系下的路径换算到 world 系
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // 航点下标从 0 开始；状态机初始状态是 INIT
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    // 这几个布尔量表示“收到过/发生过没有”：触发信号、目标、里程计、新目标
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    // 是否已从首帧里程计记录下目标高度
    rviz_height_ready_ = false;
    // 底盘是否正在原地转向（会暂停轨迹时钟）
    go2_execution_frozen_ = false;
    // 自动恢复开关 / 是否需要悬停停下 / 是否正在从急停恢复
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    // 连续重规划失败计数器清零
    replan_fail_count_ = 0;
    // 脱困计数清零
    escape_total_ = 0;
    escape_active_ = false;
    escape_strategy_ = 0;
    // 运行状态复位：未收到目标前不参与到达判定
    status_have_goal_ = false;
    status_target_valid_ = false;
    status_is_stop_ = false;
    status_goal_valid_ = false;
    status_error_latched_ = false;
    status_last_state_.clear();
    // 记录当前时间，供“冻结”逻辑计算时间差
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    // ---------- 第2步：读取全部参数（名字对应 common.yaml） ----------
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    // 起点与当前位置相差超过这个距离 → 触发重规划
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    // 离终点小于这个距离 → 不必再重规划
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    // 规划视野：每次只规划前方这么远（米）
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    // 预计在这个时间(秒)内会撞上障碍 → 判为紧急
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    // 是否允许自动从急停恢复
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    // 连续规划失败超过这个次数 → 强制急停
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 20);
    // 脱困后撤的全局次数上限：防止「重试→后撤→重试」无限循环
    max_escape_total_ = load_parameter<int>(node_, "fsm.max_escape_total", 6);
    if (max_escape_total_ < 0)
      max_escape_total_ = 0;
    // 目标不可达时的降级搜索半径：0=关闭（退回"沿轨迹截断"的旧行为）
    goal_degrade_radius_ = load_parameter<double>(node_, "fsm.goal_degrade_radius", 1.0);
    degrade_verify_after_fails_ =
        load_parameter<int>(node_, "fsm.degrade_verify_after_fails", 5);
    if (degrade_verify_after_fails_ < 0)
      degrade_verify_after_fails_ = 0;
    if (goal_degrade_radius_ < 0.0)
      goal_degrade_radius_ = 0.0;
    // 连续规划失败上限：0=不限制（关掉这个兜底）
    max_consecutive_plan_fail_ =
        load_parameter<int>(node_, "fsm.max_consecutive_plan_fail", 30);
    if (max_consecutive_plan_fail_ < 0)
      max_consecutive_plan_fail_ = 0;
    // 是否把「必须人工干预」类失败锁存为 error（默认关闭，见头文件说明）
    status_latch_hard_error_ =
        load_parameter<bool>(node_, "fsm.status_latch_hard_error", false);
    // 下面几个是机器人外形尺寸（用于画自身膨胀 + 碰撞检测）
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    // 外部使能（/motion_enabled）的初始值。真机必须等遥控器 START，默认 false；
    // 仿真没有这个遥控器，launch 会把它置 true，否则规划器和控制器一直不动。
    motion_enabled_default_ = load_parameter<bool>(node_, "fsm.motion_enabled_default", false);
    motion_enabled_ = motion_enabled_default_;
    if (!motion_enabled_)
      RCLCPP_INFO(node_->get_logger(),
                  "[fsm] event=gated reason=motion_disabled hint=\"waiting for /motion_enabled=true (remote START/B)\"");
    reference_tf_max_age_ = load_parameter<double>(node_, "fsm.reference_tf_max_age", 0.3);
    reference_tf_grace_ = load_parameter<double>(node_, "fsm.reference_tf_grace", 0.5);
    reference_tf_jump_translation_ = load_parameter<double>(node_, "fsm.reference_tf_jump_translation", 0.2);
    reference_tf_jump_yaw_ = load_parameter<double>(node_, "fsm.reference_tf_jump_yaw_deg", 5.0) * M_PI / 180.0;
    if (reference_tf_max_age_ <= 0.0 || reference_tf_grace_ <= 0.0 ||
        reference_tf_jump_translation_ <= 0.0 || reference_tf_jump_yaw_ <= 0.0)
      throw std::runtime_error("reference TF limits must be positive");

    // ---------- 运行状态(running/goal_reached)判定参数 ----------
    status_goal_tol_ = load_parameter<double>(node_, "fsm.status_goal_tol", 0.20);
    status_goal_tol_exit_ = load_parameter<double>(node_, "fsm.status_goal_tol_exit", 0.30);
    status_speed_tol_ = load_parameter<double>(node_, "fsm.status_speed_tol", 0.05);
    status_speed_tol_exit_ = load_parameter<double>(node_, "fsm.status_speed_tol_exit", 0.10);
    status_stop_debounce_ = load_parameter<double>(node_, "fsm.status_stop_debounce", 1.0);
    status_heartbeat_hz_ = load_parameter<double>(node_, "fsm.status_heartbeat_hz", 1.0);
    status_goal_change_eps_ = load_parameter<double>(node_, "fsm.status_goal_change_eps", 0.05);
    status_error_time_ = load_parameter<double>(node_, "fsm.status_error_time", 3.0);
    status_name_disabled_ = load_parameter<std::string>(node_, "fsm.status_name_disabled", "disabled");
    status_name_paused_ = load_parameter<std::string>(node_, "fsm.status_name_paused", "stopped");
    stuck_window_ = load_parameter<double>(node_, "fsm.finish_stuck_window", 3.0);
    stuck_progress_eps_ = load_parameter<double>(node_, "fsm.finish_progress_eps", 0.10);
    finish_timeout_extra_ = load_parameter<double>(node_, "fsm.finish_timeout_extra", 30.0);
    // 卡住后有限重试，重试期内不清目标
    max_stuck_retry_ = load_parameter<int>(node_, "fsm.max_stuck_retry", 3);
    // 重试用尽后的脱困后撤距离
    escape_distance_ = load_parameter<double>(node_, "fsm.escape_distance", 0.35);
    // 后撤固定时长（低速直连轨迹，不走普通优化）
    escape_duration_ = load_parameter<double>(node_, "fsm.escape_duration", 3.0);
    // 收尾时是否发一条“停在原地”轨迹，让控制器立即刹停（P1）
    stop_traj_on_finish_ = load_parameter<bool>(node_, "fsm.stop_traj_on_finish", true);
    // 轨迹时钟最多超前实测投影进度多少秒（<=0 关闭，P4）
    traj_clock_max_lead_ = load_parameter<double>(node_, "fsm.traj_clock_max_lead", 0.3);
    // 重规划重试节流：连续失败后按 min×2^n 退避，封顶 max（enable=false 关闭）
    replan_retry_enable_ = load_parameter<bool>(node_, "fsm.replan_retry_enable", true);
    replan_retry_min_ = load_parameter<double>(node_, "fsm.replan_retry_min", 0.2);
    replan_retry_max_ = load_parameter<double>(node_, "fsm.replan_retry_max", 1.0);
    if (replan_retry_min_ < 0.0 || replan_retry_max_ < replan_retry_min_)
      throw std::runtime_error("fsm.replan_retry_min/max must be non-negative and min <= max");
    // 起点脱困：起点落在膨胀层里时，沿"起点→局部目标"方向最多前推的距离 [m]
    start_escape_max_offset_ = load_parameter<double>(node_, "fsm.start_escape_max_offset", 1.0);
    // [replan-geom] 节流(ms)；0=不节流
    diag_throttle_ms_ = load_parameter<int>(node_, "fsm.diag_throttle_ms", 200);
    diag_verbose_ = load_parameter<bool>(node_, "fsm.diag_verbose", false);
    collision_skip_from_start_ = load_parameter<double>(node_, "fsm.collision_skip_from_start", 0.25);
    diag_param_cb_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          for (const auto &p : params)
          {
            if (p.get_name() == "fsm.diag_verbose") diag_verbose_ = p.as_bool();
            else if (p.get_name() == "fsm.diag_throttle_ms") diag_throttle_ms_ = static_cast<int>(p.as_int());
          }
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          return result;
        });

    // ---------- 第3步：预设航点模式下，解析航点数组 ----------
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      // fsm.waypoints 是扁平数组，每 3 个数字组成一个航点 (x,y,z)
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      // 校验：不能为空，且个数必须是 3 的倍数，否则启动直接报错
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      // 航点个数 = 数字总数 / 3
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      // 每 3 个数字拼成一个三维航点
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    // ---------- 第4步：创建可视化对象与规划总管 ----------
    /* initialize main modules */
    // 可视化对象：在 RViz 里画目标点/轨迹/A* 路径
    visualization_.reset(new PlanningVisualization(node_));
    // 规划总管：真正算轨迹的对象（内部会建地图、建优化器）
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    // ---------- 第5步：创建两个定时器 ----------
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           // 10ms 定时器 → 状态机主循环 execFSMCallback
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             // 50ms 定时器 → 碰撞安全检查 checkCollisionCallback
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    // ---------- 第6步：订阅话题 ----------
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        // 订阅里程计：机器人真实位置/速度/姿态
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    // 订阅“底盘是否冻结”：原地转向时让规划器暂停轨迹时钟
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    // 任务控制：/mission/control（"pause" 清轨迹停住 / "resume" 重建并继续）；命令类话题，不锁存
    // 外部使能：/motion_enabled（遥控器 START/B）；使用默认 QoS（KEEP_LAST(10)+RELIABLE+VOLATILE，不锁存）
    //   注意：VOLATILE ⇒ 节点重启后拿不到“上一次的锁存值”，必须等发布端再次发 true。
    motion_enabled_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "motion_enabled", rclcpp::QoS(10),
        std::bind(&SCANReplanFSM::motionEnabledCallback, this, std::placeholders::_1));
    mission_control_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/mission/control", rclcpp::QoS(10).reliable(),
        std::bind(&SCANReplanFSM::missionControlCallback, this, std::placeholders::_1));

    // ---------- 第7步：创建发布器 ----------
    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    // 把当前局部轨迹也以 Path 形式发出来（navi_mode=3 用来观察参考路径跟随效果）
    local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_path", 10);
    // 发布规划出的轨迹（控制器订阅它来跟踪）
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    // 发布调试数据（给 RViz 显示）
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        // 发布机器人外形（前后两个圆柱的 Marker）
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    // 倾斜双圆柱 / 姿态轴线 / 角度文字（独立话题与命名空间，避免与检测窗口混淆）
    body_pose_visual_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "body_pose_visual", rclcpp::QoS(10).reliable().transient_local());
    // 双圆柱 marker 改由 20 Hz 定时器发布（原来挂在里程计回调里，约 100 Hz × 2 条 = 200 条/秒）。
    // RViz 的 Marker 订阅 Depth 只有 5，跟不上；队列按“保留最新”丢样本，每对里先发的
    // 前圆柱（id=0）几乎总被丢掉 —— 这就是“第一个圆柱隔很久才出现”的原因。
    self_inflation_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (have_odom_)
      {
        publishSelfInflationMarker();
        publishBodyPoseVisualization();
      }
    });
    // 运行状态话题：只发 running / goal_reached（idle 不发），用 transient_local 让后订阅者也能拿到当前状态
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/local_planner/status", rclcpp::QoS(1).reliable().transient_local());

    // ---------- 第8步：按导航模式订阅对应的“目标来源” ----------
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      // 模式 1：订阅 RViz 的 2D 目标点
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    // 模式 3：订阅外部发布的全局路径
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", rclcpp::QoS(1).reliable().transient_local(), std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    // 模式 2：预设航点，不需要订阅；收到第一帧里程计后自动开始
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    // 其它值 → 参数非法，直接抛异常退出
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
    // 启动摘要：把现场最常调的生效参数集中打出来，避免排查时反复翻 YAML
    RCLCPP_INFO(node_->get_logger(),
                "[sys] event=startup navi_mode=%d horizon=%.2fm replan=%.2fm no_replan=%.2fm "
                "emergency=%.2fs max_vel=%.2fm/s max_acc=%.2fm/s2 map_res=%.2fm",
                navi_mode_, planning_horizon_, replan_thresh_, no_replan_thresh_,
                emergency_time_, planner_manager_->pp_.max_vel_, planner_manager_->pp_.max_acc_,
                planner_manager_->grid_map_->getResolution());
    RCLCPP_INFO(node_->get_logger(),
                "[sys] event=startup_recovery stuck=%.1fs retry_max=%d max_escape=%d "
                "escape=%.2fm/%.1fs degrade=%.2fm consecutive_fail=%d "
                "retry_backoff=%.2f-%.2fs traj_lead=%.2fs start_escape=%.2fm "
                "latch_hard_error=%d diag_verbose=%d motion_enabled=%d",
                stuck_window_, max_stuck_retry_, max_escape_total_,
                escape_distance_, escape_duration_, goal_degrade_radius_,
                max_consecutive_plan_fail_, replan_retry_min_, replan_retry_max_,
                traj_clock_max_lead_, start_escape_max_offset_,
                static_cast<int>(status_latch_hard_error_), static_cast<int>(diag_verbose_),
                static_cast<int>(motion_enabled_));
  }

  // 预设航点模式的入口：收到第一帧里程计后，从第一个预设航点开始规划。
  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    // 复制一份预设航点，作为“本轮要依次前往的航点列表”
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    // 先把每个航点画到 RViz（显示成目标球）
    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    // 激活航点列表，从第 0 个航点开始
    active_waypoints_ = wps;
    current_wp_ = 0;
    // 打开触发开关；把起点记为当前位置
    trigger_ = true;
    init_pt_ = odom_pos_;

    // 规划到第一个航点
    if (planNextWaypoint())
    {
      // 成功 → 切换到“生成新轨迹”状态
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      // 失败 → 打错误日志（状态保持不变）
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  // 回调：在 RViz 里点了 2D 目标点后，这里被触发（navi_mode = 1）。
  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    // 空消息直接忽略
    if (!msg)
      return;

    // 还没收到过里程计 → 不知道目标该放在什么高度，先忽略
    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    // 把单个目标点包装成“只有 1 个点的 Path”，复用统一处理函数
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    // 交给统一入口 waypointCallback 处理
    waypointCallback(path);
  }

  // 目标点处理的“主入口”：把目标点转成全局轨迹，并决定进入哪个状态。
  // （RViz 点选的目标会先被包装成只有 1 个点的 Path，再交给这里）
  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    // 空消息或没有点 → 忽略
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    // 高度小于 -0.1 视为无效目标（RViz 点选时可能点到地面以下）
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    // 正式触发：记录触发标志和起点
    RCLCPP_INFO(node_->get_logger(), "[FSM] Triggered goal at (%.2f,%.2f,%.2f)",
                odom_pos_(0), odom_pos_(1), odom_pos_(2));
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    // 目标点：x、y 用点选的坐标，高度用之前记录的目标高度
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    // 【降级幂等】只有"真正的新目标"才重置任务级目标状态，且必须在
    //   adjustGlobalTargetIfOccupied（会写降级/截断标记）之前做。
    //   原来这三个标记在 adjust 之后被无条件清零，等于把刚算出的降级/截断
    //   结果全部抹掉：mode1 下目标被占时截断标记丢失 → 到达判定仍指向
    //   障碍点 → 永远到不了，最终只能按连续失败放弃。
    const bool is_new_goal = status_is_stop_ || !status_goal_valid_ ||
                             (end_pt_ - status_goal_ref_).norm() > status_goal_change_eps_;
    if (is_new_goal)
    {
      resetMissionGoalState();
      status_goal_ref_ = end_pt_;
      status_goal_valid_ = true;
    }
    // 让规划总管生成一条“从当前位置到目标”的全局轨迹
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 全局轨迹成功后再检查：目标点是否落在障碍里
    if (success)
      success = adjustGlobalTargetIfOccupied();

    // 把目标点画到 RViz
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    // ---- 成功分支：更新标志，并在 RViz 显示全局路径 ----
    if (success)
    {

      /*** display ***/
      // 每 0.1 秒采样一个点，用来把全局路径画成一条线
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      // 逐点采样
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      // 终点速度设为 0（到终点要停住）
      end_vel_.setZero();
      // 标记“已有目标”“这是一个新目标”
      have_target_ = true;
      have_new_target_ = true;
      // 【V9】新目标必须清掉上一轮的失败计数，否则 finishProcess 会立刻把
      //   新任务打成后撤/急停（pathCallback 的 P23 已这么做，这里要对齐）。
      replan_fail_count_ = 0;
      stuck_retry_count_ = 0;
      escape_active_ = false;
      escape_strategy_ = 0;
      escape_total_ = 0;
      exec_no_progress_ = false;
      // mode1/mode2 的目标点直接就是 world 系坐标，无需等参考 TF
      status_target_valid_ = true;
      // 到达判定基准 = 本次任务的真实最终目标。
      //   降级成功时 end_pt_ 已是降级点、mission_goal_pt_ 也已由 tryDegradeTarget
      //   写成同一个点，这里不能覆盖；截断/降级的标记同样不能在这里清零
      //   （重置已在 adjustGlobalTargetIfOccupied 之前按"是否新目标"做过）。
      if (!mission_goal_degraded_)
        mission_goal_pt_ = end_pt_;
      // 新目标 → 运行状态置为 running
      armPlannerStatus();

      /*** FSM ***/
      // 按当前状态切换：等待中/急停中 → 生成新轨迹；正在执行 → 重新规划
      // 【V5】必须包含 EMERGENCY_STOP：否则新目标无法把急停状态唤醒，
      //   （mode3 下外部只发一次路径时）任务会永久停在急停。
      // 但 TF 停机（reference_tf_stop_）必须排除：那时新目标同样无法换算，
      // 唤醒只会造成震荡，恢复交给 EMERGENCY_STOP 的 TF_RECOVERED 分支。
      if (exec_state_ == WAIT_TARGET ||
          (exec_state_ == EMERGENCY_STOP && !reference_tf_stop_))
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      // 显示全局路径
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      // ---- 失败分支：打错误日志 ----
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  // 根据一串航点，让规划总管生成一条全局参考轨迹。
  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    // 参考路径至少要有两个点（第一个点当起点，最后一个点当终点）
    if (waypoints.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(), "Reference path requires at least two points");
      return false;
    }

    // 终点先记在局部变量里。只有在全局轨迹真正生成成功之后才写进 reference_end_pt_，
    // 否则规划失败时会出现"局部目标沿旧轨迹取、到达判定按新终点判"的不同源问题。
    const Eigen::Vector3d new_reference_end = waypoints.back();
    // 第一个点作为起点，其余点作为中间航点
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    // 把每个航点都画出来
    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i,
                                       reference_frame_id_);
    }

    // 让规划总管用这些航点生成全局参考轨迹
    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        reference_waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    // 失败 → 打日志并返回
    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    // 采样全局轨迹（每 0.1 秒一个点）用于显示
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    // 成功 → 此时才把终点写进 reference_end_pt_，保证它与 global_data_ 同源
    reference_end_pt_ = new_reference_end;

    // 终点速度设为 0；标记有目标、有新目标
    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    // 显示全局路径与终点（都要指定参考坐标系，否则 RViz 里会画错位置）
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0, reference_frame_id_);
    visualization_->displayGoalPoint(reference_end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3,
                                     static_cast<int>(waypoints.size()) - 1, reference_frame_id_);

    // 成功
    return true;
  }

  // 前往“下一个”航点：把起点设成当前状态，终点设成该航点，然后规划全局轨迹。
  bool SCANReplanFSM::planNextWaypoint()
  {
    // 航点下标越界 → 报错并返回失败
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    // 终点 = 当前要前往的航点
    end_pt_ = active_waypoints_[current_wp_];
    // 每个航点都是新的任务目标：任务级目标状态必须在这里重置，
    //   而且要在 adjustGlobalTargetIfOccupied（会写降级/截断标记）之前。
    if (status_is_stop_ || !status_goal_valid_ ||
        (end_pt_ - status_goal_ref_).norm() > status_goal_change_eps_)
    {
      resetMissionGoalState();
      status_goal_ref_ = end_pt_;
      status_goal_valid_ = true;
    }
    // 起点：默认用里程计；已有轨迹时用轨迹上的速度/加速度（衔接更平滑）
    setStartStateFromOdomOrCurrentTraj();

    // 规划“从起点到该航点”的全局轨迹
    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    // 失败 → 报错返回
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      // 必须写入本次失败原因，否则上层守卫会读到上一次遗留的原因（可能被判成“非错误”而漏计）
      setStatusReason(ReplanFailure::GLOBAL_TRAJ_FAILED);
      return false;
    }

    // 检查终点是否被障碍占据
    if (!adjustGlobalTargetIfOccupied())
      return false;

    // 采样轨迹用于显示
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    // 终点速度 0；标记有目标、有新目标
    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    // 航点坐标本身就在 world 系，目标立即可用于到达判定
    status_target_valid_ = true;
    // 航点模式下每个航点本身就是"本段真实目标"。
    //   降级时 end_pt_/mission_goal_pt_ 已指向降级点，不要覆盖。
    if (!mission_goal_degraded_)
      mission_goal_pt_ = end_pt_;
    // 显示全局路径与当前航点
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    // 打日志：正在前往第几个航点
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    // 新航点：清除重试退避，第一次尝试立即进行
    resetPlanRetry();
    return true;
  }

  // 判断当前是不是“按预设航点依次走”的模式（只有 navi_mode = 2 才是）。
  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    // 只有 navi_mode == 2 才是“依次走航点”模式
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  // 目标不可达时，在 goal_degrade_radius_ 内找最近的可行点替代它。
  //   目标还没进滑动地图（occ=-1）→ 什么都不做，等走近了再判。
  //   目标在膨胀层里（occ>0）  → 四周由近到远搜一个能站的点。
  // 返回 true = 无需处理 / 已处理；false = 搜了但没找到（调用方兜底）。
  // 重置"任务级目标状态"：目标是否被截断/降级、以及实际可达点缓存。
  //   三个入口（waypointCallback / pathCallback / planNextWaypoint）在确认
  //   "这是新目标"后调用。**必须在 adjustGlobalTargetIfOccupied 之前调用**，
  //   否则会把那一轮刚算出的降级/截断结果清掉。
  void SCANReplanFSM::resetMissionGoalState()
  {
    mission_goal_truncated_ = false;
    have_truncated_reachable_pt_ = false;
    mission_goal_degraded_ = false;
    truncated_reachable_pt_ = Eigen::Vector3d::Zero();
  }

  bool SCANReplanFSM::tryDegradeTarget(const Eigen::Vector3d &goal_pt)
  {
    auto map = planner_manager_ ? planner_manager_->grid_map_ : nullptr;
    if (!map || goal_degrade_radius_ <= 0.0)
      return false;

    // 已降级过就直接复用，否则每轮都会把目标再往外挪一点
    if (mission_goal_degraded_ && have_truncated_reachable_pt_)
    {
      end_pt_ = truncated_reachable_pt_;
      return true;
    }

    const double yaw = estimateYawFromSegment(odom_pos_, goal_pt);
    const int occ = map->getInflateOccupancy(goal_pt, yaw);

    // -1 = 目标不在滑动地图内，属于「还不知道」，不是「不可达」：
    //   滑动地图会随机器人移动而覆盖到目标，等到那时再判断。
    if (occ < 0)
      return true;

    // occ == 0（目标格本身空闲）**不能**直接放行。
    //   "格子空闲"只说明这里没有障碍，既不保证站得住（该判据只看朝向
    //   前后两点，侧向不查），也不保证走得到（目标可能在一片被封死的
    //   空腔里：格子是空的、8 邻居也空，但与机器人不连通）。
    //   实测（enclosed_goal 场景）：目标在四面墙内的空腔正中，occ=0，
    //   原实现直接 return true → 降级从不触发 → 机器人反复重试直到
    //   "连续失败上限"把任务判成 failed。
    //
    // 但也不能无条件验证：occ==0 在绝大多数情况下就是正常可达的目标，
    //   每次都跑圆盘+A* 是白花开销，还可能因 A* 的瞬时失败把好目标误降级。
    //   所以只在「已经为这个目标连续失败过若干次」时才升级为完整验证——
    //   这时才有证据怀疑是"站得住但去不了"。
    if (occ == 0 && replan_fail_count_ < degrade_verify_after_fails_)
      return true;

    // 这里**故意不做失败退避**。
    //   调用频率本来就受 REPLAN_TRAJ 的 planAttemptAllowed() 控制（实测间隔
    //   1.22~1.23 s），单次 A* 约 17 ms 只占 1.4%，不构成性能问题。
    //   更重要的是：目标可达性会随滑动地图更新而改变（障碍被清除、或被新建），
    //   "上一次失败"不代表"这一次也失败"。加静默期会让降级在可达性恢复后
    //   仍然被锁住，破坏"边走边判"这个设计前提。
    Eigen::Vector3d degraded;
    // 让降级验证里的 A* 日志携带本次规划 rid（否则会沿用旧值）
    planner_manager_->setDiagContext(replan_rid_, diag_verbose_);
    if (!planner_manager_->findNearestReachableGoal(odom_pos_, goal_pt,
                                                    goal_degrade_radius_, degraded))
      return false;

    const double shift = (degraded - goal_pt).head<2>().norm();
    if (shift <= 1e-3)
    {
      mission_goal_truncated_ = false;
      return true;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[fsm] event=goal_degraded rid=%llu from=(%.2f,%.2f,%.2f) to=(%.2f,%.2f,%.2f) shift=%.2fm",
                static_cast<unsigned long long>(replan_rid_),
                goal_pt(0), goal_pt(1), goal_pt(2),
                degraded(0), degraded(1), degraded(2), shift);

    // 降级点成为新目标：规划、到达判定都以它为准
    end_pt_ = degraded;
    mission_goal_pt_ = degraded;
    mission_goal_degraded_ = true;
    mission_goal_truncated_ = false;
    truncated_reachable_pt_ = degraded;
    have_truncated_reachable_pt_ = true;

    // 【必须在降级点确定后立刻重建全局轨迹】
    //   否则 getLocalTarget 仍从旧轨迹取样，前视点还指着原障碍终点，
    //   于是降级成功后紧接着就报 no_free_target（实测 go2 在同一毫秒内
    //   连续失败、直到任务被放弃）。
    //   放在这里而不是调用方，是因为这条路径有两个入口：
    //     · callReboundReplan 入口（会自己重建，但只在首次降级时走到）
    //     · adjustGlobalTargetIfOccupied（原本完全不重建）
    //   统一到这里，两处都正确。
    // mode3 的终点每 tick 由 getLocalTarget / monitorReferenceTransform 从
    //   reference_end_pt_ 重算。必须同步写回，否则降级点立刻被覆盖回原目标
    //   （实测 go2：降级成功后同一毫秒内 getLocalTarget 报 no_free_target）。
    //
    //   写回失败时**必须整体回滚**，不能只跳过写回：
    //     end_pt_ 已是降级点、mission_goal_degraded_ 也已置 true，
    //     但到下个 tick monitorReferenceTransform() 会做
    //       end_pt_ = current * reference_end_pt_   （reference_end_pt_ 仍是原目标）
    //       mission_goal_pt_ = end_pt_              （连任务目标也一起打回）
    //     于是到达判定指向原目标、而 mission_goal_degraded_ 让
    //     goal_is_final 成立 → 既到不了、也不会重新降级，任务卡死。
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      Eigen::Isometry3d ref_to_world;
      if (!lookupReferenceTransform(ref_to_world))
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "[fsm] event=goal_degrade_rollback rid=%llu reason=tf_unavailable to=(%.2f,%.2f)",
                             static_cast<unsigned long long>(replan_rid_), degraded(0), degraded(1));
        end_pt_ = goal_pt;
        mission_goal_degraded_ = false;
        have_truncated_reachable_pt_ = false;
        truncated_reachable_pt_ = Eigen::Vector3d::Zero();
        return false;
      }
      reference_end_pt_ = ref_to_world.inverse() * end_pt_;
    }

    const bool rebuilt = planner_manager_->planGlobalTraj(
        start_pt_, start_vel_, start_acc_, end_pt_,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    if (!rebuilt)
    {
      // 重建失败（典型：起点还在膨胀层里）。此时不能宣称降级成功，
      //   否则 getLocalTarget 会用旧轨迹取样并报 no_free_target。
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[fsm] event=goal_degrade_rollback rid=%llu reason=global_traj_failed to=(%.2f,%.2f)",
                           static_cast<unsigned long long>(replan_rid_), degraded(0), degraded(1));
      end_pt_ = goal_pt;
      mission_goal_degraded_ = false;
      have_truncated_reachable_pt_ = false;
      return false;
    }
    // 判定基准变了，已进入的"到达候选"要重新计
    status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    status_goal_snapshot_ = Eigen::Vector3d::Zero();
    // 降级成功是"已修复"的事件，不是待处理故障：
    //   status_reasons_ 的语义是"当前还有哪些未解决的问题"，供上层诊断。
    //   这里若记 TARGET_OCCUPIED，会在"同轮规划仍失败"时被累积，
    //   超时后上报成 failed；更糟的是它属硬错误，一旦打开
    //   fsm.status_latch_hard_error 就会把一次成功的降级锁存成 error，直到重启节点。
    return true;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    // 取出地图和全局轨迹数据
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    // 没有地图、或轨迹太短 → 无需调整，直接当成功
    if (!map || duration < 1e-3)
      return true;

    // 从轨迹末端开始，按 0.05 秒间隔采样
    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    // 轨迹最后一个点（原始终点）
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    // 终点的前一个采样点（用来估计朝向）
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);

    // 先在目标四周搜（tryDegradeTarget 内部判定是否需要）
    if (tryDegradeTarget(final_pt))
      return true;

    // 四周搜不到 → 沿轨迹往回退，找最后一个能站的点
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "[fsm] event=goal_degrade_fallback rid=%llu reason=no_reachable_point radius=%.1fm",
                         static_cast<unsigned long long>(replan_rid_), goal_degrade_radius_);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        // 只截断轨迹时长，不改 end_pt_：
        //   end_pt_ 被就地改成半路点后再没人还原，getLocalTarget 会一直返回它，
        //   靠近 0.2 m 就必然报 CLOSE_TO_GOAL（确定性失败）。
        mission_goal_truncated_ = true;
        setStatusReason(ReplanFailure::TARGET_OCCUPIED);
        // 记住这个可达点：重建全局轨迹时用它当终点，否则又会回到障碍处
        truncated_reachable_pt_ = pt;
        have_truncated_reachable_pt_ = true;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_INFO(node_->get_logger(),
                    "[fsm] event=goal_truncated rid=%llu from=(%.2f,%.2f,%.2f) to=(%.2f,%.2f,%.2f)",
                    static_cast<unsigned long long>(replan_rid_),
                    end_pt_(0), end_pt_(1), end_pt_(2), pt(0), pt(1), pt(2));
        return true;
      }
    }

    // 整条轨迹都找不到可用点 → 报错返回失败
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "[fsm] event=goal_unreachable rid=%llu reason=no_free_point",
                          static_cast<unsigned long long>(replan_rid_));
    return false;
  }

  // 回调：收到外部给的一条全局路径时触发（navi_mode = 3）。
  // 会把路径上每个点抬高 body_height_，再转成全局轨迹。
  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    // 空路径 → 忽略
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    // 收到参考路径先做三项校验：必须有里程计、必须带坐标系、各点坐标系要一致
    if (!have_odom_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No odometry yet; ignoring initial_path");
      return;
    }

    if (msg->header.frame_id.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path with an empty frame_id");
      return;
    }
    for (const auto &pose : msg->poses)
    {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != msg->header.frame_id)
      {
        RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path with mixed pose frames");
        return;
      }
    }

    // 把路径上每个点转成三维航点
    std::vector<Eigen::Vector3d> waypoints;
    // 把路径上的点统一换算成 world 系下的三维航点（内部会补上 body_height_，因为外部给的是地面高度）
    std::string path_error;
    if (!prepareReferenceWaypoints(*msg, body_height_, 0.5, waypoints, &path_error))
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path: %s", path_error.c_str());
      return;
    }

    // 缓存最近一条可用的全局路径：外部 resume 时用它重建
    last_global_path_ = msg;
    // 外部停止期间只缓存、不规划（恢复时再重建）
    if (externally_stopped_)
    {
      RCLCPP_INFO(node_->get_logger(), "External STOP active: /global_path cached, not planning");
      return;
    }
    // 外部未使能期间只缓存、不规划（解冻时由 motionEnabledCallback 重建）
    if (!motion_enabled_)
    {
      RCLCPP_INFO(node_->get_logger(), "motion disabled: /global_path cached, not planning");
      return;
    }

    // 运行状态：目标变了、重复路径、或"已到达(idle)"后又来路径 → 都算新目标
    const bool status_new_goal =
        status_is_stop_ || !status_goal_valid_ ||
        (waypoints.back() - status_goal_ref_).norm() > status_goal_change_eps_;
    if (status_new_goal)
    {
      status_goal_ref_ = waypoints.back();
      status_goal_valid_ = true;
    }

    // 记住这条参考路径的坐标系，并清掉上一轮的 TF 状态
    reference_frame_id_ = msg->header.frame_id;
    have_last_reference_transform_ = false;
    reference_jump_replan_ = false;
    // 收到参考路径即视为触发
    trigger_ = true;
    // 用这些航点生成全局轨迹
    bool success = planGlobalTrajByWaypoints(waypoints);

    // 成功 → 按当前状态切到“生成新轨迹”或“重新规划”
    if (success)
    {
      // 【P23】新全局路径必须强制替换旧目标/旧失败状态，
      //   否则 REPLAN_TRAJ 里的 close_to_goal 循环会继续沿用旧 end_pt_。
      have_target_ = true;
      have_new_target_ = true;
      replan_fail_count_ = 0;
      stuck_retry_count_ = 0;
      escape_active_ = false;
      escape_strategy_ = 0;
      // 【V9 补充】escape_total_ 也必须清：它是"全局后撤次数上限"（max_escape_total）。
      //   若不清，上一轮任务用满 6 次后撤后，新目标来了连一次后撤都不允许，
      //   startBackupEscape 直接返回 false → finishProcess 立刻放弃任务。
      //   waypointCallback（mode1/2）一直是清的，这里原本漏了。
      escape_total_ = 0;
      exec_no_progress_ = false;
      exec_progress_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      status_is_stop_ = false;
      // 【V9】与 waypointCallback 对齐：新路径要清掉上一轮的失败/脱困计数
      plan_retry_fails_ = 0;
      last_plan_attempt_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      // 只有“真正的新目标”才清掉到达判定缓存；重复收到同一条路径
      // （外部全局规划器可能周期性重发）不应该反复重置去抖计时。
      if (status_new_goal)
      {
        status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        status_goal_snapshot_ = Eigen::Vector3d::Zero();
      }

      // 【V3】到达判定基准 = 任务的真实最终目标（参考系下记录，换到 world 后再用）。
      //   end_pt_ 之后会被 adjustGlobalTargetIfOccupied / 局部目标逻辑改写。
      //
      // 【降级幂等】只有"真正的新目标"才清降级/截断标记。
      //   外部全局规划器会周期性重发同一条路径（本函数上方注释已说明），
      //   若无条件清零，每收到一次就会重新降级一次 —— 实测同一目标被降级
      //   3~5 次，且落点在小范围内漂移（-5.58,7.13 / -5.42,7.18 …）。
      if (status_new_goal)
        resetMissionGoalState();

      // 目标点必须立刻换算到 world 系，否则 updatePlannerStatus 会拿陈旧/未初始化的
      // end_pt_ 做距离判定，机器人在旧目标附近且静止时会被误判为“已到达”。
      // 拿不到参考 TF 时先标记无效，等 monitorReferenceTransform 成功后再置位。
      Eigen::Isometry3d reference_to_world;
      if (lookupReferenceTransform(reference_to_world))
      {
        end_pt_ = reference_to_world * reference_end_pt_;
        // 任务的真实最终目标（world 系），到达判定只看它
        mission_goal_pt_ = end_pt_;
        status_target_valid_ = true;
      }
      else
      {
        status_target_valid_ = false;
      }

      // 目标变化 → 进入 running（重复路径不重置）
      if (status_new_goal)
        armPlannerStatus();

      if (reference_tf_jump_failed_)
      {
        reference_tf_jump_failed_ = false;
        reference_tf_stop_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "NEW_PATH");
      }
      /*** FSM ***/
      // 【V5】新路径同样能唤醒 EMERGENCY_STOP，但**排除 TF 停机的急停**：
      //   此时参考 TF 还不可用，新路径也无法换算到 world 系，
      //   若在这里唤醒就会变成"路径重发 → 唤醒 → 又 TF 超时 → 再急停"的高频震荡，
      //   而且机器人根本没停住。TF 恢复由 EMERGENCY_STOP 内的 TF_RECOVERED 分支负责。
      if (exec_state_ == WAIT_TARGET ||
          (exec_state_ == EMERGENCY_STOP && !reference_tf_stop_))
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
      else if (exec_state_ == REPLAN_TRAJ || exec_state_ == GEN_NEW_TRAJ)
      {
        // 新路径到达时，REPLAN_TRAJ/GEN_NEW_TRAJ 里的旧失败循环必须退出
        changeFSMExecState(GEN_NEW_TRAJ, "NEW_PATH");
      }

      // 打日志确认已接受参考路径
      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    // 失败 → 报错
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  // 回调：每收到一帧里程计就执行。
  // 作用：更新“我现在在哪、速度多少”，记录首帧高度，并画出机器人外形。
  // 这是整个程序的数据来源——没有里程计就寸步难行。
  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    // 更新当前位置 (x, y, z)
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // 模式 1：用首帧里程计的高度作为 RViz 目标点的默认高度（只做一次）
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    // 更新当前速度
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    // 更新当前姿态（四元数）
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // 【P21】记录实际走过的路径，供卡住后沿原路后撤（比“当前朝向正后方”可靠）
    {
      constexpr double kHistoryStep = 0.03;  // 每 3 cm 记一个点
      constexpr double kHistoryMax = 2.0;    // 最多保留 2 m
      if (odom_history_.empty())
      {
        odom_history_.push_back(odom_pos_);
      }
      else
      {
        const double step = (odom_pos_ - odom_history_.back()).norm();
        if (step > 0.5)
        {
          // 定位跳变/重置：旧历史不再可信
          odom_history_.clear();
          odom_history_.push_back(odom_pos_);
        }
        else if (step >= kHistoryStep)
        {
          odom_history_.push_back(odom_pos_);
          double total = 0.0;
          for (size_t i = odom_history_.size() - 1; i > 0; --i)
          {
            total += (odom_history_[i] - odom_history_[i - 1]).norm();
            if (total > kHistoryMax)
            {
              odom_history_.erase(odom_history_.begin(),
                                  odom_history_.begin() + static_cast<std::ptrdiff_t>(i - 1));
              break;
            }
          }
        }
      }
    }

    // 标记“已收到里程计”——这是状态机从 INIT 走出来的前提条件
    have_odom_ = true;
    // 预设航点模式：第一帧里程计到达后启动航点导航（只做一次）
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  // 回调：底盘告诉规划器“我正在原地转向，先别推进轨迹”（消息是 true/false）。
  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    // 记录底盘是否“冻结”（正在原地转向）
    go2_execution_frozen_ = msg->data;
  }

  // 处理上面的“冻结”信号：
  // 如果底盘正在原地转向，就把局部轨迹的开始时间往后推，等于让轨迹时钟暂停，
  // 这样轨迹不会在机器人还没转过来时就白白走完。
  // 把机器人实测位置投影到当前局部轨迹上，返回它对应的轨迹时刻。
  //   窗口 [-3.0, +1.0] 秒：偏向负方向是因为机器人通常落后；窗口化避免轨迹自交跳支；
  //   只比较 xy（轨迹与 odom 的 z 有约 0.14 m 偏置）。
  //   P0 阶段只用于诊断，不参与任何决策。
  double SCANReplanFSM::projectOdomToTrajParam(double t_hint) const
  {
    const LocalTrajData *info = &planner_manager_->local_data_;
    if (info->duration_ <= 1e-5)
      return std::max(0.0, t_hint);

    const double t_max = info->duration_;
    double best_t = std::min(std::max(t_hint, 0.0), t_max);
    // 窗口 [-1.0, +1.0] 秒：与控制器 projectOntoTrajClamped 保持一致。
    //   两边窗口不同会让"规划器判定的进度"与"控制器判定的进度"差出几秒，
    //   冻结/超前夹紧互相抵消（V23）。
    const double lo = std::max(0.0, t_hint - 1.0);
    const double hi = std::min(t_max, t_hint + 1.0);
    constexpr double step = 0.02;
    double best_d = std::numeric_limits<double>::max();
    for (double t = lo; t <= hi + 1e-9; t += step)
    {
      const Eigen::Vector3d p = info->position_traj_.evaluateDeBoorT(t);
      const double d = (p.head<2>() - odom_pos_.head<2>()).squaredNorm();
      if (d < best_d)
      {
        best_d = d;
        best_t = t;
      }
    }
    return best_t;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    // 距上次处理过了多少秒
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    // 更新“上次处理时间”
    last_freeze_update_time_ = now;

    // 时间差异常（<=0 或 >0.2 秒）→ 忽略本次，防止时钟跳变
    if (dt <= 0.0 || dt > 0.2)
      return;

    // 取出当前局部轨迹数据
    LocalTrajData *info = &planner_manager_->local_data_;
    // 轨迹还没开始 / 时长为 0 → 无事可做
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    // 【P4】轨迹时钟跟随实际进度：连续夹住，不允许 t_wall 比“实测投影进度”超前超过 max_lead。
    //   为什么改 start_time_ 而不是各个 t_cur 调用点：t_cur 有 5 个消费者
    //   （planFromCurrentTraj / setStartStateFromOdomOrCurrentTraj / checkCollisionCallback /
    //    EXEC_TRAJ 的重规划冷却 / reboundReplan 的 Case B），改时钟本身一处生效。
    const double t_wall = (now - info->start_time_).seconds();
    double t_proj = t_wall;
    if (have_odom_ && t_wall > 0.0)
      t_proj = projectOdomToTrajParam(t_wall);

    if (traj_clock_max_lead_ > 0.0 && have_odom_ && t_wall > 0.0)
    {
      const double lead = t_wall - t_proj;
      if (lead > traj_clock_max_lead_)
        info->start_time_ += rclcpp::Duration::from_seconds(lead - traj_clock_max_lead_);
    }

    // 1 Hz 诊断：t_cur 与"实际投影进度 + 允许超前量"的关系
    {
      const double t_cur_now = (now - info->start_time_).seconds();
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[traj-clock] t_wall=%.2f t_proj=%.2f t_cur=%.2f lead=%.2f max_lead=%.2f dur=%.2f traj_id=%d",
                           t_wall, t_proj, t_cur_now, t_cur_now - t_proj, traj_clock_max_lead_,
                           info->duration_, info->traj_id_);
    }

    // 原有逻辑：底盘原地转向 / 外部未使能期间 → 冻结轨迹时钟
    if (go2_execution_frozen_ || !motion_enabled_)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  // 收到新目标时进入 running；若之前是 goal_reached，这里把它重置。
  void SCANReplanFSM::armPlannerStatus()
  {
    // 新目标：清除重试退避，第一次尝试立即进行
    resetPlanRetry();
    // 【V5】新目标意味着"上一轮失败太多"的语义已失效：
    //   若不清掉，EMERGENCY_STOP 的 need_hover_stop_ 分支会立刻把新目标丢掉。
    need_hover_stop_ = false;
    // 新目标是新局面：清掉"连续失败放弃"的标记与计数，否则状态会一直报 failed
    status_gave_up_ = false;
    consecutive_plan_fails_ = 0;
    status_have_goal_ = true;
    status_is_stop_ = false;
    status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    status_problem_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    status_reason_ = ReplanFailure::NONE;
    exec_progress_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    exec_best_dist_ = 1e9;
    exec_best_tproj_ = 0.0;
    exec_no_progress_ = false;
    stuck_retry_count_ = 0;
    escape_active_ = false;
    escape_strategy_ = 0;
    if (!status_error_latched_)
      status_reasons_.clear();  // 锁存 error 后保留原因，供上报
    // 外部停止期间不发 running（状态话题保持 stopped）
    if (!externally_stopped_)
      publishPlannerStatus("running");
  }

  // 统计"执行进展"：只用物理量（到目标的实际距离 + 实际速度），不看时间。
  // 用"距离包络"：只有比历史最近还近 eps 才算有进展，绕路/回摆不算。
  // 外部使能回调：false → 冻结轨迹时钟、不规划、不发布；true → 从原处继续
  void SCANReplanFSM::motionEnabledCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    if (!msg)
      return;
    if (msg->data == motion_enabled_)
    {
      if (msg->data)
      {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "START ignored: motion already enabled; publish a new /global_path to change target");
      }
      return;
    }
    motion_enabled_ = msg->data;
    if (motion_enabled_)
      RCLCPP_INFO(node_->get_logger(), "motion_enabled=true: resume planning / cmd_vel");
    else
      RCLCPP_WARN(node_->get_logger(), "motion_enabled=false: freeze trajectory, stop cmd_vel");

    // 冻结期间缓存了新的全局路径、且当前没有目标 → 解冻时重建并规划
    if (motion_enabled_ && !have_target_ && last_global_path_ && !last_global_path_->poses.empty())
    {
      RCLCPP_INFO(node_->get_logger(), "motion enabled: planning from cached /global_path");
      resetPlanRetry();
      pathCallback(last_global_path_);
      // 只有参考轨迹真的建好了才进 GEN_NEW_TRAJ：
      //   否则 getLocalTarget 会在 duration<=0 时退化成"直奔远处终点"。
      if (have_target_ && hasUsableGlobalTraj())
        changeFSMExecState(GEN_NEW_TRAJ, "MOTION_ENABLE");
      else
        RCLCPP_WARN(node_->get_logger(),
                    "motion enabled: cached /global_path did not yield a usable global traj; "
                    "waiting for a new one");
    }
  }

  void SCANReplanFSM::updateExecutionProgress()
  {
    const rclcpp::Time now = node_->now();
    LocalTrajData *info = &planner_manager_->local_data_;

    // 【P21】用“局部目标距离”而不是全局终点距离：
    //   全局目标可能在 8 m 外，局部轨迹只有 4 m，用全局距离做 5 cm 级判据太粗。
    const Eigen::Vector3d goal = (local_target_pt_.head<2>().squaredNorm() > 1e-6)
                                     ? local_target_pt_
                                     : end_pt_;
    const double d = (goal.head<2>() - odom_pos_.head<2>()).norm();

    if (exec_progress_time_.nanoseconds() == 0)
    {
      exec_progress_time_ = now;
      exec_best_dist_ = d;
      exec_best_tproj_ = 0.0;
    }

    bool progressed = false;
    // 只有"更靠近目标"才算真正进展（会清掉脱困累计次数）。
    bool goal_progressed = false;
    if (d < exec_best_dist_ - stuck_progress_eps_)
    {
      exec_best_dist_ = d;
      progressed = true;
      goal_progressed = true;
    }

    // 【P21】轨迹投影进度：真正衡量“这条轨迹有没有在走”。
    //   只有实际投影向前推进超过 0.02 s 才算有进展。
    if (info->duration_ > 1e-5)
    {
      const double t_hint = std::max(0.0, (now - info->start_time_).seconds());
      const double t_proj = projectOdomToTrajParam(t_hint);
      if (t_proj > exec_best_tproj_ + 0.02)
      {
        exec_best_tproj_ = t_proj;
        progressed = true;
      }
    }

    if (progressed)
    {
      exec_progress_time_ = now;
      // 只有真正前进才清零重试计数，避免“规划成功但原地不动”无限重试
      stuck_retry_count_ = 0;
    }
    if (goal_progressed)
    {
      // 【V13】只有"更靠近目标"才清脱困累计次数与策略序号。
      //   轨迹投影推进（如后撤挪动一点）不算——否则会无限后撤。
      escape_strategy_ = 0;
      escape_total_ = 0;
      // 连续失败计数也在这里清零：这才是"任务确实在推进"的证据。
      //   （规划成功不算，见 notePlanResult 的说明）
      consecutive_plan_fails_ = 0;
    }

    exec_no_progress_ = (now - exec_progress_time_).seconds() >= stuck_window_ &&
                        odom_vel_.norm() < status_speed_tol_ &&
                        !go2_execution_frozen_;
  }

  // 沿实际走过路径回退 distance 米，返回该点（保证是机器人真实走过的位置）
  bool SCANReplanFSM::getBackupPointAlongHistory(double distance, Eigen::Vector3d &pt, double &used) const
  {
    used = 0.0;
    if (distance <= 0.0 || odom_history_.size() < 2)
      return false;

    double acc = 0.0;
    for (size_t i = odom_history_.size() - 1; i > 0; --i)
    {
      const Eigen::Vector3d &a = odom_history_[i];
      const Eigen::Vector3d &b = odom_history_[i - 1];
      const double seg = (a - b).norm();
      if (seg < 1e-6)
        continue;
      if (acc + seg >= distance)
      {
        const double r = (distance - acc) / seg;
        pt = a + r * (b - a);
        used = distance;
        return true;
      }
      acc += seg;
    }

    if (acc >= 0.10)
    {
      pt = odom_history_.front();
      used = acc;
      return true;
    }
    return false;
  }

  bool SCANReplanFSM::startBackupEscape(const char *reason)
  {
    // 进入脱困流程：清除重试退避，脱困结束后立即尝试正常规划
    resetPlanRetry();
    if (escape_active_ || escape_strategy_ >= 3)
      return false;
    // 【V13】累计后撤次数上限：单次"有位移"不清零，防止无限「重试→后撤→重试」
    if (max_escape_total_ > 0 && escape_total_ >= max_escape_total_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[FSM] backup escape budget exhausted (%d); giving up this segment",
                  escape_total_);
      return false;
    }

    static const double kScale[3] = {1.0, 0.6, 1.4};
    const double want = std::max(0.10, escape_distance_ * kScale[escape_strategy_ % 3]);

    Eigen::Vector3d pt;
    double used = 0.0;
    if (getBackupPointAlongHistory(want, pt, used))
    {
      escape_target_pt_ = pt;
      if (used + 1e-3 < want)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[FSM] backup history only %.2fm (wanted %.2fm); using available history",
                    used, want);
      }
    }
    else
    {
      const double yaw = getOdomYaw();
      escape_target_pt_ = odom_pos_ -
                          Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0) * want;
      escape_target_pt_(2) = odom_pos_(2);
      used = want;
      RCLCPP_WARN(node_->get_logger(),
                  "[FSM] no usable history; fallback to heading backup %.2fm", used);
    }

    if (!planner_manager_ ||
        !planner_manager_->EscapeBackup(odom_pos_, escape_target_pt_, escape_duration_))
      return false;

    publishCurrentTrajectory();
    escape_active_ = true;
    escape_start_pos_ = odom_pos_;
    traj_commit_wall_ = node_->now();
    stuck_retry_count_ = 0;
    ++escape_total_;

    RCLCPP_WARN(node_->get_logger(),
                "[FSM] stuck (%s): low-speed backup %.2fm in %.1fs to (%.2f,%.2f,%.2f)",
                reason, used, escape_duration_,
                escape_target_pt_(0), escape_target_pt_(1), escape_target_pt_(2));
    changeFSMExecState(EXEC_TRAJ, "STUCK_ESCAPE");
    return true;
  }

  void SCANReplanFSM::publishCurrentTrajectory()
  {
    auto info = &planner_manager_->local_data_;
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
      bspline.knots.push_back(knots(i));

    bspline_pub_->publish(bspline);
    publishLocalPath(info->position_traj_);
  }

  // 每个 tick 判定运行状态，按优先级发到 /local_planner/status：
  //   disabled(未使能) > paused(任务暂停) > idle(无目标/已到达) > error(锁存) > 到达判定
  // 状态发生切换时【立即发布】；同一状态才按 1Hz 心跳。
  void SCANReplanFSM::updatePlannerStatus()
  {
    const rclcpp::Time now = node_->now();
    const bool heartbeat_due =
        status_heartbeat_hz_ > 1e-6 &&
        (now - status_last_pub_time_).seconds() >= 1.0 / status_heartbeat_hz_;

    std::string state;

    if (!motion_enabled_)
    {
      // 外部未使能（遥控器未 START / 按 B / 发布者掉线）⇒ 安全回路未上使能
      state = status_name_disabled_;
    }
    else if (externally_stopped_)
    {
      // /mission/control="pause" ⇒ 任务暂停（轨迹已清）
      state = status_name_paused_;
    }
    else if (status_gave_up_)
    {
      // 【连续失败超限】任务已被主动放弃：必须排在 !status_have_goal_ 之前，
      //   否则会因为目标已清空而被报成 idle，上层看不到"这次任务失败了"。
      //   收到新目标时由 armPlannerStatus/notePlanResult 清除。
      state = "failed";
    }
    else if (!status_have_goal_)
    {
      state = "idle";
    }
    else if (status_is_stop_)
    {
      // 已到达目标、且 goal_reached 已发过一次 → 转为 idle，等待新目标
      state = "idle";
    }
    else if (!status_target_valid_)
    {
      // 目标点还没换算到 world 系（mode3 刚收到新路径、参考 TF 尚未就绪）。
      // 此时 end_pt_ 不可信，不能做到达判定，否则会拿陈旧/未初始化的目标误判“已到达”。
      state = "running";
    }
    else if (status_error_latched_)
    {
      // 硬错误已锁存：error 不自动恢复，只有重启节点才清除
      state = "error";
    }
    else
    {
      // 到达判定：分「进入候选」与「候选保持」两段，加滞回 + 目标点快照。
      //   - 进入候选：距目标(2D) < status_goal_tol_ 且 速度 < status_speed_tol_
      //   - 候选期间：目标点锁存进 status_goal_snapshot_（end_pt_ 会随定位刷新而漂移，
      //               锁存保证判定基准稳定），并用更宽的退出阈值判断是否掉出候选
      //   - 连续保持 >= status_stop_debounce_ → goal_reached（只发一次）
      const double speed_now = odom_vel_.norm();
      const bool candidate_active = status_hold_since_.nanoseconds() != 0;

      if (candidate_active)
      {
        const double d_snap = (status_goal_snapshot_.head<2>() - odom_pos_.head<2>()).norm();
        if (d_snap >= status_goal_tol_exit_ || speed_now >= status_speed_tol_exit_)
        {
          // 掉出候选窗口（滞回）→ 计时清零，重新判断
          status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
          state = "running";
        }
        else if ((now - status_hold_since_).seconds() >= status_stop_debounce_)
        {
          status_is_stop_ = true;
          state = "goal_reached";
        }
        else
        {
          state = "running";
        }
      }
      else
      {
        // 【V3】到达判定只看任务的真实最终目标 mission_goal_pt_，不看 end_pt_。
        //   end_pt_ 会被 adjustGlobalTargetIfOccupied（目标被占时截断到半路）改写，
        //   用它判到达会在"原目标不可达"时误报 goal_reached。
        //
        // 例外：目标被判为不可达（占据）并主动截断时，判定基准改为
        //   "实际可达的最后一个自由点"——否则永远差一截，任务无法收尾。
        const Eigen::Vector3d &arrival_ref =
            (mission_goal_truncated_ && have_truncated_reachable_pt_) ? truncated_reachable_pt_
                                                                     : mission_goal_pt_;
        const bool arrived =
            (arrival_ref.head<2>() - odom_pos_.head<2>()).norm() < status_goal_tol_;
        const bool velocity_zero = speed_now < status_speed_tol_;
        // 注意：mission_goal_truncated_ 不能算作 problem ——
        //   "目标不可达、停在可达点"是一次**正常的任务完成**，
        //   若计入 problem，状态话题会永远停在 failed（永不自恢复）。
        const bool problem = (replan_fail_count_ != 0) || (exec_state_ == EMERGENCY_STOP);

        if (arrived && velocity_zero)
        {
          // 进入到达候选：锁存目标点 + 开始去抖计时
          status_hold_since_ = now;
          status_goal_snapshot_ = arrival_ref;
          state = "running";
        }
        else
        {
          state = "running";
          if (!arrived)
          {
            if (problem)
            {
              if (status_problem_since_.nanoseconds() == 0)
                status_problem_since_ = now;
            }
            else
            {
              status_problem_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
              status_reason_ = ReplanFailure::NONE;
              status_reasons_.clear();
            }
            if (problem && status_problem_since_.nanoseconds() != 0 &&
                (now - status_problem_since_).seconds() >= status_error_time_)
            {
              // 按"原因是否必须人工干预"分级：
              //   硬错误 → error（是否锁存由 fsm.status_latch_hard_error 决定）
              //   软错误 → failed（有错但仍能跑）
              bool needs_intervention = false;
              for (const auto &r : status_reasons_)
              {
                if (replanFailureNeedsIntervention(r))
                {
                  needs_intervention = true;
                  break;
                }
              }
              if (needs_intervention && status_latch_hard_error_)
              {
                status_error_latched_ = true;  // 锁存：只有重启节点才清除
                state = "error";
              }
              else
              {
                // 未启用锁存（默认）时，硬错误也只报 failed，故障消失即可自恢复
                state = "failed";
              }
            }
          }
          else
          {
            // 已到达（去抖中/仍在滑行）→ 不算 error
            status_problem_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
            status_reason_ = ReplanFailure::NONE;
            status_reasons_.clear();
          }
        }
      }
    }

    // 状态切换 → 立即发布；同状态 → 1Hz 心跳（goal_reached 只发一次）
    // 【V17】比较对象必须是"门控后的实际状态"，与 status_last_state_ 的存储口径一致，
    //   否则 disabled/paused 期间每 10 ms 都会被判成一次切换并重发。
    const std::string effective = gateStatusName(state);
    if (effective != status_last_state_)
    {
      publishPlannerStatus(state);
    }
    else if (effective != "goal_reached" && heartbeat_due)
    {
      publishPlannerStatus(state);
    }
  }

  // 把门控（未使能 / 任务暂停 / error 锁存）施加到状态名上。
  std::string SCANReplanFSM::gateStatusName(const std::string &state) const
  {
    std::string out = state;
    if (!motion_enabled_ && out != status_name_disabled_)
      out = status_name_disabled_;                 // 外部未使能
    // 未使能优先于 error 锁存：没上使能时报 disabled 更有信息量（安全回路没通）。
    //   注意 paused 不需要在这里单独排除：updatePlannerStatus 里 paused 是
    //   externally_stopped_ 分支给出的，而 error 锁存只可能出现在"有目标在跑"
    //   的分支里，两者不会同时成立。
    else if (status_error_latched_ && out != status_name_disabled_)
      out = "error";
    return out;
  }

  // 把状态发到 /local_planner/status（std_msgs/String，内容为 JSON）；空串不发布。
  void SCANReplanFSM::publishPlannerStatus(const std::string &state)
  {
    if (state.empty())
      return;
    const std::string out = gateStatusName(state);
    std_msgs::msg::String msg;
    msg.data = buildStatusJson(out);
    status_pub_->publish(msg);
    status_last_pub_time_ = node_->now();
    status_last_state_ = out;
  }

  // 拼 JSON：{"state":..,"msgs":["原因1","原因2",...]}
  // “如果没有门控，本该报什么状态”：只读、无副作用，供 JSON 的 inner 字段使用
  // 门控（未使能/暂停）期间用于诊断“本来应该是什么状态”
  std::string SCANReplanFSM::statusInnerState() const
  {
    if (!status_have_goal_)                       return "idle";
    if (status_is_stop_)                          return "idle";
    if (status_error_latched_)                    return "error";
    if (status_problem_since_.nanoseconds() != 0) return "failed";  // 失败计时中（近似）
    return "running";
  }

  std::string SCANReplanFSM::buildStatusJson(const std::string &state)
  {
    std::string msgs = "[]";
    if (state == "error" || state == "failed")
    {
      std::string arr = "[";
      for (size_t i = 0; i < status_reasons_.size(); ++i)
      {
        if (i > 0)
          arr += ",";
        arr += "\"";
        arr += replanFailureToString(status_reasons_[i]);
        arr += "\"";
      }
      arr += "]";
      msgs = arr;
    }
    std::string inner = statusInnerState();
    std::string json = "{\"state\":\"" + state + "\",\"msgs\":" + msgs;
    json += ",\"enabled\":" + std::string(motion_enabled_ ? "true" : "false");
    json += ",\"paused\":" + std::string(externally_stopped_ ? "true" : "false");
    json += ",\"latched\":" + std::string(status_error_latched_ ? "true" : "false");
    json += ",\"problem\":" + std::string(((replan_fail_count_ != 0) || (exec_state_ == EMERGENCY_STOP)) ? "true" : "false");
    json += ",\"have_goal\":" + std::string(status_have_goal_ ? "true" : "false");
    json += ",\"rid\":" + std::to_string(replan_rid_);
    json += ",\"traj_id\":" + std::to_string(planner_manager_ ? planner_manager_->local_data_.traj_id_ : -1);
    json += ",\"replan_fail\":" + std::to_string(replan_fail_count_);
    json += ",\"retry\":" + std::to_string(plan_retry_fails_);
    json += ",\"inner\":\"" + inner + "\"";
    json += "}";
    return json;
  }

  // 记录一次失败原因：设为当前原因，并把新出现的原因累积到本次 error 的原因列表里
  void SCANReplanFSM::setStatusReason(ReplanFailure r)
  {
    if (r != status_reason_ && r != ReplanFailure::NONE)
      RCLCPP_INFO(node_->get_logger(), "[diag] rid=%llu stage=status event=reason reason=%s replan_fail=%d",
                  static_cast<unsigned long long>(replan_rid_), replanFailureToString(r), replan_fail_count_);
    status_reason_ = r;
    if (r == ReplanFailure::NONE || replanFailureIsBenign(r))
      return;
    if (std::find(status_reasons_.begin(), status_reasons_.end(), r) == status_reasons_.end())
      status_reasons_.push_back(r);
  }

  // 任务控制回调：/mission/control → "pause"（清轨迹停住）/ "resume"（重建并继续）
  void SCANReplanFSM::missionControlCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
  {
    if (!msg)
      return;
    if (msg->data == "pause")
    {
      if (!externally_stopped_)
        applyExternalStop();
    }
    else if (msg->data == "resume")
    {
      if (externally_stopped_)
        applyExternalResume();
    }
  }

  // 外部停止：清全局参考轨迹 + 清局部轨迹（机器人由控制器停住）；局部地图照常更新。
  void SCANReplanFSM::applyExternalStop()
  {
    externally_stopped_ = true;
    // 立即刹车：给控制器发一条“停在原地”的轨迹。
    // 不这样做的话，控制器手上还握着上一条局部轨迹，会【走完那一小段】才停。
    // 还没有里程计时不发（此时 odom_pos_ 未初始化）；控制器没里程计本来就会发 0 速度。
    if (have_odom_)
      callEmergencyStop(odom_pos_);
    if (!status_error_latched_)
      status_reasons_.clear();  // 锁存 error 后保留原因，供上报
    // 清全局参考轨迹
    planner_manager_->global_data_.global_duration_ = 0.0;
    planner_manager_->global_data_.last_progress_time_ = 0.0;
    reference_end_pt_.setZero();
    end_pt_.setZero();
    end_vel_.setZero();
    // 目标已清空：到达判定必须先失效，等新路径重新换算后再置位
    status_target_valid_ = false;
    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    have_last_reference_transform_ = false;
    // 【V8】脱困/失败状态必须一起清：否则它们会泄漏到 resume，
    //   replan_fail_count_ 超限时 resume 后第一拍就被 finishProcess 打成后撤/急停。
    escape_active_ = false;
    escape_strategy_ = 0;
    escape_total_ = 0;
    stuck_retry_count_ = 0;
    replan_fail_count_ = 0;
    exec_no_progress_ = false;
    exec_progress_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    resetPlanRetry();
    need_hover_stop_ = false;
    // 清局部轨迹
    planner_manager_->local_data_.duration_ = 0.0;
    planner_manager_->local_data_.start_time_ =
        rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      // 状态 → paused（立即发一次，之后 1Hz）
    status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    publishPlannerStatus(status_name_paused_);
    RCLCPP_WARN(node_->get_logger(),
                "External STOP: global/local trajectories cleared; local map kept");
  }

  // 外部继续：(a) 有缓存的全局路径 → 重建并规划；(b) 否则等新的 /global_path。
  void SCANReplanFSM::applyExternalResume()
  {
    externally_stopped_ = false;
    status_is_stop_ = false;  // 恢复后重新判定是否到达
    // 恢复：清除重试退避，立即尝试规划
    resetPlanRetry();
    if (!status_error_latched_)
      status_reasons_.clear();  // 锁存 error 后保留原因，供上报
    if (last_global_path_ && !last_global_path_->poses.empty())
    {
      RCLCPP_INFO(node_->get_logger(), "External RESUME: rebuilding from cached /global_path");
      pathCallback(last_global_path_);
      // V1：必须确认参考轨迹真的建好了再切 GEN_NEW_TRAJ。
      //   否则 duration=0 时 getLocalTarget 会退化成"直奔远处终点"，绕过 planning_horizon。
      if (have_target_ && hasUsableGlobalTraj())
        changeFSMExecState(GEN_NEW_TRAJ, "RESUME");
      else
      {
        RCLCPP_WARN(node_->get_logger(),
                    "External RESUME: cached /global_path produced no usable global traj; "
                    "waiting for a new /global_path");
        changeFSMExecState(WAIT_TARGET, "RESUME_FAILED");
      }
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "External RESUME: waiting for a new /global_path");
      changeFSMExecState(WAIT_TARGET, "RESUME");
    }
    // 状态话题：有目标则立即回到 running；无目标由 update 发 idle
    if (status_have_goal_)
      publishPlannerStatus("running");
  }

  // 从里程计的姿态里取出“偏航角 yaw”（即机器人朝哪个方向）。
  double SCANReplanFSM::getOdomYaw() const
  {
    // 取旋转矩阵第一列，即机器人朝向在水平面的投影
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    // 投影太小（数值退化）→ 无法判断，返回 0
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    // 用 atan2 求出偏航角
    return std::atan2(heading(1), heading(0));
  }

  // 用“从 from 指向 to 的这段线段”来估计朝向。
  // 因为碰撞检测要知道机器人朝哪个方向（机器人是前后两个圆柱），
  // 所以凡是查障碍都要带上一个 yaw，这个函数就是用来算 yaw 的。
  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    // 用 from→to 的水平方向作为朝向
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    // 两点几乎重合 → 退回用里程计的朝向
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  // 发布机器人自身外形：在 RViz 里画前后两个半透明圆柱，
  // 它俩就是碰撞检测所用的“机器人形状”。
  void SCANReplanFSM::publishSelfInflationMarker()
  {
    // 读取半径和上下膨胀高度（都保证非负）
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    // 圆柱总高度（至少 1 毫米，避免退化成 0）
    const double height = std::max(1e-3, z_up + z_down);

    // 新建一个 Marker（RViz 里的可视化图元）
    visualization_msgs::msg::Marker marker;
    // 坐标框架：参数里没配就用 world
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    // 时间戳
    marker.header.stamp = node_->now() - rclcpp::Duration::from_seconds(0.1);
    marker.ns = "self_inflation";
    // 形状选圆柱；动作是“添加/更新”
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // 尺寸：直径 × 直径 × 高度
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    // 颜色：浅蓝、半透明
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    // 生命周期 2 秒：既要“过期自动消失，避免残影”，又不能短到在重规划期间
    // 被 RViz 判为过期（marker 由里程计回调发出，若规划占住执行器，回调会短暂停顿，
    // 0.2 秒会因此整块闪掉）。2 秒足以跨过规划停顿，规划器退出后也会自动消失。
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);

    // 圆柱中心：机器人位置，高度方向按上下膨胀不对称做补偿
    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    // 沿朝向方向，算出“前圆柱”和“后圆柱”的圆心
    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    // 先发布前圆柱
    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    // 再发布后圆柱
    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  // 发布“倾斜双圆柱 + 姿态轴线 + 角度文字”，用于确认姿态数据/轴向是否正确。
  // 与检测窗口（self_inflation）使用独立命名空间和颜色，不参与碰撞查询。
  void SCANReplanFSM::publishBodyPoseVisualization()
  {
    auto map = planner_manager_->grid_map_;
    if (!map)
      return;

    const Eigen::Quaterniond q = map->getBodyQuaternion();
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const Eigen::Vector3d fwd = R.col(0);
    const Eigen::Vector3d up = R.col(2);

    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);
    const double zshift = 0.5 * (z_up - z_down);
    const double off = self_double_cylinder_offset_;

    // 一轮只发一条 MarkerArray：队列不会只收到其中一半，避免圆柱间歇消失。
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now() - rclcpp::Duration::from_seconds(0.1);
    marker.ns = "body_pose_visual";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    // 与 self_inflation 一致：跨过规划停顿，避免整块闪掉
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);

    // 按俯仰角分档着色（第一版）
    const double pitch_deg = std::abs(map->getBodyPitch()) * 180.0 / M_PI;
    if (pitch_deg < 10.0)
    {
      marker.color.r = 0.35; marker.color.g = 0.80; marker.color.b = 0.85;
    }
    else if (pitch_deg < 20.0)
    {
      marker.color.r = 0.95; marker.color.g = 0.75; marker.color.b = 0.25;
    }
    else
    {
      marker.color.r = 0.92; marker.color.g = 0.42; marker.color.b = 0.30;
    }
    marker.color.a = 0.30;

    // 两个倾斜圆柱：整个组件随机身旋转，长轴仍沿机体上方向
    for (int i = 0; i < 2; ++i)
    {
      const double s = (i == 0) ? 1.0 : -1.0;
      const Eigen::Vector3d c = odom_pos_ + s * off * fwd + zshift * up;
      marker.id = 10 + i;
      marker.pose.position.x = c(0);
      marker.pose.position.y = c(1);
      marker.pose.position.z = c(2);
      arr.markers.push_back(marker);
    }

    // 姿态轴线
    {
      visualization_msgs::msg::Marker axis;
      axis.header = marker.header;
      axis.ns = "body_pose_visual";
      axis.id = 12;
      axis.type = visualization_msgs::msg::Marker::LINE_STRIP;
      axis.action = visualization_msgs::msg::Marker::ADD;
      axis.pose.orientation.w = 1.0;
      axis.scale.x = 0.015;
      axis.color.r = 1.0;
      axis.color.g = 0.55;
      axis.color.b = 0.10;
      axis.color.a = 1.0;
      axis.lifetime = marker.lifetime;
      const Eigen::Vector3d a = odom_pos_ - (off + radius) * fwd + zshift * up;
      const Eigen::Vector3d b = odom_pos_ + (off + radius) * fwd + zshift * up;
      geometry_msgs::msg::Point pa;
      pa.x = a(0); pa.y = a(1); pa.z = a(2);
      geometry_msgs::msg::Point pb;
      pb.x = b(0); pb.y = b(1); pb.z = b(2);
      axis.points.push_back(pa);
      axis.points.push_back(pb);
      arr.markers.push_back(axis);
    }

    // 角度文字：贴在前圆柱上方，逐帧随 MarkerArray 发布
    {
      visualization_msgs::msg::Marker text;
      text.header = marker.header;
      text.ns = "body_pose_visual";
      text.id = 13;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.orientation.w = 1.0;
      const double text_h = 0.09;
      // 前圆柱中心 → 顶端 → 文字中心（留 3 cm 间隙）
      const Eigen::Vector3d c = odom_pos_ + off * fwd
          + (zshift + 0.5 * height + 0.5 * text_h + 0.03) * up;
      text.pose.position.x = c(0);
      text.pose.position.y = c(1);
      text.pose.position.z = c(2);
      text.scale.z = text_h;
      text.color.r = 1.0;
      text.color.g = 0.90;
      text.color.b = 0.20;
      text.color.a = 1.0;
      text.lifetime = marker.lifetime;
      char buf[128];
      std::snprintf(buf, sizeof(buf), "pitch: %+.1f°　　roll: %+.1f°",
                    map->getBodyPitch() * 180.0 / M_PI,
                    map->getBodyRoll() * 180.0 / M_PI);
      text.text = buf;
      arr.markers.push_back(text);
    }

    body_pose_visual_pub_->publish(arr);
  }

  // 切换状态（例如 GEN_NEW_TRAJ → EXEC_TRAJ），同时打印一行日志。
  // pos_call 只是日志里说明“是谁触发的切换”。
  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    // 同一状态的重复调用只累计次数，不打印：
    //   失败重试会反复进入同一状态，打印 "from X to X" 只会刷屏。
    if (new_state == exec_state_)
    {
      continuously_called_times_++;
      return;
    }
    continuously_called_times_ = 1;

    // 状态名表（下标对应枚举值，仅用于打印）
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    RCLCPP_INFO(node_->get_logger(), "[fsm] event=state from=%s to=%s trigger=%s",
                state_str[pre_s].c_str(), state_str[int(new_state)].c_str(), pos_call.c_str());
  }

  // 返回 <当前状态已经连续被调用了多少次, 当前状态>，用来做重试策略。
  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    // 返回 <同一状态连续被调用的次数, 当前状态>
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  // 【重试节流】本拍是否允许再尝试一次规划。
  //   失败后 FSM 会留在原状态（GEN_NEW_TRAJ / REPLAN_TRAJ），主循环 10 ms 一拍就再来一次，
  //   而两次之间位置/地图/目标都没变，注定失败。间隔按连续失败次数翻倍：
  //   min×2^(fails-1)，封顶 max，成功清零（0.2 → 0.4 → 0.8 → 1.0 …）。
  //   具体算法在 plan_manage/replan_retry.h（有独立单测）。
  bool SCANReplanFSM::planAttemptAllowed()
  {
    if (!replan_retry_enable_)
      return true;
    if (last_plan_attempt_.nanoseconds() == 0)
      return true;
    const double elapsed = (node_->now() - last_plan_attempt_).seconds();
    return replanRetryAllowed(true, elapsed, plan_retry_fails_,
                              replan_retry_min_, replan_retry_max_);
  }

  void SCANReplanFSM::notePlanAttempt()
  {
    last_plan_attempt_ = node_->now();
  }

  void SCANReplanFSM::notePlanResult(bool ok)
  {
    if (ok)
    {
      plan_retry_fails_ = 0;
      // 成功后不保留冷却：下一次（例如安全重规划）应立即允许
      last_plan_attempt_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      status_gave_up_ = false;
      // 注意：consecutive_plan_fails_ **不在这里清零**。
      //   实测（目标在障碍里）会出现"规划出 4~5 个控制点的短轨迹 → ok=1 →
      //   走一点 → 再 optimize_failed"的循环，success 每几秒就出现一次。
      //   若按 success 清零，这个兜底永远不会触发。
      //   真正的"有出路"证据是**执行有真实进展**（更靠近目标），
      //   清零放在 updateExecutionProgress 的 goal_progressed 分支。
    }
    else
    {
      plan_retry_fails_ = std::min(plan_retry_fails_ + 1, 100);
      consecutive_plan_fails_ = std::min(consecutive_plan_fails_ + 1, 1000000);
    }
  }

  void SCANReplanFSM::resetPlanRetry()
  {
    plan_retry_fails_ = 0;
    last_plan_attempt_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    // 注意：故意**不**清 consecutive_plan_fails_。
    //   resetPlanRetry 在"每次重试/脱困/恢复"时都会被调用；若在那里清零，
    //   "失败→脱困→再失败"的循环就永远攒不到上限，兜底会失效。
  }

  // 放弃当前任务：清目标、清轨迹、复位到达判定，并让状态话题回到 idle。
  //   give-up（STUCK_GIVEUP）与"失败太多"的急停退出都走这里。
  //   关键点：status_have_goal_ 必须一起清，否则 updatePlannerStatus 会一直报 running。
  void SCANReplanFSM::abandonMission(const char *reason)
  {
    if (stop_traj_on_finish_ && have_odom_)
      callEmergencyStop(odom_pos_);

    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    need_hover_stop_ = false;
    stuck_retry_count_ = 0;
    // 【问题4修正】放弃任务时把两个失败计数器一起清零。
    //   原来只清 stuck_retry_count_，导致 replan_fail_count_ 残留：
    //   若它已累积到接近 max_replan_fail_count_，下一个任务一开始就可能立刻
    //   触发脱困后撤，或直接被判定为"失败太多"。
    replan_fail_count_ = 0;
    consecutive_plan_fails_ = 0;
    // 脱困状态也必须复位：否则 escape_active_ 残留会让下一条轨迹被当成
    //   "后撤轨迹"整段劫持（EXEC_TRAJ 的 escape 分支会提前 return）。
    escape_active_ = false;
    escape_strategy_ = 0;
    escape_total_ = 0;

    // 目标已清空：到达判定与运行状态都必须失效
    status_have_goal_ = false;
    status_target_valid_ = false;
    status_is_stop_ = false;
    status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    status_problem_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    // 注意：这里**不改** status_gave_up_ 与 status_reasons_ 的取舍由调用方决定：
    //   正常收尾（到达/航点走完）应让状态回到 idle，而"连续失败放弃"要保留 failed。
    if (!status_error_latched_ && !status_gave_up_)
    {
      status_reason_ = ReplanFailure::NONE;
      status_reasons_.clear();
    }

    if (isWaypointSequenceMode())
    {
      active_waypoints_.clear();
      current_wp_ = 0;
    }

    changeFSMExecState(WAIT_TARGET, reason);
    const bool normal_done = std::string(reason) == "GOAL_REACHED" ||
                             std::string(reason) == "WAYPOINTS_DONE";
    if (normal_done)
      RCLCPP_INFO(node_->get_logger(), "[fsm] event=mission_done reason=%s rid=%llu",
                  reason, static_cast<unsigned long long>(replan_rid_));
    else
      RCLCPP_WARN(node_->get_logger(), "[fsm] event=mission_abandoned reason=%s rid=%llu",
                  reason, static_cast<unsigned long long>(replan_rid_));
  }

  // 全局轨迹是否可用于规划：navi_mode=3 的 resume 必须等参考轨迹真正建好，
  // 否则 getLocalTarget 会在 duration<=0 时退化成"直奔远处终点"。
  bool SCANReplanFSM::hasUsableGlobalTraj() const
  {
    if (!planner_manager_)
      return false;
    return planner_manager_->global_data_.global_duration_ > 1e-3;
  }

  // 打印当前状态名（调试用）。
  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    // 打印当前状态名（调试用）
    RCLCPP_DEBUG(node_->get_logger(), "[FSM] state: %s", state_str[int(exec_state_)].c_str());
  }

  // ==========================================================================
  // 【核心】execFSMCallback()：每 10 毫秒被调用一次，是状态机的主循环。
  //   1. 先处理“冻结”：如果底盘原地转向，就暂停轨迹时钟
  //   2. 每隔约 1 秒打印一次当前状态（方便观察）
  //   3. 用 switch 分派到六个状态中的一个，执行那个状态要做的事
  //   4. 收尾：finishProcess()，并发布调试数据
  // 下面每个 case 就是“某个状态下要做什么”。
  // ==========================================================================
  void SCANReplanFSM::execFSMCallback()
  {
    // 第一步：处理“冻结”（底盘原地转向时暂停轨迹时钟）
    updateLocalTrajTimeFreeze();
    // 同步运行状态（running/goal_reached/stopped）；放在最前，避免被后续的提前 return 跳过
    updatePlannerStatus();
    // 【问题5修正】参考 TF 监控必须每拍都跑，包括门控（paused / disabled）期间。
    //   原来它排在两个门控 return 之后，于是门控期间 reference_tf_missing_since_
    //   不会被更新：等恢复时若 TF 已缺失很久，会立刻拿一个陈旧的时间戳判超时急停。
    //   这里只做"监控与记账"，不推进状态机；真正的状态迁移仍受门控约束。
    const bool reference_ok = monitorReferenceTransform();
    // 外部停止（/mission/control = pause）：不重规划、不推进，直到 resume
    if (externally_stopped_)
      return;
    // 外部未使能（未按 START / 按了 B）：冻结，不重规划、不发布
    if (!motion_enabled_)
      return;
    // 【V14】监控结果不能用来"冻结整个 FSM"：monitorReferenceTransform() 自己会在
    //   超宽限时切 EMERGENCY_STOP，而那个状态的 TF 恢复分支必须能继续跑到。
    (void)reference_ok;

    // 下面几行：每 100 次（约 1 秒）打印一次当前状态
    static int fsm_num = 0;
    fsm_num++;
    // 每 100 次才打印一次
    if (fsm_num == 100)
    {
      // 打印状态；顺便提示是否还缺里程计 / 目标
      printFSMExecState();
      if (!have_odom_)
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "[fsm] event=no_odom");
      if (!trigger_)
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "[fsm] event=wait_goal");
      // 计数清零
      fsm_num = 0;
    }

    // 第二步：按当前状态分派到不同分支
    switch (exec_state_)
    {
    // 状态 INIT：刚启动。等“有里程计”且“有目标触发”后，进入 WAIT_TARGET。
    // ---- 状态 INIT：等待“有里程计”且“有触发” ----
    case INIT:
    {
      // 还没收到里程计 → 继续等待
      if (!have_odom_)
      {
        return;
      }
      // 还没被触发 → 继续等待
      if (!trigger_)
      {
        return;
      }
      // 两个条件都满足 → 进入 WAIT_TARGET
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    // 状态 WAIT_TARGET：等待目标点。一旦有目标，就进入 GEN_NEW_TRAJ。
    // ---- 状态 WAIT_TARGET：等待目标点 ----
    case WAIT_TARGET:
    {
      // 还没有目标 → 什么都不做
      if (!have_target_)
        return;
      else
      {
        // 有目标了 → 去生成新轨迹
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    // 状态 GEN_NEW_TRAJ：生成一条全新轨迹。
    // 先决定起点用哪个数据，再调用 callReboundReplan() 让总管规划。
    // 成功 → EXEC_TRAJ；失败 → 记录失败次数，等待重试。
    // ---- 状态 GEN_NEW_TRAJ：生成一条全新轨迹 ----
    case GEN_NEW_TRAJ:
    {
      // 【重试节流】同一状态下连续重试时，至少隔 replan_retry_min..max 秒再试一次。
      //   失败后 FSM 会留在本状态，主循环 10 ms 一拍就再来一次；不节流就是每秒十几次
      //   注定失败的重规划。用 break 让本拍的收尾逻辑照跑。
      if (!planAttemptAllowed())
        break;
      notePlanAttempt();
      // 先确定起点（默认里程计；有轨迹时用轨迹上的速度/加速度）
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      // 第一次调用用普通多项式；反复失败后改用“插随机点”的方式
      bool flag_random_poly_init;
      // 判断是不是本状态的第一次调用
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      // 交给规划总管规划，返回是否成功
      bool success = callReboundReplan(true, flag_random_poly_init);
      // 成功 → 失败计数清零，进入“执行轨迹”状态
      if (success)
      {

        replan_fail_count_ = 0;
        notePlanResult(true);
        changeFSMExecState(EXEC_TRAJ, "FSM");
        // 允许后续从急停中恢复
        flag_escape_emergency_ = true;
      }
      else
      // 失败 → 失败计数 +1，留在本状态下轮重试
      {
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_jump_replan_)
        {
          reference_tf_stop_ = true;
          reference_tf_jump_failed_ = true;
          setStatusReason(ReplanFailure::EMERGENCY_TF_JUMP_FAIL);
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          // 进入 TF 跳变急停：清除重试退避，故障解除后立即恢复
          resetPlanRetry();
          changeFSMExecState(EMERGENCY_STOP, "TF_JUMP_FAIL");
        }
        else
      {
        // 已到局部目标：消费为到达，避免 GEN_NEW_TRAJ 自循环
        if (status_reason_ == ReplanFailure::CLOSE_TO_GOAL)
        {
          notePlanResult(true);
          completeCurrentSegment("CLOSE_TO_GOAL");
          return;
        }
        if (!replanFailureIsBenign(status_reason_)) replan_fail_count_++;
        notePlanResult(false);
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      }
      break;
    }

    // 状态 REPLAN_TRAJ：在已有轨迹基础上重新规划（例如偏离了、有障碍了）。
    // ---- 状态 REPLAN_TRAJ：重新规划 ----
    case REPLAN_TRAJ:
    {
      // 【重试节流】同 GEN_NEW_TRAJ：失败后不要 10 ms 一拍地重试注定失败的重规划。
      if (!planAttemptAllowed())
        break;
      notePlanAttempt();
      // 从当前轨迹出发重新规划；若是参考系跳变触发的重规划，
      // 则以当前实测状态为起点（旧轨迹的预测状态已失效）
      bool success;
      if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_jump_replan_)
      {
        // 定位修正后旧轨迹的预测状态已失效，改从实测状态起算
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
        success = callReboundReplan(true, false);
      }
      else
        success = planFromCurrentTraj();
      if (success)
      {
        reference_jump_replan_ = false;
        // 成功 → 进入执行状态
        replan_fail_count_ = 0;
        notePlanResult(true);
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // 参考系跳变导致的重规划失败 → 直接急停，避免继续跑在失效轨迹上
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_jump_replan_)
        {
          reference_tf_stop_ = true;
          reference_tf_jump_failed_ = true;
          setStatusReason(ReplanFailure::EMERGENCY_TF_JUMP_FAIL);
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          // 进入 TF 跳变急停：清除重试退避，故障解除后立即恢复
          resetPlanRetry();
          changeFSMExecState(EMERGENCY_STOP, "TF_JUMP_FAIL");
        }
        else
      {
        // 已到局部目标：这是“到达”，不是失败，不能继续无限 REPLAN
        if (status_reason_ == ReplanFailure::CLOSE_TO_GOAL)
        {
          notePlanResult(true);
          completeCurrentSegment("CLOSE_TO_GOAL");
          return;
        }
        // 失败 → 计数 +1，保持重规划状态继续尝试
        if (!replanFailureIsBenign(status_reason_)) replan_fail_count_++;
        notePlanResult(false);
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      }

      break;
    }

    // 状态 EXEC_TRAJ：正在沿轨迹执行。
    // 计算“已经走了多久 t_cur”，取出当前位置；
    // 判断是否快到终点、是否该前往下一个航点、是否需要重新规划。
    // ---- 状态 EXEC_TRAJ：正在沿轨迹执行 ----
    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      // 取出当前局部轨迹数据
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      // 已经执行了多少秒
      double t_cur = (time_now - info->start_time_).seconds();
      // 把时间夹在 [0, 总时长] 之内
      t_cur = min(info->duration_, t_cur);

      // 当前时刻应该到达的位置
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // 【P22】后撤轨迹执行期间不做普通重规划，等它走完/超时再恢复
      if (escape_active_)
      {
        const double escape_wall = (traj_commit_wall_.nanoseconds() == 0)
                                       ? 0.0
                                       : (time_now - traj_commit_wall_).seconds();
        const bool done = t_cur >= info->duration_ - 0.05;
        const bool timeout = escape_wall > info->duration_ + 2.0;
        if (!done && !timeout)
          return;

        const double moved = (odom_pos_ - escape_start_pos_).norm();
        escape_active_ = false;
        if (moved < 0.05)
        {
          ++escape_strategy_;
          RCLCPP_WARN(node_->get_logger(),
                      "[FSM] backup produced only %.2fm movement; next strategy %d/3",
                      moved, escape_strategy_);
        }
        else
        {
          escape_strategy_ = 0;
          RCLCPP_INFO(node_->get_logger(),
                      "[FSM] backup moved %.2fm; resume normal planning", moved);
        }
        have_new_target_ = true;
        changeFSMExecState(GEN_NEW_TRAJ, "ESCAPE_DONE");
        return;
      }

      // 统计执行进展（只看物理量：实际距离 + 实际速度）
      updateExecutionProgress();

      // 【航点模式】已到当前航点附近（0.5 米内）→ 前往下一个航点
      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        // 【V24】先规划成功再推进下标：否则规划失败也会前进，
        //   下一个航点被静默跳过，最终"成功到达"却漏走了若干航点。
        const int next_wp = current_wp_ + 1;
        current_wp_ = next_wp;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        // 规划失败 → 回退下标，留在本航点重试（计入失败次数）
        current_wp_ = next_wp - 1;
        if (!replanFailureIsBenign(status_reason_)) replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      // 【本段是否结束】
      //   非航点模式：物理到达(与状态话题同源) / 连续无进展 / 很宽的绝对上限（只防死循环）
      //   航点模式：轨迹时长到 或 同样适用无进展看门狗
      const bool time_done = t_cur > info->duration_ - 1e-2;
      // 【P16】exec_no_progress_ 还要求“当前这条轨迹已执行 ≥ stuck_window_”：
      //   刚换上一条新轨迹时不该被几秒前的旧判定直接判死。
      // 【P19b】门槛用“墙钟自提交以来的秒数”，不能用 t_cur：
      //   车卡住时 t_cur 被 P4 夹在 (t_proj + max_lead) ≈ 0.3 s，永远到不了 stuck_window(3 s)。
      const double wall_since_commit = (traj_commit_wall_.nanoseconds() == 0)
                                           ? 0.0
                                           : (node_->now() - traj_commit_wall_).seconds();
      const bool no_progress_valid = exec_no_progress_ && wall_since_commit >= stuck_window_;
      // 【V12】hard_cap 必须用墙钟：t_cur 被 P4 夹在 (t_proj + max_lead)，
      //   永远到不了 duration + finish_timeout_extra_，原判据是死代码。
      const bool hard_cap =
          wall_since_commit > info->duration_ + finish_timeout_extra_;
      // 【V11】航点模式也必须有看门狗：原实现只看 time_done（依赖被夹的 t_cur），
      //   卡住时永不触发，会永久停在 EXEC_TRAJ。
      const bool segment_done =
          status_is_stop_ ||
          no_progress_valid ||
          hard_cap ||
          (isWaypointSequenceMode() && time_done);
      if (segment_done)
      {
        RCLCPP_DEBUG(node_->get_logger(),
                    "[fsm] event=segment_end rid=%llu traj_id=%d "
                    "status_stop=%d no_progress=%d hard_cap=%d exec_no_progress=%d "
                    "wall=%.2fs t_cur=%.2f dur=%.2f best_dist=%.2f goal_dist=%.2f",
                    static_cast<unsigned long long>(replan_rid_), info->traj_id_,
                    static_cast<int>(status_is_stop_), static_cast<int>(no_progress_valid),
                    static_cast<int>(hard_cap), static_cast<int>(exec_no_progress_),
                    wall_since_commit, t_cur, info->duration_, exec_best_dist_,
                    (end_pt_ - odom_pos_).norm());
        // 复位执行进展统计
        exec_progress_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        exec_best_dist_ = 1e9;
        exec_best_tproj_ = 0.0;
        exec_no_progress_ = false;

        // 航点模式下"撞障碍/被卡住"不是到达：必须先重试/脱困，
        // 绝不能在卡住时推进航点下标（否则会静默跳过航点）。
        const bool waypoint_stuck =
            isWaypointSequenceMode() && !time_done && !status_is_stop_;

        // 航点正常走完：还有下一个航点 → 继续前往（先规划成功再推进）
        if (isWaypointSequenceMode() && !waypoint_stuck &&
            current_wp_ + 1 < (int)active_waypoints_.size())
        {
          const int next_wp = current_wp_ + 1;
          current_wp_ = next_wp;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          current_wp_ = next_wp - 1;
          if (!replanFailureIsBenign(status_reason_)) replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        // 航点模式走到最后一个航点 → 按原逻辑结束
        if (isWaypointSequenceMode() && !waypoint_stuck)
        {
          active_waypoints_.clear();
          current_wp_ = 0;
          abandonMission("WAYPOINTS_DONE");
          return;
        }

        // 【P21】非航点模式（以及"航点模式下的卡住"）：把“到达”和“卡住”分开处理。
        //   到达 → 正常结束；
        //   卡住/超时 → 有限重试，重试期内绝不清目标。
        if (status_is_stop_)
        {
          abandonMission("GOAL_REACHED");
          return;
        }

        const char *done_reason = no_progress_valid ? "no_progress" : (hard_cap ? "hard_cap" : "unknown");

        if (stuck_retry_count_ < max_stuck_retry_)
        {
          ++stuck_retry_count_;
          RCLCPP_WARN(node_->get_logger(),
                      "[FSM] stuck (%s): retry %d/%d traj_id=%d wall=%.2fs t_cur=%.2f dur=%.2f",
                      done_reason, stuck_retry_count_, max_stuck_retry_,
                      info->traj_id_, wall_since_commit, t_cur, info->duration_);
          // GEN_NEW_TRAJ 内部会重新从 odom 取起点状态
          changeFSMExecState(GEN_NEW_TRAJ, "STUCK_RETRY");
          return;
        }

        if (startBackupEscape(done_reason))
          return;

        // 重试 + 后撤都失败 → 放弃本段
        // 【V10】give-up 必须清 status_have_goal_（否则状态话题永远 running）
        abandonMission("STUCK_GIVEUP");
        return;
      }
      // 离终点已经很近 → 不必重规划，继续走完
      else if ((end_pt_ - odom_pos_).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      // 离这条轨迹的起点很近 → 说明刚重规划过，也先不重规划
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      // 其它情况 → 该重新规划了
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    // 状态 EMERGENCY_STOP：紧急停止。
    // 先把轨迹收缩成原地停住，然后等速度降下来再尝试恢复（如果开了 fail_safe）。
    // ---- 状态 EMERGENCY_STOP：紧急停止 ----
    case EMERGENCY_STOP:
    {
      if (reference_tf_stop_)
      {
        // 【V15】恢复前提是"TF 真的回来了"：
        //   用 monitorReferenceTransform() 本 tick 已查询并缓存的结果，
        //   **不要**在这里再调一次 lookupTransform —— 单线程执行器下
        //   重复查询会挤占 /tf 订阅回调的时间片，实测会让 TF 消息处理滞后，
        //   反而"永远等不到 TF 恢复"。
        const bool tf_ok = reference_tf_available_;

        // 注意：本分支**故意**不看 enable_fail_safe_，与下面普通急停分支不同。
        //   两者性质不一样：
        //     · 碰撞类急停 —— 障碍可能持续存在，自动恢复会导致"恢复→再撞"的
        //       震荡，所以必须由 enable_fail_safe_ 明确授权；
        //     · TF 停机   —— 是外部定位系统故障，故障解除即应恢复，
        //       此时继续停在原地没有安全收益，所以无条件恢复。

        // 坡道上速度可能长期降不到 0.1（定位微漂）→ TF 可用时不再等速度归零
        const bool tf_stop_timeout =
            reference_tf_stop_since_.nanoseconds() != 0 &&
            (node_->now() - reference_tf_stop_since_).seconds() > reference_tf_grace_;
        const bool stopped = odom_vel_.norm() < 0.1 || tf_stop_timeout;

        if (tf_ok && !reference_tf_jump_failed_ && stopped)
        {
          reference_tf_stop_ = false;
          reference_tf_stop_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
          reference_jump_replan_ = false;
          have_new_target_ = true;
          // 故障已解除：清除重试退避，立即恢复规划
          resetPlanRetry();
          changeFSMExecState(GEN_NEW_TRAJ, "TF_RECOVERED");
        }
        break;
      }

      // 刚进入急停：先把轨迹收缩成“停在原地”（只做一次）
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        // 【V6】fail_safe=false 也必须有出口：否则 EMERGENCY_STOP 会永久停住，
        //   只能重启节点。这里改成"失败太多 → 放弃任务"，其它情况仍需 fail_safe。
        if (need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          abandonMission("EMERGENCY_EXIT");
          return;
        }
        // 普通情况 → 回到 GEN_NEW_TRAJ 重新尝试规划（需要 fail_safe）
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      // 本轮处理完成，清掉“刚进入急停”的标志
      flag_escape_emergency_ = false;
      break;
    }
    }

    // 第三步：收尾（失败太多则强制急停）
    finishProcess();

    // 第四步：发布调试数据（带当前时间戳）
    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  // 每轮主循环结束后的收尾：如果连续规划失败次数太多，就强制进入紧急停止。
  // 收尾检查：如果连续失败次数太多，就强制急停并等待新目标
  // 当前段已经到达局部目标：按航点模式推进，或结束本段
  void SCANReplanFSM::completeCurrentSegment(const char *reason)
  {
    // 进入新一段：清除重试退避，第一次尝试立即进行
    resetPlanRetry();
    if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
    {
      // 【V24】先规划成功再推进下标
      const int next_wp = current_wp_ + 1;
      current_wp_ = next_wp;
      if (planNextWaypoint())
      {
        changeFSMExecState(GEN_NEW_TRAJ, "NEXT_WAYPOINT");
        return;
      }
      current_wp_ = next_wp - 1;
      if (!replanFailureIsBenign(status_reason_)) replan_fail_count_++;
      changeFSMExecState(GEN_NEW_TRAJ, "NEXT_WAYPOINT_FAIL");
      return;
    }

    // 【V10】本段结束统一走 abandonMission，保证 status_have_goal_ 一起清掉，
    //   状态话题不会永远停在 running。
    //
    // 但 CLOSE_TO_GOAL 的语义是"起点距**局部目标** <0.2 m"，在 mode1/mode3 下
    // 局部目标只是前方 horizon 处的截断点。若此时离任务真实终点还很远，
    // 说明局部目标退化了，不能当成任务完成——按一次失败处理，让重试/脱困接管。
    //
    // 例外：目标本身被判为"不可达（被障碍占据）"而主动截断了全局轨迹时，
    //   停在截断点就是**正常完成**（本来就去不了原目标），不能当失败无限重试。
    const bool close_to_goal_case = (std::string(reason) == "CLOSE_TO_GOAL");
    // 【问题1修正】这两类情形走到尽头就是**正常完成**，不能当成"局部目标退化"：
    //   - mission_goal_degraded_  ：目标已不可达，走到了最近的可行点；
    //   - mission_goal_truncated_ ：轨迹已沿原路截断到最后一个可达点。
    //   原写法 `!truncated || degraded` 会把 degraded 送进失败分支（注释与语义相反）。
    const bool goal_is_final = mission_goal_degraded_ || mission_goal_truncated_;
    if (close_to_goal_case && status_target_valid_ && !goal_is_final &&
        (mission_goal_pt_.head<2>() - odom_pos_.head<2>()).norm() > status_goal_tol_exit_)
    {
      ++replan_fail_count_;
      // 调用方在进来之前已按"到达"调过一次 notePlanResult(true)，把退避清零了。
      // 这里判定为退化失败，必须重新记一次失败，否则回 GEN_NEW_TRAJ 会以 10 ms
      // 一拍热循环重试同一个注定失败的局部目标（无退避）。
      notePlanResult(false);
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "[fsm] event=close_to_goal_degenerate rid=%llu dist=%.2fm",
                           static_cast<unsigned long long>(replan_rid_),
                           (mission_goal_pt_ - odom_pos_).norm());
      changeFSMExecState(GEN_NEW_TRAJ, "CLOSE_TO_GOAL_DEGENERATE");
      return;
    }
    if (close_to_goal_case && mission_goal_degraded_)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "[fsm] event=mission_done reason=goal_degraded rid=%llu dist=%.2fm",
                  static_cast<unsigned long long>(replan_rid_),
                  (mission_goal_pt_ - odom_pos_).norm());
    }
    else if (close_to_goal_case && mission_goal_truncated_)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "[fsm] event=mission_done reason=goal_truncated rid=%llu dist=%.2fm",
                  static_cast<unsigned long long>(replan_rid_),
                  (mission_goal_pt_ - odom_pos_).norm());
    }

    abandonMission(reason);
  }

  void SCANReplanFSM::finishProcess()
  {
    // 【连续失败兜底】"目标不可达 / 反复 optimize_failed"这类场景没有出路：
    //   脱困后撤能挪动一点、replan_fail_count_ 会被清零，于是会无限「退避重试→后撤」。
    //   consecutive_plan_fails_ 的清零条件要精确说清（很容易记错）：
    //     · 只在「执行有真实进展（更靠近目标）」或「收到新目标」时清零
    //       （见 updateExecutionProgress 的 goal_progressed 分支 / armPlannerStatus）；
    //     · **不**按"规划成功"清零 —— notePlanResult(true) 里明确跳过它，
    //       否则"规划出短轨迹 → 走一点 → 再失败"的循环会让计数永远攒不起来，
    //       这个兜底就永远不会触发。因此它能识别出上述死局。
    if (max_consecutive_plan_fail_ > 0 &&
        consecutive_plan_fails_ >= max_consecutive_plan_fail_ && have_target_)
    {
      const int fails = consecutive_plan_fails_;
      RCLCPP_ERROR(node_->get_logger(),
                   "Planning failed %d times in a row with no success in between; giving up task",
                   fails);
      // 顺序很重要：先立"已放弃"标记与失败原因，再 abandonMission。
      //   否则 abandonMission 会按"正常收尾"把 status_reasons_ 清掉、状态回落 idle。
      status_gave_up_ = true;
      setStatusReason(ReplanFailure::MAX_REPLAN_FAIL);
      abandonMission("CONSECUTIVE_FAIL_LIMIT");
      publishPlannerStatus("failed");
      return;
    }

    // 失败次数达到上限：先尝试低速后撤；后撤策略也用尽才急停
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      const int failed = replan_fail_count_;
      replan_fail_count_ = 0;
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; try backup escape before giving up", failed);

      if (have_target_ && startBackupEscape("planning_fail"))
        return;

      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed repeatedly; emergency stop and wait for a new target");
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      setStatusReason(ReplanFailure::MAX_REPLAN_FAIL);
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  // 从“当前轨迹”出发重新规划：
  // 起点用当前位置，速度/加速度从当前轨迹上取，然后重新规划到终点。
  // 这是最常见的重规划方式（比如检测到障碍时）。
  bool SCANReplanFSM::planFromCurrentTraj()
  {
    // 取出当前局部轨迹数据
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    // 这条轨迹已经执行了多少秒
    double t_cur = (time_now - info->start_time_).seconds();
    // 把时间夹在 [0, 总时长] 之内（防止负值或超界）
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    // 参考路径模式下，重规划起点取“当前轨迹上此刻的位置”，
    // 这样参考系变动时不会因为里程计跳变而抖
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      // 【问题3修正】mode3 的降级已统一放在 callReboundReplan() 入口处
      //   （在 getLocalTarget 之前，否则前视点被围时根本走不到降级）。
      //   这里不再重复调用。
      bool success = callReboundReplan(false, false);
      if (!success)
      {
        success = callReboundReplan(true, false);
        if (!success)
        {
          success = callReboundReplan(true, true);
          if (!success)
            return false;
        }
      }

      return true;
    }

    // 起点 = 机器人当前位置
    start_pt_ = odom_pos_;
    // 速度、加速度都从“当前轨迹”上取，保证重规划后轨迹平滑衔接
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    // 计算“从当前位置指向目标”的水平方向
    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    // 如果当前速度是朝反方向走的（背对目标）
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      // 就把速度、加速度清零，避免越走越远
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // 重新生成一条“从当前位置到终点”的全局参考轨迹
    // 若目标此前被判为"不可达、轨迹已截断"，则重建到那个可达点，
    // 避免每轮都把轨迹重新拉回真实目标（障碍处）→ 重复截断。
    const Eigen::Vector3d replan_end_pt =
        (mission_goal_truncated_ && have_truncated_reachable_pt_) ? truncated_reachable_pt_
                                                                 : end_pt_;
    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            replan_end_pt,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      // 失败 → 打错误日志并返回
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      setStatusReason(ReplanFailure::GLOBAL_TRAJ_FAILED);
      return false;
    }

    // 检查终点是否落在障碍里（若在，会沿轨迹回退找一个能站的点）
    if (!adjustGlobalTargetIfOccupied())
    {
      setStatusReason(ReplanFailure::TARGET_OCCUPIED);
      return false;
    }

    // 让规划总管做一次局部重规划（普通方式）
    bool success = callReboundReplan(true, false);
    // 第一次失败 → 再用“带随机点”的方式试一次（有助于跳出卡死）
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        // 两次都失败 → 放弃本轮
        return false;
    }

    // 规划成功
    return true;
  }

  // 决定“起点”用哪套数据：默认用里程计；
  // 如果当前已有轨迹且在时间范围内，就用轨迹上的速度/加速度，让衔接更平滑。
  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    // 默认方案：起点用里程计的位置和速度，加速度设为 0
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    // 如果当前没有正在执行的轨迹（还没开始过）→ 就用默认方案
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    // 当前时刻距离轨迹开始过了多久
    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    // 时间明显超出轨迹范围（或为负）→ 也用默认方案
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    // 把时间夹进有效范围
    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    // 用轨迹上的速度/加速度作为起点状态（比用里程计更平滑）
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    // 如果速度背对目标 → 清零，避免“往反方向起步”
    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  // ==========================================================================
  // checkCollisionCallback()：每 50 毫秒被调用一次，负责“安全检查”。
  // 做法：沿着当前局部轨迹往前走，每隔 0.01 秒取一个点，问地图“这里能站人吗”。
  //   只检查轨迹的前 2/3（因为后段本来就会重规划）。
  //   发现会撞：先尝试重规划；不行且撞上时间很近 → 紧急停止。
  // ==========================================================================
  void SCANReplanFSM::checkCollisionCallback()
  {
    // 【V21】不在这里再调 updateLocalTrajTimeFreeze()：
    //   它按"距上次调用"推进冻结/夹紧，两个定时器（10 ms / 50 ms）交替调用会让
    //   dt 的语义混乱（一半采样被顺延、一半没有）。时钟只由主循环统一维护。
    // 外部停止 / 未使能期间不做碰撞检查
    if (externally_stopped_ || !motion_enabled_)
      return;

    // 取出当前局部轨迹和地图
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // 参考 TF 已判定异常并急停过 → 不再做碰撞检查
    if (reference_tf_stop_)
      return;
    // 正在等待/急停、轨迹还没开始、或正在执行脱困后撤 → 不需要检查
    // 【V7】后撤轨迹是沿来路的直线，指向机器人刚走过的地方，形状上极易被判成"撞"。
    //   若不跳过，安全回调会 planFromCurrentTraj() 覆盖后撤轨迹，并让 escape_active_
    //   残留为 true → 新轨迹在 EXEC_TRAJ 里被当成后撤轨迹整段劫持。
    if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP ||
        escape_active_ || info->start_time_.seconds() < 1e-5)
      return;
    // 【V7】只有真正"在执行"的轨迹才需要安全监控：
    //   GEN_NEW_TRAJ / REPLAN_TRAJ 期间 local_data_ 里还是上一条旧轨迹，
    //   拿它做检查会绕过主循环的失败计数与 CLOSE_TO_GOAL 消费逻辑。
    if (exec_state_ != EXEC_TRAJ)
      return;

    /* ---------- check trajectory ---------- */
    // 采样步长：每 0.01 秒取一个点
    constexpr double time_step = 0.01;
    // 这条轨迹已经执行了多久
    double t_cur = (node_->now() - info->start_time_).seconds();
    // 只检查轨迹的前 2/3（后段本来就会重规划，不必查）
    double t_2_3 = info->duration_ * 2 / 3;
    // 从当前时刻往后，逐步前进检查
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      // 一旦越过 2/3 分界点，就停止检查
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;
      // P17a：跳过起点附近，避免把机器人自身膨胀层判成碰撞
      if (collision_skip_from_start_ > 0.0 && t - t_cur < collision_skip_from_start_)
        continue;

      // 该时刻的位置
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      // 再取后面一点点位置，用两点连线方向估计机器人的朝向
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      const double hit_yaw = estimateYawFromSegment(pos, pos_next);
      // 问地图：机器人以这个朝向待在这里，会不会撞？
      if (map->getInflateOccupancy(pos, hit_yaw))
      {
        const double body_pitch = map->getBodyPitch();
        const double body_roll = map->getBodyRoll();
        const double self_yaw = getOdomYaw();
        const double off = map->getDoubleCylinderOffset();
        const double cp = std::cos(body_pitch), sp = std::sin(body_pitch);
        const Eigen::Vector3d self_axis(cp * std::cos(self_yaw), cp * std::sin(self_yaw), sp);
        const Eigen::Vector3d hit_axis(cp * std::cos(hit_yaw), cp * std::sin(hit_yaw), sp);
        const Eigen::Vector3d self_front = odom_pos_ + off * self_axis;
        const Eigen::Vector3d self_rear = odom_pos_ - off * self_axis;
        const Eigen::Vector3d hit_front = pos + off * hit_axis;
        const Eigen::Vector3d hit_rear = pos - off * hit_axis;
        const int self_occ = map->getInflateOccupancy(odom_pos_, self_yaw);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                             "[collision] ahead=%.2fs pos=(%.2f,%.2f,%.2f) self_occ=%d "
                             "pitch=%.1fdeg roll=%.1fdeg yaw=%.1fdeg hit_yaw=%.1fdeg "
                             "self_front=(%.2f,%.2f,%.2f) self_rear=(%.2f,%.2f,%.2f) "
                             "hit_front=(%.2f,%.2f,%.2f) hit_rear=(%.2f,%.2f,%.2f) -> 先尝试重规划",
                             t - t_cur, pos(0), pos(1), pos(2), self_occ,
                             body_pitch * 180.0 / M_PI, body_roll * 180.0 / M_PI,
                             self_yaw * 180.0 / M_PI, hit_yaw * 180.0 / M_PI,
                             self_front(0), self_front(1), self_front(2),
                             self_rear(0), self_rear(1), self_rear(2),
                             hit_front(0), hit_front(1), hit_front(2),
                             hit_rear(0), hit_rear(1), hit_rear(2));
        // 参考 TF 缺失期间又检测到碰撞 → 直接急停（此时无法确认参考路径是否仍安全）
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
            reference_tf_missing_since_.nanoseconds() != 0)
        {
          reference_tf_stop_ = true;
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          setStatusReason(ReplanFailure::EMERGENCY_TF_MISSING_COLLISION);
          changeFSMExecState(EMERGENCY_STOP, "TF_MISSING_COLLISION");
          return;
        }
        // 撞了！先尝试“从当前轨迹出发重新规划”（也许还来得及绕开）
        if (planFromCurrentTraj()) // Make a chance
        {
          // 重规划成功 → 清失败计数（否则残留的计数会让状态一直报 error），回到执行状态
          replan_fail_count_ = 0;
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          // 重规划失败，而且离碰撞时刻很近 → 只能紧急停止
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            // 打警告：多少秒后就要撞上了
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            setStatusReason(ReplanFailure::EMERGENCY_SAFETY);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          // 还比较远 → 只是要求重新规划
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          // 无论哪种情况，本轮检查到此结束
          return;
        }
        break;
      }
    }
  }

  // 起点“脱困”：把落在膨胀层里的重规划起点推到最近的自由格。
  //   为什么需要它：机器人自身位置（脚边点云 / 贴身墙面）常常就落在自己的膨胀层内，
  //   而“轨迹时钟跟随实际进度”之后，重规划起点 ≈ 机器人自身位置，于是
  //     A* 起点调整会一路退出搜索池（INIT_ERR）、优化器会以
  //     “First three control points are in obstacles” 失败 → 机器狗原地不动、时钟冻结 → 死锁。
  //   做法：沿“起点 → 局部目标”方向逐格前推，最多 start_escape_max_offset_ 米。
  bool SCANReplanFSM::pushStartPtOutOfInflation()
  {
    auto map = planner_manager_->grid_map_;
    if (!map)
      return false;

    const double yaw = getOdomYaw();
    // 起点本来就在自由格 → 什么都不用做
    if (map->getInflateOccupancy(start_pt_, yaw) == 0)
      return false;

    const double step = map->getResolution();
    const int step_max = std::max(1, static_cast<int>(std::ceil(start_escape_max_offset_ / step)));

    // 【CLOSE_TO_GOAL 自伤修复】推进起点时不能把起点送到局部目标附近：
    //   getLocalTarget 用 min_target_dist(0.30~0.40 m) 校验过距离，
    //   但 reboundReplan 的早退门槛只有 0.20 m。若这里把起点朝目标推 0.3~0.4 m，
    //   推完之后距离掉到 0.2 m 以下，规划器就会把自己刚通过校验的目标判成"已到达"。
    //   因此：任何候选点都必须与局部目标保持 > kMinGoalClearance 的距离。
    const double kMinGoalClearance =
        std::max(0.25, 1.25 * planner_manager_->pp_.ctrl_pt_dist);
    const Eigen::Vector2d goal_xy = local_target_pt_.head<2>();
    auto keepsGoalDistance = [&](const Eigen::Vector3d &cand) {
      // 局部目标就是轨迹末端时不做这个限制：此时靠近是"合法到达"，
      // 与 targetFarEnough 的 t>=duration 豁免保持一致。
      if (local_target_is_traj_end_)
        return true;
      return (cand.head<2>() - goal_xy).norm() > kMinGoalClearance;
    };

    // 【V20】多方向脱困，与 A* 的起点脱困保持一致。
    //   原来只沿"起点→局部目标"单向推进：目标在身后时会把起点推得更深。
    //   顺序：朝局部目标 → 朝反方向 → 绕起点 8 方向。
    Eigen::Vector2d primary = local_target_pt_.head<2>() - start_pt_.head<2>();
    if (primary.norm() < 1e-3)
      primary = Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
    primary.normalize();

    auto try_direction = [&](const Eigen::Vector2d &dir) -> bool {
      for (int k = 1; k <= step_max; ++k)
      {
        Eigen::Vector3d cand = start_pt_;
        cand.head<2>() += dir * (step * k);
        if (map->getInflateOccupancy(cand, yaw) != 0)
          continue;
        // 推完之后不能贴到局部目标上，否则 reboundReplan 会判 CLOSE_TO_GOAL
        if (!keepsGoalDistance(cand))
          break;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[replan] start point was inside an inflated cell; pushed by %.2f m "
                             "(dir=%.2f,%.2f)",
                             step * k, dir(0), dir(1));
        start_pt_ = cand;
        return true;
      }
      return false;
    };

    if (try_direction(primary))
      return true;
    if (try_direction(-primary))
      return true;
    for (int a = 0; a < 8; ++a)
    {
      const double ang = a * M_PI / 4.0;
      const Eigen::Vector2d dir(std::cos(ang), std::sin(ang));
      if (try_direction(dir))
        return true;
    }

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[replan] no free start point within %.2f m; keep the original start",
                         start_escape_max_offset_);
    return false;
  }


  // 调用规划总管做一次规划（先取局部目标点，再交给总管）。
  // 成功后把算出的轨迹打包成 Bspline 消息发布出去。
  // flag_use_poly_init：是否用多项式作为初始猜测；
  // flag_randomPolyTraj：是否在初始猜测里插一个随机点（用来跳出卡死）。
  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // 每次重规划一个唯一 rid：A*/优化/可行性/失败汇总都用它串起来
    ++replan_rid_;
    if (planner_manager_) planner_manager_->setDiagContext(replan_rid_, diag_verbose_);

    auto log_replan_failure = [this](ReplanFailure reason) {
      const ReplanFailure r = (reason == ReplanFailure::NONE) ? ReplanFailure::UNKNOWN : reason;
      if (replanFailureIsBenign(r)) return;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[fsm] event=replan_failed rid=%llu traj_id=%d reason=%s replan_fail=%d/%d",
                           static_cast<unsigned long long>(replan_rid_),
                           planner_manager_ ? planner_manager_->local_data_.traj_id_ : -1,
                           replanFailureToString(r), replan_fail_count_, max_replan_fail_count_);
    };
    // 目标不可达时换成附近能站的点（必须在 getLocalTarget 之前：
    //   目标被围死时前视点往往也被围，getLocalTarget 会先失败返回）
    if (!mission_goal_degraded_ && goal_degrade_radius_ > 0.0 &&
        tryDegradeTarget(end_pt_) && mission_goal_degraded_)
    {
      // mode3 的终点由 monitorReferenceTransform 每 tick 从 reference_end_pt_ 重算，
      //   写回参考系后才不会被覆盖回原目标
      if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      {
        Eigen::Isometry3d ref_to_world;
        if (lookupReferenceTransform(ref_to_world))
          reference_end_pt_ = ref_to_world.inverse() * end_pt_;
      }
      // 全局轨迹的重建已在 tryDegradeTarget 内部完成（两个入口统一处理）
    }

    // 第一步：从全局轨迹上取“前方 planning_horizon 处”的点作为局部目标；
    // 参考路径模式下若拿不到参考系变换会返回 false，本轮规划直接放弃
    if (!getLocalTarget())
    {
      setStatusReason(last_target_failure_);
      log_replan_failure(last_target_failure_);
      return false;
    }

    // 第二步：起点脱困（方向取自"起点 → 局部目标"，所以必须在取到局部目标之后）。
    //   起点若落在膨胀层里，就推向最近的自由格，别让 A*/优化器从一个"本来就撞着"的点出发。
    //   注意：它对推进距离做了上限，保证推完之后仍满足 reboundReplan 的最小目标距离，
    //   否则会出现"自己把起点推到目标旁边 → 被判 CLOSE_TO_GOAL"的自伤。
    pushStartPtOutOfInflation();

    // 第二步：让规划总管真正规划一条局部轨迹
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    if (!plan_success)
    {
      const ReplanFailure reason = planner_manager_->lastFailureReason();
      setStatusReason((reason == ReplanFailure::NONE) ? ReplanFailure::UNKNOWN : reason);
      log_replan_failure(reason);
    }
    // “新目标”标志用完即清（它只影响第一次规划）
    have_new_target_ = false;

    // 【P16】新轨迹已生成 → 重置“无进展”看门狗的参考时刻与最近距离。
    //   否则：之前卡住几秒（重规划连续失败、车没动）之后一旦规划成功，第一次 tick 就会
    //   命中 exec_no_progress_，于是刚进 EXEC_TRAJ 就被强制结束。
    // 【P19b】记录这条轨迹的墙钟提交时刻，供 segment_done 的墙钟门槛使用。
    if (plan_success)
    {
      exec_progress_time_ = node_->now();
      // 与 updateExecutionProgress 使用同一把尺子：局部目标距离
      const Eigen::Vector3d goal0 = (local_target_pt_.head<2>().squaredNorm() > 1e-6)
                                        ? local_target_pt_
                                        : end_pt_;
      exec_best_dist_ = (goal0.head<2>() - odom_pos_.head<2>()).norm();
      exec_best_tproj_ = 0.0;
      exec_no_progress_ = false;
      traj_commit_wall_ = node_->now();
    }

    // 打印规划结果（ROS 日志，带 rid）
    const auto fail_reason = planner_manager_ ? planner_manager_->lastFailureReason() : ReplanFailure::NONE;
    RCLCPP_DEBUG(node_->get_logger(),
                "[diag] rid=%llu stage=fsm event=replan ok=%d reason=%s traj_id=%d",
                static_cast<unsigned long long>(replan_rid_), static_cast<int>(plan_success),
                replanFailureToString(fail_reason), planner_manager_ ? planner_manager_->local_data_.traj_id_ : -1);

    // ---- 【P9 诊断】每次重规划的几何量（只打印，不改行为）----
    //   看两个数：bearing=局部目标相对机身朝向的夹角（±180° = 目标在身后 → 控制器会原地调头）；
    //             d_target=局部目标距离（≈0 说明局部目标退化了 → 轨迹方向没有意义）。
    {
      const double yaw = getOdomYaw();
      const Eigen::Vector2d to_tgt = local_target_pt_.head<2>() - odom_pos_.head<2>();
      const double d_tgt = to_tgt.norm();
      double bearing_deg = 0.0;
      if (d_tgt > 1e-3)
      {
        const double raw = std::atan2(to_tgt(1), to_tgt(0)) - yaw;
        bearing_deg = std::atan2(std::sin(raw), std::cos(raw)) * 180.0 / M_PI;
      }
      Eigen::Vector2d tdir(0.0, 0.0);
      double traj_dur = 0.0;
      if (plan_success)
      {
        const auto &info = planner_manager_->local_data_;
        traj_dur = info.duration_;
        if (traj_dur > 1e-3)
        {
          const Eigen::Vector3d p0 = info.position_traj_.evaluateDeBoorT(0.0);
          const Eigen::Vector3d p1 = info.position_traj_.evaluateDeBoorT(std::min(0.1, traj_dur));
          tdir = (p1 - p0).head<2>();
          if (tdir.norm() > 1e-6)
            tdir.normalize();
        }
      }
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), diag_throttle_ms_,
                           "[replan-geom] rid=%llu ok=%d robot=(%.2f,%.2f,yaw=%.0f) goal=(%.2f,%.2f) d_goal=%.2f "
                           "start=(%.2f,%.2f) target=(%.2f,%.2f) d_target=%.2f bearing=%.0fdeg "
                           "traj_dir=(%.2f,%.2f) traj_dur=%.2f",
                           static_cast<unsigned long long>(replan_rid_), static_cast<int>(plan_success), odom_pos_(0), odom_pos_(1), yaw * 180.0 / M_PI,
                           end_pt_(0), end_pt_(1), (end_pt_ - odom_pos_).norm(),
                           start_pt_(0), start_pt_(1), local_target_pt_(0), local_target_pt_(1),
                           d_tgt, bearing_deg, tdir(0), tdir(1), traj_dur);
    }

    // ---- 成功：把轨迹打包成消息发布出去 ----
    if (plan_success)
    {
      if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      {
        last_reference_to_world_ = planning_reference_to_world_;
        have_last_reference_transform_ = true;
      }

      // 取出刚算好的局部轨迹
      auto info = &planner_manager_->local_data_;

      /* publish traj */
      // 构造要发送的 B 样条消息
      scan_planner_msgs::msg::Bspline bspline;
      // 阶数固定为 3（三次 B 样条，与规划器内部一致）
      bspline.order = 3;
      // 起始时刻 + 轨迹编号（控制器靠它们判断是不是新轨迹）
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      // 把控制点从矩阵搬进消息
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      // 逐个控制点复制坐标
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      // 再把节点向量（knots）搬进消息
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      // 发布轨迹 —— 控制器订阅 planning/bspline，会开始跟踪它
      if (motion_enabled_)
      {
        bspline_pub_->publish(bspline);
        publishLocalPath(info->position_traj_);
      }

      // 顺便在 RViz 里画出这条最优轨迹
      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  // 把当前局部轨迹按固定间隔采样成 Path 发到 local_path 话题（供外部观察 / 录制）
  void SCANReplanFSM::publishLocalPath(const UniformBspline &trajectory)
  {
    UniformBspline traj = trajectory;
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;

    const double duration = traj.getTimeSum();
    const double sample_dt = duration > 0.0 ? std::max(0.01, std::min(0.05, duration / 200.0)) : 0.01;

    for (double t = 0.0; t <= duration + 1e-6; t += sample_dt)
    {
      const auto pos = traj.evaluateDeBoorT(std::min(t, duration));
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.header.stamp = path.header.stamp;
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.position.z = pos.z();
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      path.poses.push_back(pose);
    }

    local_path_pub_->publish(path);
  }

  // 紧急停止：让总管生成一条“停在 stop_pos”的轨迹，并发布出去。
  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    // 让规划总管生成一条“停在原地”的轨迹
    planner_manager_->EmergencyStop(stop_pos);

    // 下面把这条静止轨迹打包成消息（流程与 callReboundReplan 完全一样）
    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    // 发布 —— 控制器收到后会立刻刹停
    bspline_pub_->publish(bspline);
    publishLocalPath(info->position_traj_);

    return true;
  }

  // 查询“参考坐标系 → world 系”的变换。
  // 参考系与 world 系相同时直接返回单位变换；TF 查不到、或数据太旧，都算失败。
  bool SCANReplanFSM::lookupReferenceTransform(Eigen::Isometry3d &reference_to_world)
  {
    reference_to_world.setIdentity();
    if (reference_frame_id_ == self_inflation_frame_id_)
      return true;
    if (reference_frame_id_.empty())
      return false;
    try
    {
      const auto transform = tf_buffer_->lookupTransform(
          self_inflation_frame_id_, reference_frame_id_, tf2::TimePointZero);
      // 变换的“新鲜度”：太旧的 TF 不可信，当成拿不到
      const double age = (node_->now() - rclcpp::Time(
          transform.header.stamp, node_->get_clock()->get_clock_type())).seconds();
      if (!referenceTransformFresh(age, reference_tf_max_age_))
        return false;
      reference_to_world = referenceToWorld(transform);
      return true;
    }
    catch (const tf2::TransformException &error)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Reference TF unavailable: %s", error.what());
      return false;
    }
  }

  // 监控参考坐标系：长时间拿不到就急停；若发生明显跳变（定位修正），
  // 就把当前轨迹作废并触发重规划。返回 true 表示参考系正常。
  bool SCANReplanFSM::monitorReferenceTransform()
  {
    // 每 tick 只查询一次，结果缓存给 EMERGENCY_STOP 复用（见 reference_tf_available_）。
    reference_tf_available_ = false;
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || !have_target_)
    {
      // 非 mode3 或没有目标：TF 与本状态机无关，视为"可用"以免误触发恢复判定
      reference_tf_available_ = true;
      return true;
    }

    Eigen::Isometry3d current;
    if (!lookupReferenceTransform(current))
    {
      // 记下“从什么时候开始拿不到”；超过宽限时间就急停
      const rclcpp::Time now = node_->now();
      if (reference_tf_missing_since_.nanoseconds() == 0)
        reference_tf_missing_since_ = now;
      if (referenceTfGraceExpired((now - reference_tf_missing_since_).seconds(),
                                  reference_tf_grace_) &&
          !reference_tf_stop_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Reference TF missing for %.2f s; stopping",
                     reference_tf_grace_);
        reference_tf_stop_ = true;
        // 记录进入 TF 急停的时刻：坡道上速度可能长期降不到 0.1，需要时间兜底
        if (reference_tf_stop_since_.nanoseconds() == 0)
          reference_tf_stop_since_ = now;
        callEmergencyStop(odom_pos_);
        flag_escape_emergency_ = false;
        setStatusReason(ReplanFailure::EMERGENCY_TF_TIMEOUT);
        changeFSMExecState(EMERGENCY_STOP, "TF_TIMEOUT");
      }
      return false;
    }

    // 拿到了 → 记下"本 tick TF 可用"，供 EMERGENCY_STOP 判定恢复
    reference_tf_available_ = true;
    // 清空“缺失起始时刻”，并按最新变换刷新终点
    reference_tf_missing_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    end_pt_ = current * reference_end_pt_;
    // 到达判定基准跟随参考系刷新（定位会在世界系里漂移，基准必须同步更新）
    mission_goal_pt_ = end_pt_;
    // 目标点已经能换算到 world 系，到达判定从这里开始才可信
    status_target_valid_ = true;
    // 执行轨迹期间，参考系若突然跳了一大步，旧轨迹的预测状态已不可信 → 重规划
    if (exec_state_ == EXEC_TRAJ && have_last_reference_transform_ &&
        referenceTransformJumped(last_reference_to_world_, current, odom_pos_,
                                 reference_tf_jump_translation_, reference_tf_jump_yaw_))
    {
      reference_jump_replan_ = true;
      changeFSMExecState(REPLAN_TRAJ, "TF_CHANGED");
    }
    return true;
  }

  // 取“局部目标点”：沿全局轨迹往前走大约 planning_horizon_ 米，
  // 把那个点作为这一轮规划的局部目标（而不是一路直奔最终终点）。
  // 若该点正好在障碍里，就前后找一个能站人的点代替。
  // navi_mode=3 时全局轨迹给在“参考坐标系”下，这里会先换算到 world 系；
  // 换算不成功（TF 不可用或过旧）则返回 false，本轮规划直接放弃。
  bool SCANReplanFSM::getLocalTarget()
  {
    last_target_failure_ = ReplanFailure::NONE;
    Eigen::Isometry3d reference_to_world = Eigen::Isometry3d::Identity();
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      if (!lookupReferenceTransform(reference_to_world))
      {
        last_target_failure_ = ReplanFailure::TF_UNAVAILABLE;
        return false;
      }
      planning_reference_to_world_ = reference_to_world;
      end_pt_ = reference_to_world * reference_end_pt_;
    }
    auto toWorld = [&](const Eigen::Vector3d &reference_point) {
      return reference_to_world * reference_point;
    };
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;

    // 【地图一致性】前视距离不能超过局部地图真正可信的范围。
    double horizon = planning_horizon_;
    if (planner_manager_ && planner_manager_->grid_map_)
    {
      const Eigen::Vector3d range = planner_manager_->grid_map_->getLocalUpdateRange();
      horizon = scan_planner::clampLocalHorizon(
          planning_horizon_, range(0), range(1), self_double_cylinder_radius_, 0.5);
    }
    else
    {
      horizon = std::max(0.5, horizon);
    }
    if (horizon + 1e-6 < planning_horizon_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "[getLocalTarget] planning_horizon %.2f m exceeds local map range; "
                           "clamped to %.2f m",
                           planning_horizon_, horizon);
    }

    // 采样步长：把规划视野长度分成 20 份
    double t_step = max_vel > 1e-6 ? horizon / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    // 纯函数：路径投影 + 前视目标 + 被占时的前后替代点。
    auto world_path = [&](double t) {
      return toWorld(planner_manager_->global_data_.getPosition(t));
    };
    const auto projection = scan_planner::projectOntoPath(
        world_path, duration, t_step, start_pt_);
    const double t_proj = projection.t_proj;

    // 【根治 CLOSE_TO_GOAL 退化】"走不满 horizon 时的兜底点"必须取
    // **全局轨迹的真实末端**，不能取 end_pt_：
    //   end_pt_ 是任务最终目标，可能远在轨迹之外（例如目标被占、轨迹被截断），
    //   拿它当兜底会让局部目标落到轨迹外、还可能正好在障碍里。
    const Eigen::Vector3d traj_end_pt = world_path(duration);
    const auto forward_target = scan_planner::findForwardTarget(
        world_path, duration, t_proj, horizon, t_step, traj_end_pt);
    double target_t = forward_target.t;
    const bool target_found = forward_target.found;
    local_target_pt_ = forward_target.point;
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    // 小工具：查询某个点是否被障碍占据（查询时带上朝向）
    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    // 替代点不能贴着机器人：否则 reboundReplan 会按 CLOSE_TO_GOAL 早退，
    // 形成“目标太近 → 不规划 → 目标仍太近”的自锁。
    // 全局终点本身即使很近也是合法到达，不受这个下限约束。
    const double min_target_dist =
        std::max(0.30, 2.0 * planner_manager_->pp_.ctrl_pt_dist);
    auto acceptableTarget = [&](const Eigen::Vector3d &pt, double t) {
      if (targetOccupancy(pt) != 0)
        return false;
      return scan_planner::targetFarEnough(
          pt, t, duration, start_pt_, min_target_dist);
    };

    // 如果局部目标恰好落在障碍里 → 前后各找一个“能站人”的点代替
    if (targetOccupancy(local_target_pt_) != 0)
    {
      const auto fallback = scan_planner::findFallbackTarget(
          world_path, duration, t_proj, target_t, t_step,
          acceptableTarget, traj_end_pt);
      if (fallback.found)
      {
        local_target_pt_ = fallback.point;
        target_t = fallback.t;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
        last_target_failure_ = ReplanFailure::NO_FREE_TARGET;
        return false;
      }
      // 局部目标只影响“这一轮往哪走”，不改全局路径与最终终点。
    }

    // 记录"本轮局部目标是否就是轨迹末端"：末端目标靠近机器人是合法到达，
    // 不受最小距离约束（与 targetFarEnough 的 t>=duration 豁免保持同一口径）。
    local_target_is_traj_end_ = (target_t >= duration - 1e-3);

    // 如果局部目标已经非常接近最终终点（进入刹车距离之内）
    const double braking_distance = max_acc > 1e-6
        ? (max_vel * max_vel) / (2.0 * max_acc)
        : std::numeric_limits<double>::infinity();
    if ((end_pt_ - local_target_pt_).norm() < braking_distance)
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      // 局部目标速度设为 0 —— 表示“到达后要停下”
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    // 否则
    else
    {
      // 局部目标速度 = 全局轨迹在该时刻的速度（保持匀速通过）
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      local_target_vel_ = reference_to_world.linear() * local_target_vel_;
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
    return true;
  }

} // namespace scan_planner
