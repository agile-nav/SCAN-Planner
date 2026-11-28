// ============================================================================
// 文件：include/plan_manage/scan_replan_fsm.h
//
// 【这个文件是干什么的】
//   定义“状态机”类 SCANReplanFSM。状态机 = 程序的大脑：
//   它不停判断“我现在处于哪个阶段”，然后做那个阶段该做的事。
//
// 【六个阶段（状态）怎么流转】
//   INIT          启动后等待，直到收到“里程计”和“目标点”
//        ↓
//   WAIT_TARGET   等着用户/程序给一个目标点
//        ↓
//   GEN_NEW_TRAJ  第一次生成一条新轨迹
//        ↓
//   EXEC_TRAJ     正在沿轨迹走；每走一小段就检查要不要重新规划
//        ↓ ↑
//   REPLAN_TRAJ   重新规划（比如偏离了、前面有障碍了）
//        ↓
//   EMERGENCY_STOP 紧急停止（前面有障碍、来不及重规划时）
//
// 【两个定时器（心脏跳动）】
//   exec_timer_   每 10 毫秒跳一次 → execFSMCallback()，驱动状态机
//   safety_timer_ 每 50 毫秒跳一次 → checkCollisionCallback()，检查碰撞
//
// 【三种“目标怎么来”的模式 NAVI_MODE】
//   1 在 RViz 里用鼠标点一个目标点
//   2 用参数文件里预先写好的航点序列
//   3 订阅外部给的一条全局路径
//
// 【阅读提醒】
//   本文件只是“声明”，所有函数实现都在 src/scan_replan_fsm.cpp，
//   两边函数名一一对应。
// ============================================================================
#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <plan_env/replan_failure.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    // 状态机的六个状态（下面各自的中文含义见上）
    // enum = 枚举：把一组有名字的整数常量放在一起。
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      // 刚启动：等里程计和触发信号
      INIT,
      // 等待目标点
      WAIT_TARGET,
      // 生成一条全新的轨迹
      GEN_NEW_TRAJ,
      // 重新规划（在已有轨迹基础上重算）
      REPLAN_TRAJ,
      // 正在沿轨迹执行
      EXEC_TRAJ,
      // 紧急停止
      EMERGENCY_STOP
    };
    // 三种“目标来源”模式。= 1/2/3 与参数 fsm.navi_mode 对应。
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    // ---------- 规划用的工具对象 ----------
    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    // ---------- 参数（从 YAML 文件读进来） ----------
    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;
    std::string reference_frame_id_;
    double reference_tf_max_age_{0.3}, reference_tf_grace_{0.5};
    double reference_tf_jump_translation_{0.2}, reference_tf_jump_yaw_{5.0 * 3.14159265358979323846 / 180.0};

    // ---------- 运行状态(running/goal_reached)判定参数 ----------
    // 到达目标的 2D 水平距离容差(米)
    double status_goal_tol_{0.20};
    // 到达判定的退出阈值（滞回）：越小越容易掉出"到达候选"
    double status_goal_tol_exit_{0.30};
    // "速度为零"阈值(米/秒)
    double status_speed_tol_{0.05};
    // "已停"判定的退出阈值（滞回）
    double status_speed_tol_exit_{0.10};
    // ---- 执行进展(stuck)检测：只看物理进展，不看时间 ----
    // 无进展窗口(秒)：这么久没有"更靠近目标"就判为卡住
    double stuck_window_{3.0};
    // 窗口内距离至少要比历史最小值再小这么多才算"有进展"(米)
    double stuck_progress_eps_{0.05};
    // 绝对上限(秒)：t_cur > 轨迹时长 + 它 才强制收尾，只防死循环
    double finish_timeout_extra_{30.0};
    // 卡住后有限重试次数；重试期内不清目标
    int max_stuck_retry_{3};
    // 重试用尽后的脱困后撤距离 [m]
    double escape_distance_{0.35};
    // 轨迹时钟最多超前实际投影进度多少秒（<=0 关闭，P4）
    double traj_clock_max_lead_{0.3};
    // 重规划重试节流：同一状态下连续重试的最小间隔与退避上限(秒)；enable=false 则每拍都试
    bool replan_retry_enable_{true};
    double replan_retry_min_{0.2}, replan_retry_max_{1.0};
    // 脱困后撤的全局次数上限：即使每次都能挪动一点，也不允许无限「重试→后撤→重试」。
    // 有真实进展 / 新目标时清零。
    int max_escape_total_{6};
    // 【两个失败计数的分工 —— 服务于不同决策，不要合并】
    //   · replan_fail_count_     : 短周期，服务于"脱困后撤"。到
    //                              max_replan_fail_count_ 触发后撤，后撤成功即清零。
    //   · consecutive_plan_fails_: 长周期，服务于"放弃任务"。**不随后撤清零**，
    //                              只有 goal_progressed（执行有真实进展）或收到新
    //                              目标时才归零。
    //   为什么需要后者：目标不可达时"后撤能挪动一点"就会清掉前者，于是
    //   "失败→后撤→再失败"可以无限循环。后者只认"真的更靠近目标了"这一条证据，
    //   所以能兜住"反复 optimize_failed 且没有出路"的死局。
    //
    //   连续规划失败上限（0=不限制）：达到后停止重试、放弃任务并上报 failed。
    int max_consecutive_plan_fail_{30};
    // ---- 异常终止后自动请求全局重规划 ----
    // 非正常结束任务（卡死放弃/连续失败/急停退出）后，向全局规划器
    // 重发一次目标点，让它从当前位置重新规划一条到目标的路径。
    // 全局规划器（/goal_3d -> /global_path）会用自己订阅的实时里程计作起点，
    // 因此这里只需要给出目标点。
    bool global_replan_enable_{true};
    // 同一目标最多请求几次；用尽后不再请求，等人工介入
    int global_replan_max_requests_{3};
    // 两次请求之间的最小间隔(秒)
    double global_replan_min_interval_{1.0};
    // 发给 /goal_3d 的坐标系；global_planner 会自行转换到它的 map_frame
    std::string global_goal_frame_{"world"};
    // 当前目标已请求次数（新目标 / 真实前进时清零）
    int global_replan_requests_{0};
    rclcpp::Time last_global_replan_time_{0, 0, RCL_ROS_TIME};
    // 收尾时是否发一条“停在原地”轨迹让控制器刹停（P1）
    bool stop_traj_on_finish_{true};
    // 到达且停稳持续多久才判为 goal_reached(秒)
    double status_stop_debounce_{1.0};
    // 心跳频率(Hz)：周期性重发当前状态；0 表示关闭
    double status_heartbeat_hz_{1.0};
    // 判定"目标变了"的位置阈值(米)：新目标点与上一次相差超过它才算新目标
    double status_goal_change_eps_{0.05};

    // ---------- 运行时数据（程序自己维护的状态） ----------
    /* planning data */
    bool trigger_{false}, have_target_{false}, have_odom_{false}, have_new_target_{false};
    bool preset_started_{false};
    bool rviz_height_ready_{false};
    bool go2_execution_frozen_{false};
    // 外部使能（/motion_enabled，遥控器 START/B）：false 时冻结轨迹不规划不下发
    bool motion_enabled_{false};
    // 无外部使能时的初始值：真机默认 false（必须等 START）；仿真由 launch 置 true 才能跑起来
    bool motion_enabled_default_{false};
    // 重规划起点脱困：起点落在膨胀层里时，沿"起点→局部目标"方向最多前推这么多米
    double start_escape_max_offset_{1.0};
    // [replan-geom] 节流(ms)；0=不节流
    int diag_throttle_ms_{200};
    // P17a：碰撞检查跳过起点附近这段(秒)
    double collision_skip_from_start_{0.25};
    // 诊断：每次重规划关联号；diag_verbose 控制成功/逐迭代日志（默认关）
    uint64_t replan_rid_{0};
    bool diag_verbose_{false};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr diag_param_cb_;
    // ---------- 运行统计（health / 诊断话题 / 退出摘要） ----------
    uint64_t replan_total_{0}, replan_ok_{0};
    std::array<uint64_t, static_cast<size_t>(ReplanFailure::UNKNOWN) + 1> failure_counts_{};
    uint64_t degrade_count_{0}, backup_count_{0}, backup_short_count_{0};
    double plan_ms_last_{0.0}, plan_ms_sum_{0.0}, plan_ms_max_{0.0};
    uint64_t plan_ms_n_{0};
    std::deque<double> plan_ms_window_;
    rclcpp::Time start_wall_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_health_log_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_diag_pub_time_{0, 0, RCL_ROS_TIME};
    bool enable_fail_safe_{true}, need_hover_stop_{false};
    // 重规划分配失败（std::bad_alloc）后置位；下一次收尾统一转 EMERGENCY_STOP，
    // 避免异常继续向外传播导致节点退出。
    bool memory_error_latched_{false};
    FSM_EXEC_STATE exec_state_{INIT};
    int continuously_called_times_{0};
    // 短周期失败计数：服务于脱困后撤（分工说明见 max_consecutive_plan_fail_）
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_{0, 0, RCL_ROS_TIME};
    // 当前局部轨迹的墙钟提交时刻（P19b：无进展看门狗门槛）
    rclcpp::Time traj_commit_wall_{0, 0, RCL_ROS_TIME};
    // 轨迹投影进度的历史最好值（P21：无进展判据）
    double exec_best_tproj_{0.0};
    // 卡住重试计数（有实际进展/新目标时清零）
    int stuck_retry_count_{0};
    // 当前是否正在执行后撤轨迹
    bool escape_active_{false};
    // 后撤策略序号：0=0.35m, 1=0.21m, 2=0.49m
    int escape_strategy_{0};
    // 后撤总次数（不因单次有位移而清零）：防止「重试→后撤→重试」无限循环
    int escape_total_{0};
    // 后撤固定时长 [s]
    double escape_duration_{3.0};
    // 后撤起点（用于判断是否真的动了）
    Eigen::Vector3d escape_start_pos_{Eigen::Vector3d::Zero()};
    // 脱困后撤目标（沿实际走过路径回退得到）
    Eigen::Vector3d escape_target_pt_{Eigen::Vector3d::Zero()};
    // 最近实际走过的位姿历史（用于沿原路后撤）
    std::deque<Eigen::Vector3d> odom_history_;

    Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()}, odom_vel_{Eigen::Vector3d::Zero()},
        odom_acc_{Eigen::Vector3d::Zero()}; // odometry state
    Eigen::Quaterniond odom_orient_{Eigen::Quaterniond::Identity()};

    Eigen::Vector3d init_pt_{Eigen::Vector3d::Zero()}, start_pt_{Eigen::Vector3d::Zero()},
        start_vel_{Eigen::Vector3d::Zero()}, start_acc_{Eigen::Vector3d::Zero()},
        start_yaw_{Eigen::Vector3d::Zero()}; // start state
    Eigen::Vector3d end_pt_{Eigen::Vector3d::Zero()};                        // goal state
    Eigen::Vector3d end_vel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d local_target_pt_{Eigen::Vector3d::Zero()},
        local_target_vel_{Eigen::Vector3d::Zero()};                     // local target state
    // 本轮局部目标是否就是全局轨迹末端（此时"靠近目标"是合法到达，
    // 不受"替代点不能贴着机器人"的最小距离约束；见 getLocalTarget/targetFarEnough）
    bool local_target_is_traj_end_{false};
    Eigen::Vector3d reference_end_pt_{Eigen::Vector3d::Zero()};
    Eigen::Isometry3d planning_reference_to_world_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d last_reference_to_world_{Eigen::Isometry3d::Identity()};
    bool have_last_reference_transform_{false};
    bool reference_tf_stop_{false};
    // 本 tick 内"参考 TF 是否可用"（monitorReferenceTransform 统一查询并缓存）。
    //   规划器是单线程执行器：同一 tick 内重复 lookupTransform 会挤占 /tf
    //   订阅回调的时间片，导致 TF 消息处理滞后甚至丢失。
    bool reference_tf_available_{false};
    // 进入 TF 急停的时刻（用于"坡道微漂导致永远停着"的兜底恢复）
    rclcpp::Time reference_tf_stop_since_{0, 0, RCL_ROS_TIME};
    bool reference_tf_jump_failed_{false};
    bool reference_jump_replan_{false};
    rclcpp::Time reference_tf_missing_since_{0, 0, RCL_ROS_TIME};
    std::vector<Eigen::Vector3d> active_waypoints_;
    int current_wp_{0};

    // ---------- 运行状态发布(running/goal_reached) ----------
    // 是否收到过目标：没收到过属于 idle，不上话题
    bool status_have_goal_{false};
    // 当前目标点是否已换算到 world 系（mode3 需要等第一帧参考 TF）
    // false 时 updatePlannerStatus 不做到达判定，避免用陈旧/未初始化的 end_pt_ 误判“已到达”
    bool status_target_valid_{false};
    // 任务的**真实最终目标**（world 系）。到达判定只看它，不看 end_pt_：
    //   end_pt_ 会被 adjustGlobalTargetIfOccupied / 局部目标逻辑改写，
    //   用它判到达会在"目标点被占、截断到半路"时误报 goal_reached。
    Eigen::Vector3d mission_goal_pt_{Eigen::Vector3d::Zero()};
    // 目标点因被占据而被截断（到不了原目标）：用于上报 target_occupied
    bool mission_goal_truncated_{false};
    // 被截断后"实际可达的最后一个自由点"（world 系）。
    //   planFromCurrentTraj 重建全局轨迹时用它作终点，否则会把轨迹重新拉回
    //   真实目标（那里是障碍）→ 又截断一次 → 每轮重复"规划-截断"。
    Eigen::Vector3d truncated_reachable_pt_{Eigen::Vector3d::Zero()};
    bool have_truncated_reachable_pt_{false};
    // 目标不可达时的"降级落点"：由 findNearestReachableGoal 搜出的最近可达点。
    //   任务语义是"走到尽可能靠近目标的可行位置并正常结束"，而不是判失败。
    bool mission_goal_degraded_{false};
    // 降级搜索半径上限（米）：<=0 关闭降级，退回"沿轨迹截断"的旧行为。
    //   1.0 m 覆盖"目标落在最大障碍块中心"的最坏情况（膨胀后需外移约 0.65 m）。
    double goal_degrade_radius_{1.0};
    // 目标格空闲（occ==0）时，连续失败多少次才升级为「圆盘 + A*」完整验证。
    //   0 = 每次都验证（最严格，但对正常目标有白花开销）；
    //   默认 5：先按普通路径重试几次，仍失败才怀疑「站得住但去不了」。
    int degrade_verify_after_fails_{5};
    // 当前是否已判为 goal_reached（锁存，直到收到新目标）
    bool status_is_stop_{false};
    // 是否已有"上一次目标点"，用于重复路径去重
    bool status_goal_valid_{false};
    // 上一次用于判定的目标点（参考系/map）
    Eigen::Vector3d status_goal_ref_{Eigen::Vector3d::Zero()};
    // 到达判定期间锁存的目标点：防止 end_pt_ 随定位漂移导致判定基准变化
    Eigen::Vector3d status_goal_snapshot_{Eigen::Vector3d::Zero()};
    // "到达且停稳"起始时刻，用于去抖
    rclcpp::Time status_hold_since_{0, 0, RCL_ROS_TIME};
    // 上一次发布状态的时刻，用于心跳
    rclcpp::Time status_last_pub_time_{0, 0, RCL_ROS_TIME};
    // 持续失败/急停的起始时刻（用于 error 去抖）
    rclcpp::Time status_problem_since_{0, 0, RCL_ROS_TIME};
    // 执行进展：本轮见过的"到目标最小 2D 距离"、最后一次有进展的时刻、本拍是否无进展
    double exec_best_dist_{1e9};
    rclcpp::Time exec_progress_time_{0, 0, RCL_ROS_TIME};
    bool exec_no_progress_{false};
    // 上一次发布的状态（用于状态切换时立即上报）
    std::string status_last_state_;
    // 本次 error 期间出现过的所有失败原因（去重、按首次出现顺序）
    std::vector<ReplanFailure> status_reasons_;
    // 【进程内终态】状态不可信、必须重启节点才能恢复。
    //   只在"确定性证据"成立时置位：连最小分配（停车轨迹）都失败。
    //   置位后恒报 error，不再自动清除；节点重启后内存重建 ⇒ 自然回 idle。
    bool status_fatal_{false};
    // 置位时刻（供 diagnostics 输出持续时长）
    rclcpp::Time status_fatal_since_{0, 0, RCL_ROS_TIME};
    // 【单发事件】本任务已判定"失败终结"（彻底无解，不会再规划曲线）。
    //   由 abandonMission() 统一按 !normal_done 推断，判定后**立即发布**一帧
    //   failed（不等下一拍，避免同一拍内被新目标的 armPlannerStatus 复位而漏报），
    //   下一拍 status_have_goal_ 已为 false ⇒ 自然回落 idle。
    bool status_task_failed_{false};
    // 重试节流：上次真正尝试规划的时刻、连续失败次数（成功即清零）
    rclcpp::Time last_plan_attempt_{0, 0, RCL_ROS_TIME};
    int plan_retry_fails_{0};
    // 连续规划失败次数（跨脱困保留；只有成功/真实进展/新目标才清零）
    int consecutive_plan_fails_{0};
    // 正常 CLOSE_TO_GOAL 后等待到达判定：成立后才清目标并转 idle
    bool close_to_goal_waiting_{false};
    // 进入"等到达判定"的时刻：用于超时兜底，防止该分支永久提前 return 导致 FSM 停摆
    rclcpp::Time close_to_goal_wait_since_{0, 0, RCL_ROS_TIME};
    // 门控状态名（可配置；下游若需兼容旧的 stopped，可把 disabled 配成 stopped）
    std::string status_name_disabled_{"disabled"};
    std::string status_name_paused_{"paused"};

    // 最近的失败原因（供 error 的 reason 字段）
    ReplanFailure status_reason_{ReplanFailure::NONE};
    // getLocalTarget 失败的具体原因（no_free_target / tf_unavailable）
    ReplanFailure last_target_failure_{ReplanFailure::NONE};

    // 外部任务控制：/mission/control 是否处于 pause（停止）
    bool externally_stopped_{false};
    // 最近一条可用的全局路径（/global_path）；resume 时用它重建
    nav_msgs::msg::Path::ConstSharedPtr last_global_path_;

    // ---------- pause 期间的锁存（resume 时用来"继续原任务"） ----------
    // 锁存"本次任务实际要去的点"（world 系）。
    //   取值与到达判定同一口径：目标被截断时用 truncated_reachable_pt_，
    //   否则用 mission_goal_pt_（降级后它已被改写成落点）。
    //   resume 时若没有外部路径可用，就按它重新生成全局参考轨迹。
    Eigen::Vector3d resume_goal_pt_{Eigen::Vector3d::Zero()};
    bool have_resume_goal_{false};
    // 参考路径的 world 系快照（仅 mode3）。用于 resume 时参考 TF 已不可用的兜底：
    //   直接用它拟合全局轨迹，不再依赖 TF。参考系跳变时作废。
    //   若路径曾被截断，这里只保存到可达点的前缀（与 resume 目标同口径）。
    std::vector<Eigen::Vector3d> resume_waypoints_;
    // global_data_ 里存的轨迹是否是 world 系（由 resume_waypoints_ 拟合而来）。
    //   true 时 getLocalTarget / monitorReferenceTransform 跳过参考系变换，
    //   避免对已经是 world 系的轨迹再做一次 toWorld（双重变换）。
    bool global_frame_is_world_{false};

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    // TF 缓冲与监听器：用来查询“参考坐标系 → world”的变换（navi_mode=3 用）
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // 两个定时器：exec_timer_ 每 10ms 驱动状态机；safety_timer_ 每 50ms 查碰撞
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, self_inflation_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_enabled_sub_;
    // 任务控制：/mission/control（pause 停 / resume 继续）
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_control_sub_;
    // 异常终止后向全局规划器重发目标点（/goal_3d）
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_request_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    // 倾斜双圆柱 + 姿态轴线 + 角度文字（独立话题/命名空间）
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr body_pose_visual_pub_;
    // 运行状态话题：只发 running / goal_reached（idle 不发）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    // 诊断话题：计数器/失败分布/规划耗时，供 rqt/仪表盘直接消费
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;

    // ---------- 辅助函数（内部使用） ----------
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callReboundReplanImpl(bool flag_use_poly_init, bool flag_randomPolyTraj);
    void handleReplanMemoryError(double plan_ms);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();
    // 重试节流闸门：本拍是否允许再尝试一次规划
    bool planAttemptAllowed();
    void notePlanAttempt();
    void notePlanResult(bool ok);
    // 新目标/新航点/恢复时清除退避，保证第一次尝试不等待
    void resetPlanRetry();
    // 放弃当前任务（give-up / 急停退出）：清目标并复位状态话题
    void abandonMission(const char *reason);
    // 异常终止后请求全局规划器从当前位置重规划到原目标。
    // 返回 true 表示已发出请求。受开关/次数上限/最小间隔约束。
    bool requestGlobalReplan(const char *reason);
    // 全局轨迹是否可用（navi_mode=3 下 resume 前必须为真）
    bool hasUsableGlobalTraj() const;
    // 把实测位置投影到当前局部轨迹，返回对应轨迹时刻（P0：仅诊断用）
    double projectOdomToTrajParam(double t_hint) const;

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    // 切换状态（例如从 GEN_NEW_TRAJ 切到 EXEC_TRAJ），并打印日志
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    // 【目标降级】目标不可达时在 goal_degrade_radius_ 内找最近可行点并改写
    //   end_pt_/mission_goal_pt_。true=已处理，false=没找到可用点。
    //   mode1/mode2 与 mode3 共用（后者需再把结果写回 reference_end_pt_）。
    // 重置"任务级目标状态"（截断/降级标记与可达点缓存）。
    //   三个目标入口在确认"这是新目标"后、且必须在
    //   adjustGlobalTargetIfOccupied 之前调用。
    void resetMissionGoalState();
    bool tryDegradeTarget(const Eigen::Vector3d &goal_pt);
    // 从全局轨迹上取“前方 planning_horizon 米处”的点，作为局部目标
    bool getLocalTarget();
    // 查询“参考坐标系 → world”的变换；查不到（超时等）时返回 false
    bool lookupReferenceTransform(Eigen::Isometry3d &reference_to_world);
    // 取“本次规划实际要用的”参考系变换：global_data_ 已是 world 系时返回单位阵。
    //   ok=false 表示参考 TF 不可用（仅 mode3 且非 world 快照时会失败）。
    Eigen::Isometry3d effectiveReferenceTransform(bool &ok);
    // 监控参考 TF 的新鲜度与跳变；返回 true 表示当前应停止推进
    bool monitorReferenceTransform();
    void finishProcess();
    // 发布机器人自身外形（前后两个圆柱），在 RViz 里看得到
    void publishSelfInflationMarker();
    // 发布倾斜双圆柱 / 姿态轴线 / 角度文字（仅观测，不参与控制）
    void publishBodyPoseVisualization();
    void publishLocalPath(const UniformBspline &trajectory);
    double getOdomYaw() const;
    // 重规划起点若落在膨胀层里，沿"起点→局部目标"方向前推到最近的自由格
    bool pushStartPtOutOfInflation();
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    // 收到新目标时调用：进入 running 并清掉 goal_reached/去抖状态
    void armPlannerStatus();
    // 每个 tick 评估是否"到达目标且停稳"，并用心跳周期重发
    void updatePlannerStatus();
    void updateExecutionProgress();
    void motionEnabledCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
    // 沿实际走过路径回退 distance 米；used 返回实际回退距离
    bool getBackupPointAlongHistory(double distance, Eigen::Vector3d &pt, double &used) const;
    // 直接生成并发布一条低速后撤轨迹；成功返回 true
    bool startBackupEscape(const char *reason);
    // 当前段已到达局部目标：推进航点或结束本段
    void completeCurrentSegment(const char *reason);
    // 把 local_data_ 里的当前轨迹打包发布
    void publishCurrentTrajectory();
    // 向 /local_planner/status 发布状态（空串不发布）
    void publishPlannerStatus(const std::string &state);
    // 把门控（未使能/暂停/error 锁存）施加到状态名上。
    //   发布与"是否算状态切换"两处必须用同一个函数，否则会出现
    //   "比较用原始值、存储用门控值"导致每拍都判定为切换并重复发布。
    std::string gateStatusName(const std::string &state) const;
    // status 话题专用门控：不再用 disabled 覆盖任务状态，仅保留 error 锁存
    std::string gateStatusNameForStatus(const std::string &state) const;
    // 把状态拼成 JSON（state / msgs）
    std::string buildStatusJson(const std::string &state);
    // 运行统计与诊断
    void recordReplanResult(bool ok, ReplanFailure reason, double ms);
    void updateHealthSummary();
    std::string buildDiagnosticsJson();
    std::string buildFailureCountsJson() const;
    void logRunSummary(const char *event);
    // “如果没有门控，本该报什么状态”（只读、无副作用），供 JSON 的 inner 字段使用
    std::string statusInnerState() const;
    // 记录一次失败原因（设为当前原因，并累积到本次 error 的原因列表）
    void setStatusReason(ReplanFailure r);
    // 任务控制回调：pause / resume
    void missionControlCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
    // 外部停止：清全局+局部轨迹，锁存本任务目标，状态置 stopped
    void applyExternalStop();
    // 外部继续：按四级降级重建（TF 可用 → Path；否则 world 快照 → 目标）
    void applyExternalResume();
    // 用 world 系航点直接拟合全局轨迹（resume 第 2 级，不依赖参考 TF）
    bool planGlobalFromWorldWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    // 用 world 系目标重建全局轨迹（resume 第 3 级，mode1/2 主路径）
    bool planGlobalFromWorldGoal(const Eigen::Vector3d &goal_pt);
    // 进入进程内终态 error（只在确定性证据成立时调用）
    void enterFatalError(const char *reason);

    // ---------- ROS 回调函数（收到消息/定时器到点时被自动调用） ----------
    /* ROS functions */
    // 【核心】10ms 定时器回调：状态机的主循环
    void execFSMCallback();
    // 50ms 定时器回调：检查轨迹前段是否会撞到障碍
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    // is_resume=true 表示"同一个任务继续"（pause 后的重建），不重置任务级目标状态
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg, bool is_resume = false);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM();

    // 唯一对外入口：做全部初始化（读参数、订阅话题、建定时器）
    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
