# 待办与已确认结论（断点记录）

> 更新：2026-11-11 16:2x　分支 `feat/progress-clock`　真机 HEAD `3b0327c`（= P17a + P19）

## 一、当前状态

| 项 | 值 |
|---|---|
| 分支 / 真机 HEAD | `feat/progress-clock` / **`3b0327c`** = **P17a + P19**（P17b、P19.1 已按用户要求删除，备份 tag `keep/p17b_p19`） |
| 已验证通过的阶段 | P1 结束后停车、P3 控制器时钟跟随（ctl_lead 硬顶 0.20）、P4 轨迹时钟夹住（traj_lead 硬顶 0.30）、P5 卡住刹停、P8 起点脱困、P14 起终同格修复 + 打转守卫 + 角速度 1.2、P15 A\* 二分重试 + 障碍法线兜底、P16 修"刚成功就 WAIT_TARGET"、**P17a 碰撞检查跳过当前位置 + self_occ 打点**、**P19 冻结时钟/墙钟超时死锁** |
| 最近一次好成绩（15:41–15:46） | 5 分钟、60 次重规划、**ok1 71%**、`First3=0`、**d_goal 19.2 → 1.0 m**、`trajectory older`=0、`empty initial_path`=0 |
| 诊断工具 | `tools/preflight_check.sh`（进程×1 / 门控 / 状态 / 冻结 / 路径点数+终点+与目标一致性 / 长度÷直线比 / 位姿是否被膨胀层包住）；真机另有 `~/ab/live_watch.py`（实时状态+日志跟踪记录器）与 `~/ab/live_watch.log` |

## 二、待办（按优先级）

### 1. 全局规划器加膨胀（根因，最重要）
- 位置：`~/nav_ws/src/global_planner/config/global_planner.yaml`（快照见 `robot_overrides/global_planner/global_planner.yaml`）
- 现状：**`robot_radius: 0.0`** ⇒ 全局路径按"质点"规划，可穿过局部地图判定为障碍的结构
- 改法：`robot_radius: 0.0 → 0.28`（局部 0.25 + 余量），**重新生成全局路径**
- 症状链（实测 15:41:26）：局部目标在墙另一侧 → 初始路径穿墙 → 控制点 `occ_in=1`/`occ_out=1` 成串 → A\* 二分到 0.07~0.24 m 仍失败 → 障碍法线硬推 → 优化失败 / **`dynamic_infeasible`** → 撞点 ahead 5.16→0.52 s ⇒ `EMERGENCY_SAFETY` 急停（行为正确，随后自动恢复）
- 验证判据：`occ_in=1` 比例下降；状态里不再出现 `dynamic_infeasible`；`[collision] ahead` 不再从 5 s 一路降到 0.5 s；ok1 再上一台阶

### 2. 碰撞检查"只查前 2/3"的结构盲区
- 三处：`bspline_optimizer.cpp:1359`、`:1484`、`scan_replan_fsm.cpp:1871`（`t_2_3 = duration*2/3`）；控制器侧无碰撞检查
- 后果：6 s 轨迹的最后 ~2 s（约 1~2 m）从未被校验，可穿障碍照发照开（"局部轨迹穿障碍还在跟"的直接原因）
- 改法：三处改为参数 `check_fraction`（默认 0.95）

### 3. 局部目标投影带 z + 单调（要按正确写法）
- 若重做：**先用"路径起点 z − 机器人 z"给出偏置**（让第一次搜索就有 z 门限），并设投影质量阈值（>1.0 m 就重做全范围、且该拍不锁单调）——不要只上"纯单调锁"，会锁死在折返的另一跑（P17b 的坑：`d_target` 只剩 1 m、`traj_dur` 1.7 s、目标在 16 m 外反方向）
- 备选：同层目标直接 `navi_mode:=1`（用局部地图自己规划，天然带膨胀）

## 三、已确认的事实（避免重复排查）

- **`/motion_enabled=false` 只让本栈静默**（规划器不规划、控制器不下发），**遥控器仍可直接驱动机器人** ⇒ "门控关着但 odom 在动（1.2 m/s、跑十几米、掉头）"是正常现象（15:42:18 后那段就是人工操控），**不要当成定位跳变或打滑**。
- **mode 3 的目标来自参考路径末端**：`end_pt_ = reference_to_world * reference_end_pt_`。若 `/global_path` 与目标不一致（实测 14:52：路径终点 (18.78,−11.00) vs 目标 (26.54,1.39)，差 13 m）或路径绕远（最长一次直线 ~7 m 给了 24.87 m），会表现为"局部目标在反方向、d_goal 单调增大"⇒ `tools/preflight_check.sh` 会直接报出来。
- **全局规划器多次发空路径**（`Received empty initial_path; ignoring`）：FSM 目前只是忽略并沿用旧路径（P18b 未实现）⇒ 会出现"继续按旧路线走"。
- 关键参数当前值：`fsm.planning_horizon 5.0`、`manager.a_star_pool_size 160`、`grid_map.self_filter_radius 0.45`、`fsm.collision_skip_from_start 0.25`、`fsm.start_escape_max_offset 1.0`、`fsm.spin_*`（180°/0.15 m/2 s/5 次）、`max_vyaw 1.2`（`kMaxVYawLimit` 也是 1.2，硬上限）、`fsm.status_goal_tol 0.20`（实测常停在 0.2~0.4 m 未判到达）。
- 真机人工改过、已快照进 `robot_overrides/`：`global_planner.yaml` 的 `path_z_offset: 0.7`；`map_localization.yaml` 的 `lio.sensor.blind: 0.3`。
- 诊断类日志一览：`[replan-geom]`（每次重规划的几何）、`[traj-clock]`（1 Hz 时钟）、`[ctl-clock]`（1 Hz 控制器）、`[A-star FAIL]`（含 ret/段长/两端 occ）、`[P15] split/synthesized`、`[collision] ahead/self_occ`、`[P17b] proj`（若该分支存在）、`[spin]`、`[Aster] 平面占用图① ② ③`（节流 5 s）。
- 陷阱：`[FSM]: state:`、`final_plan_success=`、`first_optimize_step_success=`、`Need to reallocate time` 等是 `cout`，**只进终端不进日志文件**；C++ 节点日志在未正常退出前可能只刷部分内容；`strings` 不显示中文。
