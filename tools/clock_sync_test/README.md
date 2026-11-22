# G1/G2/G3 真机验证：录制 + 分析

你只负责操控机器人；脚本负责把**日志 + 关键话题**录下来，并自动生成
CSV / 图 / 摘要，供后面画分析图。

## 一次性准备

```bash
cd ~/SCAN-Planner
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scan_planner
source install/setup.bash
```

确认真机话题名（本分支的 remap）：

| 节点订阅名 | 真机实际话题 |
|---|---|
| `body_pose` | `/lio/robo/odom` |
| `sensor_pose` | `/lio/imu/odom` |
| `cloud` | `/lio/cloud_world` |

如果 odom 不是 `/lio/robo/odom`，用 `ODOM_TOPIC=/xxx/odom` 覆盖。

## 开始一次录制

```bash
bash tools/clock_sync_test/record_run.sh A_normal
```

它会：

1. 在 `~/clock_sync_runs/A_normal_<时间>/` 建目录；
2. 启动 `ros2 bag record`（带 QoS 覆盖）；
3. 启动规划栈（默认 `log_level:=debug`，可用 `--log-level` 覆盖），
   把终端输出和 `~/.ros/log` 分别写到 `run.log` 与 `roslog/`；
4. 5 秒后抓一次**生效参数**（`params_planner.yaml` / `params_controller.yaml`）；
5. 结束后自动调用 `analyze_run.py` 生成 `analysis/`，并生成
   `analysis/log_summary.txt`（`log_query summary` / `last-failure`）。

停止：**Ctrl-C**（或 `--duration 120` 自动停）。

## 建议的一次完整场景（一次 run 内做完）

| 顺序 | 操作 | 打标记（另开终端） |
|---|---|---|
| 1 | 正常巡航 60~90 s | `bash tools/clock_sync_test/record_run.sh --event "normal"` |
| 2 | 挡住车 2~5 s，然后松手 | `--event "block1"` / `--event "release1"` |
| 3 | 长时卡住 ≥5 s | `--event "block_long"` / `--event "release_long"` |
| 4 | 让它跑到目标收尾 | `--event "goal"` |

> 标记会写进 `events.tsv`；你可以按这些时间点去切段/画图。

## 产物

```
~/clock_sync_runs/<label>_<时间>/
├── meta.txt                  # 分支/HEAD/起止时间/话题
├── run.log                   # 终端输出（[traj-clock] / [ctl-clock] / FSM / rebo …）
├── bag/                      # rosbag2（tf、odom、bspline、cmd_vel、frozen、status …）
├── bag_record.log
├── events.tsv                # 你打的标记
├── params_planner.yaml       # 生效参数快照（核对 max_lead 等）
├── params_controller.yaml
└── analysis/
    ├── traj_clock.csv        # t, t_wall, t_proj, t_cur, lead, max_lead, dur, traj_id
    ├── ctl_clock.csv         # t, exec_time, t_proj, lead, dur, pos_err, traj_id
    ├── events.csv            # stuck / drop / 状态迁移 / 到达 …
    ├── cmd_vs_odom.csv       # 指令 vs 实测速度（长表）
    ├── flags.csv             # frozen / enabled
    ├── trajectories.csv      # 每条 B 样条（时长/点数）
    ├── status.csv            # /local_planner/status 状态序列
    ├── summary.txt           # 指标 + 判据
    └── plots/
        ├── traj_clock.png
        ├── ctl_clock.png
        ├── cmd_vs_odom.png
        └── traj_durations.png
```

## 手动重新分析

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash       # 解析 scan_planner_msgs/Bspline 需要
python3 tools/clock_sync_test/analyze_run.py ~/clock_sync_runs/<run_dir>

# 或直接分析最近一次 run
python3 tools/clock_sync_test/analyze_run.py
```

只分析日志、不看 bag：加 `--no-bag`。

失败链/原因分布也可以直接查：

```bash
python3 tools/log_query.py summary ~/clock_sync_runs/current/run.log
python3 tools/log_query.py last-failure ~/clock_sync_runs/current/run.log
```

## 怎么判断“生效”

| 项 | 看什么 | 期望 |
|---|---|---|
| G1 | `[traj-clock]` / `[ctl-clock]` | 有轨迹时约 1 Hz，字段齐全，带 `traj_id` |
| P3 | `ctl_clock.csv` 的 `lead` | 有界，p95 ≤ ~0.25 s，不单调增长 |
| P4 | `traj_clock.csv` 的 `lead` / `max_lead` | `max_lead=0.30`；`lead` p95 ≤ ~0.35 s；冻结期 `lead≈0.30` 正常 |
| P5 | `events.csv` 的 `robot_stuck` | 挡住 1.0~1.3 s 后出现；松开后恢复 |
| P2/P19a | `traj_timeout_drop` | 冻结期**不应**出现；杀掉规划器后约 `duration+3 s` 出现一次 |
| P16 | `fsm_transition` | `final_plan_success=1` 后 50 ms 内不应 `to WAIT_TARGET` |
| P19b | `fsm_transition` | 卡住 ≥3 s 后能离开 `EXEC_TRAJ`，不永远挂着 |
| P1 | `trajectories.csv` + `status.csv` | 到达后出现 1 条 **3.000 s** 轨迹；cmd 在 ≤0.3 s 内归零 |

## A/B（G4 未移植，不能用 param_overrides）

参数在 `init()`/构造函数里只读一次，**`ros2 param set` 无效**；要改就改 yaml 后重启，
不用重新编译（`install/` 是软链）。两种做法：

**方式 A（推荐，不污染默认）**
```bash
cp src/planner/plan_manage/config/robots/a2.yaml \
   src/planner/plan_manage/config/robots/a2_p3off.yaml
# 只改需要关的那一项（保留 robot_topics 段）
bash tools/clock_sync_test/record_run.sh B_noP3 -- \
  robot:=a2_p3off
```
> `record_run.sh` 会把 `--` 之后的参数原样传给 `ros2 launch`。

**方式 B（改完记得还原）**
```bash
# 改 a2.yaml / common.yaml → 跑 → git checkout --
```

| 补丁 | 文件 | 关值 |
|---|---|---|
| P3 | `common.yaml` | `time_lead: 99.0` |
| P2 | `common.yaml` | `traj_timeout_extra: 0.0` |
| P5 | `common.yaml` | `stuck_time: 1000000000.0` |
| P4 | `common.yaml` / `a2.yaml` | `fsm.traj_clock_max_lead: 0.0` |
| P1 | `common.yaml` / `a2.yaml` | `fsm.stop_traj_on_finish: false` |

> P16 / P19a / P19b 没有参数开关，只能 `git revert` 整组对照。

## 注意

- 本分支控制器/FSM 的 `/motion_enabled` 是 **VOLATILE** QoS；节点重启后必须由发布端再次发 `true`。
- `/body_pose` 在真机上不是真实话题；bag 里录的是 `/lio/robo/odom`。离线回放控制器时用
  `ros2 bag play <bag> --ros-args -r /lio/robo/odom:=body_pose`。
- `--with-map` 会显著增大 bag；默认不录 `/grid_map/*`。
- 如果机器人不在这台机器上：把整个 `~/clock_sync_runs/<run_dir>/` 拷回来，我可以直接分析。
