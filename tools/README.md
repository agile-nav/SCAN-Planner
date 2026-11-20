# Tools

## keypoint_recorder

Build and source the workspace, then run:

```bash
ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle --output keypoints.yaml
```

The generated file is a ROS 2 parameter file containing the flat
`fsm.waypoints: [x0, y0, z0, ...]` array. Pass it to `run.launch.py` with
`keypoints_file:=/absolute/path/to/keypoints.yaml` when `navi_mode:=2`.

## sim_check.sh —— 三机型主仿真闭环自检

逐个机型拉起 `run.launch.py is_real_world:=false navi_mode:=3 robot:=<机型>`，
再用 `sim_check_probe.py` 做闭环检查。

```bash
# 三个机型 + empty_3m（默认）
./tools/sim_check.sh

# 只跑一个（兼容旧用法）
./tools/sim_check.sh go2w

# 10 m 直线空地图（验证 horizon / 长距离局部目标）
./tools/sim_check.sh --scenario straight_10m

# 指定场景和机型
./tools/sim_check.sh --scenario straight_10m a2
./tools/sim_check.sh --scenario straight_10m --robot go2w

# GPU 渲染后端（需先用 -DUSE_GPU=ON 构建）
USE_GPU=true ./tools/sim_check.sh
```

场景文件在 `tools/scenarios/<scenario>.yaml`，由 `tools/scenario_env.py` 转成
shell 参数。当前场景：

- `empty_3m`：3 m 平地短路 + 空地图，快速烟测。
- `straight_10m`：10 m 直线 + 空地图，抓 horizon/地图不匹配和长距离退化。
- `corridor_obstacle`：10 m 直线 + 固定 PCD 横墙，验证绕障；断言与障碍物的
  最小间隙 ≥ `min_clearance`，不允许“直接穿墙也算到达”。
- `narrow_passage`：固定横墙留 0.8 m 窄缝，验证 footprint/膨胀层贴边通过。
- `turning`：L 形路径 + 固定内墙，强制转弯；断言累计路径转向角 ≥ `min_yaw_change`。
- `multi_obstacle`：三个交错固定障碍，验证多次绕行/重规划。

场景 YAML 可配置：`path_file / navi_mode / obstacle_number / use_pcd_map /
pcd_map_file / timeout / min_move / goal_x / goal_y / goal_tol / goal_settle /
obstacle_bbox / obstacle_bboxes / min_clearance / min_yaw_change / map_size_*`。

- `obstacle_bboxes` 是包围盒列表，用于多障碍/窄通道；
- `min_yaw_change` 是累计路径转向角阈值（直线≈0，L 形≈π/2）。

到点后会再观察 `goal_settle` 秒（默认 1 s），避免“刚冲过阈值就退出”。
有 `obstacle_bbox` 时，实际间隙阈值取
`max(min_clearance, 机型圆柱半径 + 0.05 m)`，不会让比机器人半径还小的间隙通过。

检查项：

1. `/robot_description` 的 `<robot name>` 与机型一致；
   A2/Go2 为 12 个腿关节，Go2-W 为 12 腿 + 4 轮（共 16）。
2. TF `world -> base` 存在。
3. 规划器收到 `/planning/bspline`、状态进入 `running`、机器人产生实际位移；
   场景给了 `goal_x/goal_y` 时，还要求最终到终点距离 <= `goal_tol`；
   场景给了 `obstacle_bbox(es)` 时，还要求轨迹与障碍物的最小间隙 >= `min_clearance`；
   场景给了 `min_yaw_change` 时，还要求累计路径转向角达标。
4. `/joint_states` 的腿/轮关节数量符合机型。
5. `/self_inflation` 前后双圆柱的半径/偏移/高度与机型档案一致。

日志：`log/sim_check/<scenario>_<机型>.log`。

注意：

- 运行时需要 `mockamap`、`local_sensing_node`、`pcl_render_node`
  （GPU 模式还需 `opengl_render_node` 和可用的 GL 3.3 上下文）。
- 脚本会 source 本工作区 `install/setup.bash`；如果依赖包在别的
  workspace，也会尝试 `$HOME/nav_ws/install/setup.bash`。
- `empty_3m` 是快速烟测；`straight_10m`/`corridor_obstacle`/
  `narrow_passage`/`turning`/`multi_obstacle` 覆盖长距离、绕障、窄通道和转弯。
- 所有场景仍是**运动学仿真**：不验证轮地接触、打滑、腿支撑等动力学行为。

## 通用模型 / Gazebo launch

这两个入口在 `scan_planner` 包里（不在 `tools/`），但常和自检一起用：

```bash
# 只看模型：A2 / Go2 / Go2-W 通用，Go2-W 会显示 16 个关节
ros2 launch scan_planner robot_view.launch.py robot:=a2
ros2 launch scan_planner robot_view.launch.py robot:=go2w

# 独立 Gazebo 物理仿真：按机型档案的 gazebo: 段启用
ros2 launch scan_planner robot_gazebo.launch.py robot:=go2
```

支持情况：

| 入口 | A2 | Go2 | Go2-W |
|---|---|---|---|
| `robot_view.launch.py` | ✅ | ✅ | ✅（16 关节） |
| `robot_gazebo.launch.py` | ❌ 按设计不做 | ✅ | ❌ 按设计不做 |

`robot_gazebo` 对没有 `gazebo:` 段的机型会明确报错，不会静默启动空场景。
Gazebo 资产路径由机型档案提供：

```yaml
gazebo:
  world: worlds/empty.sdf
  bridge_config: config/bridge.yaml
  controllers: [joint_state_broadcaster, joint_trajectory_controller]
  spawn_name: go2
  xacro_args: ["use_gazebo:=true"]
```

## 可选 CTest

`sim_check` 默认不挂进 `colcon test`，避免拖慢日常测试。需要时可以显式开启：

```bash
colcon build --symlink-install --cmake-args -DENABLE_SIM_TESTS=ON
colcon test --ctest-args -L sim
```

## CI

`.github/workflows/ci.yml` **只在手动触发时运行**，不会因 push/PR 自动启动。
它做三件事：

1. `rosdep install` + `colcon build`；
2. `colcon test`；
3. `tools/ci_sim_check.sh` 跑 `empty_3m` 烟测。

手动触发（Actions → CI → Run workflow）还会跑完整回归：

```bash
./tools/ci_sim_check.sh --full
```

完整回归包括 `straight_10m / corridor_obstacle / narrow_passage / turning /
multi_obstacle`。仿真日志会作为 CI artifact 上传到 `log/sim_check/*.log`。
