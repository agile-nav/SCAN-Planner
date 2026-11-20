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
# 三个机型都跑（a2 / go2 / go2w）
./tools/sim_check.sh

# 只跑一个
./tools/sim_check.sh go2w

# GPU 渲染后端（需先用 -DUSE_GPU=ON 构建）
USE_GPU=true ./tools/sim_check.sh
```

检查项：

1. `/robot_description` 的 `<robot name>` 与机型一致；
   A2/Go2 为 12 个腿关节，Go2-W 为 12 腿 + 4 轮（共 16）。
2. TF `world -> base` 存在。
3. 规划器收到 `/planning/bspline`、状态进入 `running`、机器人产生实际位移。
4. `/joint_states` 的腿/轮关节数量符合机型。
5. `/self_inflation` 前后双圆柱的半径/偏移/高度与机型档案一致。

日志：`log/sim_check/<机型>.log`。

注意：

- 运行时需要 `mockamap`、`local_sensing_node`、`pcl_render_node`
  （GPU 模式还需 `opengl_render_node` 和可用的 GL 3.3 上下文）。
- 脚本会 source 本工作区 `install/setup.bash`；如果依赖包在别的
  workspace，也会尝试 `$HOME/nav_ws/install/setup.bash`。
- 当前自检场景是 **3 m 平地短路 + 空地图**，用于验证“模型/关节/TF/规划闭环/
  碰撞几何”这条主链路，不覆盖长距离、障碍绕行、复杂地形和轮地打滑。

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
