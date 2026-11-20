# config/robots —— 机型档案

每个机型一个 `<机型>.yaml`，只写“与通用默认不同的项”。启动时叠加顺序：

```
config/common.yaml  ->  config/robots/<机型>.yaml  ->  命令行/launch 覆盖
```

## 现有档案

| 文件 | 说明 |
|---|---|
| `a2.yaml` | A2，实机碰撞口径 0.25 / 0.18 |
| `go2.yaml` | Go2，官方站立尺寸换算 |
| `go2w.yaml` | Go2-W，Unitree 官方 URDF（12 腿 + 4 轮） |
| `_template.yaml.example` | 新增机型模板，**不会**出现在 `robot:=` 可选项里 |

## 新增机型

```bash
cd src/planner/plan_manage/config/robots
cp _template.yaml.example myrobot.yaml
# 编辑 myrobot.yaml：robot / scan_planner_node / robot_topics 至少填齐
```

校验：

```bash
colcon test --packages-select scan_planner --ctest-args -R robot_profiles
ros2 launch scan_planner robot_view.launch.py robot:=myrobot
ros2 launch scan_planner run.launch.py robot:=myrobot is_real_world:=false
```

## 哪些字段是仿真用、哪些是真机用

| 字段 | 仿真 | 真机 | 说明 |
|---|---|---|---|
| `robot.description_package` / `model_file` / `model_args` | 需要 | 不需要 | 真机模型/TF 由宇树驱动/LIO 提供 |
| `scan_planner_node` 的几何/速度/视野/传感器参数 | 需要 | 需要 | 规划器与碰撞都依赖 |
| `quadruped_kinematic_sim` | 需要 | 不启动 | 仿真假机器人 |
| `quadruped_gait_publisher` | 可选 | 不启动 | 视觉步态/轮子转动 |
| `pcl_render_node` | 可选 | 不启动 | 仿真传感器安装位姿 |
| `robot_topics` | 不需要（仿真固定 `/quad_0/*`） | 需要 | 真机话题映射 |
| `gazebo` | 仅 Go2 | 不需要 | A2/Go2-W 按设计不做 Gazebo 仿真 |

## 最小必填项

- `robot.name` 必须与文件名一致；
- `robot.description_package` + `robot.model_file` 指向真实存在的模型；
- `fsm.planning_horizon`、`grid_map.double_cylinder_radius/offset`、
  `grid_map.body_height`、`grid_map.self_filter_radius`、
  `grid_map.obstacles_inflation_z_up/z_down`；
- 真机再加 `robot_topics` 的 `body_pose/sensor_pose/cloud/depth/cmd_vel/motion_enabled`。

没有实机数据时，不要把猜测值写成“已验证”；在注释里标 `TODO(实测)`。
