# go2w_description

Unitree Go2-W 的**官方 URDF + DAE 网格**，用于 RViz 显示和主仿真的机器人模型。

## 来源

- 仓库：<https://github.com/unitreerobotics/unitree_ros>
- 路径：`robots/go2w_description`
- commit：`157ff950351e5f535e03c5d85a16b7d594b3dbcc`（2026-11-13）
- 许可证：BSD（见官方 `package.xml` 的 `<license>BSD</license>`）

本包是 ROS 2 包装：只保留官方 `urdf/` 与 `dae/`，把 `CMakeLists.txt` / `package.xml`
改成 `ament_cmake`。**几何、关节、网格没有改动**，唯一改动是：

```xml
<!-- 官方：<robot name="go2w_description"> -->
<robot name="go2w">
```

这样 `robot:=go2w`、档案里的 `robot.name` 和 `sim_check` 的模型名检查保持一致。

## 模型内容

- 16 个可动关节 = **12 个腿关节 + 4 个轮关节**
  - 腿：`FL/FR/RL/RR_hip_joint`、`*_thigh_joint`、`*_calf_joint`（revolute）
  - 轮：`FL/FR/RL/RR_foot_joint`（continuous，轴 `0 1 0`，父 link 是 `*_calf`）
- 轮子网格：`dae/left_wheel.dae`、`dae/right_wheel.dae`
- 车身/腿网格：`dae/base.dae`、`hip.dae`、`thigh.dae`、`thigh_mirror.dae`、
  `calf.stl`、`calf_mirror.stl`
- 根 link：`base`
- 另有 `Head_upper/Head_lower`、`imu`、`radar`、`*_calflower*`、`*_foot_motor`
  等固定关节/link，属于官方模型的一部分。

## 怎么用

```bash
# 只看模型（通用入口）
ros2 launch scan_planner robot_view.launch.py robot:=go2w

# 主仿真（规划闭环）
ros2 launch scan_planner run.launch.py robot:=go2w is_real_world:=false
```

## 注意

- 官方 URDF **没有 `<gazebo>` 标签**，也没有 ROS 2 `ros2_control` / Fortress 插件。
  因此 `robot_gazebo.launch.py` 目前不支持 Go2-W；要做 Gazebo 物理仿真，需要另外
  增加 Gazebo 插件、轮子摩擦/接触参数和轮速控制器。
- 官方 URDF 的 `base` collision 仍是 `0.3762 × 0.0935 × 0.114`（和 Go2 相同）；
  规划器用的碰撞几何不读 URDF，而是 `config/robots/go2w.yaml` 的
  `double_cylinder_radius/offset` 等参数。
- 官方 `config/joint_names_go2w_description.yaml` 里有一个拼写错误
  `Fl_thigh_joint`，本包没有使用该文件；以 URDF 里的 `FL_thigh_joint` 为准。
