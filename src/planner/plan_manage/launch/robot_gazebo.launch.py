# ============================================================================
# 文件：launch/robot_gazebo.launch.py —— 通用 Gazebo 物理仿真入口
#
# 用法：
#   ros2 launch scan_planner robot_gazebo.launch.py robot:=go2
#
# 只有机型档案里写了 gazebo: 段、且 description 包提供 world/bridge/
# controller 资产的机型才支持。A2 目前没有 Gazebo 资产，会明确报错；
# Go2-W 官方 URDF 也还没有 Gazebo 插件。
# ============================================================================
"""Run a configured robot in Gazebo Fortress with ros2_control."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from robot_profile import as_bool, load_robot_profile, robot_description_command


def _resolve(description_share, value):
    return value if os.path.isabs(value) else os.path.join(description_share, value)


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    robot = LaunchConfiguration("robot").perform(context).strip()
    profile = load_robot_profile(scan_share, robot)
    meta = profile.get("robot")
    gazebo = profile.get("gazebo")
    if not meta:
        raise RuntimeError("robot profile '%s' is missing the 'robot' section" % robot)
    if not gazebo:
        raise RuntimeError(
            "robot profile '%s' has no Gazebo assets (missing 'gazebo:' section). "
            "Use `ros2 launch scan_planner run.launch.py robot:=%s "
            "is_real_world:=false` for the kinematic simulation." % (robot, robot))

    description_share = get_package_share_directory(meta["description_package"])
    world = _resolve(description_share, gazebo["world"])
    bridge_config = gazebo.get("bridge_config", "")
    bridge_config = _resolve(description_share, bridge_config) if bridge_config else ""
    controllers = list(gazebo.get("controllers", [
        "joint_state_broadcaster", "joint_trajectory_controller"]))
    spawn_name = gazebo.get("spawn_name", robot)
    xacro_args = gazebo.get("xacro_args") or ["use_gazebo:=true"]
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))

    ros_gz_share = get_package_share_directory("ros_gz_sim")
    resource_root = os.path.dirname(description_share)
    robot_description = {
        "robot_description": Command(robot_description_command(meta, xacro_args)),
        "use_sim_time": use_sim_time,
    }

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", spawn_name,
            "-allow_renaming", "true",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
    )
    actions = [
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=[resource_root, os.pathsep,
                   EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[resource_root, os.pathsep,
                   EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
        ),
        gazebo_launch,
        state_publisher,
        spawn,
    ]
    if bridge_config:
        actions.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=spawn_name + "_gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        ))
    controller_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for name in controllers
    ]
    actions.append(RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=controller_nodes)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="go2"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.5"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
