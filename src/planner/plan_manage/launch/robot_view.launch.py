# ============================================================================
# 文件：launch/robot_view.launch.py —— 通用“只查看模型”入口
#
# 用法：
#   ros2 launch scan_planner robot_view.launch.py robot:=a2
#   ros2 launch scan_planner robot_view.launch.py robot:=go2w
#
# 启动：robot_state_publisher + joint_state_publisher_gui + rviz2
# 不启动物理仿真，也不启动规划器。
# ============================================================================
"""View any configured robot model in RViz without a physics simulator."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from robot_profile import as_bool, description_path, load_robot_profile, robot_description_command


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    robot = LaunchConfiguration("robot").perform(context).strip()
    if not robot or robot == "none":
        raise RuntimeError("robot_view requires robot:=a2|go2|go2w")

    profile = load_robot_profile(scan_share, robot)
    meta = profile.get("robot")
    if not meta:
        raise RuntimeError("robot profile '%s' is missing the 'robot' section" % robot)

    rviz_config = LaunchConfiguration("rviz_config").perform(context).strip()
    if not rviz_config:
        rviz_config = (profile.get("rviz", {}) or {}).get("config", "")
    if rviz_config and not os.path.isabs(rviz_config):
        if (profile.get("rviz", {}) or {}).get("config") == rviz_config:
            rviz_config = description_path(meta, rviz_config)
        else:
            rviz_config = os.path.join(scan_share, rviz_config)
    if not rviz_config:
        rviz_config = os.path.join(scan_share, "launch", "robot_view.rviz")

    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command(robot_description_command(meta)),
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="a2"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
