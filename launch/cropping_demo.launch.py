# All-in-one launch for the "cropper unit" container: starts the cropping node,
# RViz2 with the bundled config, and rqt's Dynamic Reconfigure pinned to the
# node — so a single `ros2 launch …` brings the perception + GUI side up.
#
# For headless deployment (no display), launch with: gui:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("cropping_pointcloud")
    rviz_cfg = os.path.join(pkg_share, "rviz", "cropping_pointcloud.rviz")

    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Launch RViz2 + rqt alongside the node (set to false for headless)"),

        Node(
            package="cropping_pointcloud",
            executable="cropping_pointcloud",
            name="cropping_pointcloud",
            output="screen",
            parameters=[{
                "x_min": -2.0, "x_max": 8.0,
                "y_min": -5.0, "y_max": 5.0,
                "z_min": -1.0, "z_max": 2.0,
            }],
        ),

        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
            condition=IfCondition(gui),
        ),

        ExecuteProcess(
            cmd=["rqt", "--standalone", "rqt_reconfigure",
                 "--args", "/cropping_pointcloud"],
            output="screen",
            condition=IfCondition(gui),
        ),
    ])
