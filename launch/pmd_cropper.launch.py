# Launch the cropping_pointcloud node configured for the PMD flexx2 ToF camera.
# Subscribes to /pmd_royale_ros_camera_node/point_cloud_0, applies a tight
# AABB suited to a small object (~10 cm) sitting ~30 cm in front of the lens,
# then strips the dominant plane via RANSAC so the desk/floor disappears and
# only the object survives on /object_cloud.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("cropping_pointcloud")
    rviz_cfg = os.path.join(pkg_share, "rviz", "pmd_cropper.rviz")

    gui = LaunchConfiguration("gui")
    graph = LaunchConfiguration("graph")
    input_topic = LaunchConfiguration("input_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Launch RViz2 + rqt alongside the node (set to false for headless)"),
        DeclareLaunchArgument(
            "graph", default_value="true",
            description="Also launch rqt_graph to visualize node/topic connections"),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/pmd_royale_ros_camera_node/point_cloud_0",
            description="PointCloud2 topic to subscribe to"),

        Node(
            package="cropping_pointcloud",
            executable="cropping_pointcloud",
            name="cropping_pointcloud",
            output="screen",
            parameters=[{
                "input_topic": input_topic,
                # Optical-frame convention: +X right, +Y down, +Z forward.
                # Defaults assume a small object on a desk ~30 cm in front.
                "x_min": -0.15, "x_max": 0.15,
                "y_min": -0.20, "y_max": 0.20,
                "z_min":  0.15, "z_max":  0.60,
                "extract_object": True,
                "plane_distance_threshold": 0.01,
                "plane_max_iterations": 200,
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

        ExecuteProcess(
            cmd=["rqt_graph"],
            output="screen",
            condition=IfCondition(AndSubstitution(gui, graph)),
        ),
    ])
