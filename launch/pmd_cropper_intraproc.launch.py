# Single-process composition: PMD driver + cropping_pointcloud loaded into the
# same ComposableNodeContainer with intra-process comms enabled.
#
# This is the ROS 2 equivalent of "two ROS 1 nodelets in one nodelet manager":
# the driver publishes a PointCloud2 unique_ptr, the cropper receives that same
# pointer. No serialization, no DDS round-trip. With a 224×172 ToF cloud at
# 30 Hz the savings are modest but real; for HD lidars it can be > 50 % of CPU.
#
# Requires an image with BOTH component libraries installed:
#   - libpmd_royale_ros_node.so  (from pmd_royale_ros_driver)
#   - libcropping_pointcloud_component.so  (this package, registers
#     `CroppingPointCloud`)
# See ~/ros2_ws/docker/pmd-cropper/intraproc.Dockerfile.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory("cropping_pointcloud")
    rviz_cfg = os.path.join(pkg_share, "rviz", "pmd_cropper.rviz")

    gui = LaunchConfiguration("gui")
    intra = {"use_intra_process_comms": True}

    container = ComposableNodeContainer(
        name="pmd_cropper_container",
        namespace="",
        package="rclcpp_components",
        # _mt = multi-threaded executor; lets the driver and the cropper run
        # their callbacks on separate threads while still sharing memory.
        executable="component_container_mt",
        composable_node_descriptions=[
            # --- TF static publisher: link → optical_frame ---
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                parameters=[{
                    "frame_id": "pmd_royale_ros_camera_node_link",
                    "child_frame_id": "pmd_royale_ros_camera_node_optical_frame",
                    "translation.x": 0.0, "translation.y": 0.0, "translation.z": 0.0,
                    "rotation.x":  0.5,  "rotation.y":  0.5,
                    "rotation.z": -0.5,  "rotation.w": -0.5,
                }],
                extra_arguments=[intra],
            ),
            # --- PMD ToF camera driver ---
            ComposableNode(
                package="pmd_royale_ros_driver",
                plugin="pmd_royale_ros_driver::CameraNode",
                name="pmd_royale_ros_camera_node",
                extra_arguments=[intra],
            ),
            # --- AABB crop + RANSAC plane removal ---
            # Subscribes to point_cloud_0 directly (no remap dance), publishes
            # /cropped_cloud, /object_cloud, and /crop_bounds. With intra-proc
            # on the upstream driver, the cloud arrives as a moved unique_ptr.
            ComposableNode(
                package="cropping_pointcloud",
                plugin="CroppingPointCloud",
                name="cropping_pointcloud",
                parameters=[{
                    "input_topic": "/pmd_royale_ros_camera_node/point_cloud_0",
                    "x_min": -0.15, "x_max": 0.15,
                    "y_min": -0.20, "y_max": 0.20,
                    "z_min":  0.15, "z_max":  0.60,
                    "extract_object": True,
                    "plane_distance_threshold": 0.01,
                    "plane_max_iterations": 200,
                }],
                extra_arguments=[intra],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Launch RViz2 + rqt alongside the components"),

        container,

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
