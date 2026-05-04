"""Launch the cropping_pointcloud node with the bundled YAML parameter file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory('cropping_pointcloud'))
    default_params = pkg_share / 'config' / 'cropping_pointcloud.yaml'

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(default_params),
        description='Path to the parameter YAML for cropping_pointcloud.',
    )

    node = Node(
        package='cropping_pointcloud',
        executable='cropping_pointcloud',
        name='cropping_pointcloud',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    return LaunchDescription([params_arg, node])
