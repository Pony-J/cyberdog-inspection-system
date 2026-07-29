from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("cyberdog_inspection")
    default_config = os.path.join(pkg_share, "config", "inspection_config.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to cyberdog inspection YAML config",
    )

    node = Node(
        package="cyberdog_inspection",
        executable="cyberdog_inspection_node",
        name="cyberdog_inspection",
        output="screen",
        arguments=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, node])
