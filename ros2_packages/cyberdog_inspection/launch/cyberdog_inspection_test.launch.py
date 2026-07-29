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

    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="TF parent frame for the test pose publisher",
    )

    robot_frame_arg = DeclareLaunchArgument(
        "robot_frame",
        default_value="base_footprint_fixed",
        description="TF child frame for the test pose publisher",
    )

    inspection_node = Node(
        package="cyberdog_inspection",
        executable="cyberdog_inspection_node",
        name="cyberdog_inspection",
        output="screen",
        arguments=[LaunchConfiguration("config")],
    )

    tf_node = Node(
        package="cyberdog_inspection",
        executable="origin_tf_publisher",
        name="inspection_origin_tf",
        output="screen",
        parameters=[
            {
                "map_frame": LaunchConfiguration("map_frame"),
                "robot_frame": LaunchConfiguration("robot_frame"),
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
            }
        ],
    )

    mock_nav2_node = Node(
        package="cyberdog_inspection",
        executable="mock_nav2_server",
        name="inspection_mock_nav2",
        output="screen",
    )

    return LaunchDescription(
        [
            config_arg,
            map_frame_arg,
            robot_frame_arg,
            tf_node,
            mock_nav2_node,
            inspection_node,
        ]
    )
