import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("cyberdog_web_bridge")
    cyclonedds_xml = os.path.join(pkg_dir, "config", "cyclonedds_bridge.xml")

    actions = [
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8090"),
        DeclareLaunchArgument("inspection_base_url", default_value="http://127.0.0.1:8083"),
        DeclareLaunchArgument("inspection_poll_sec", default_value="0.5"),
        DeclareLaunchArgument("ws_interval_sec", default_value="1.0"),
        DeclareLaunchArgument("cyberdog_ns", default_value="mi1035085"),
        DeclareLaunchArgument("workspace_dir", default_value="/opt/cyberdog/ros2_ws"),
        DeclareLaunchArgument("vision_ws_url", default_value="ws://127.0.0.1:9091/vision"),
        DeclareLaunchArgument("nav2_cyclonedds_xml",
            default_value="/opt/cyberdog/ros2_ws/src/cyberdog_nav2_lidar/cyclonedds.xml",
        ),
        SetEnvironmentVariable("CYCLONEDDS_URI", f"file://{cyclonedds_xml}"),
        Node(
            package="cyberdog_web_bridge",
            executable="bridge_node",
            name="cyberdog_web_bridge",
            output="screen",
            parameters=[
                {
                    "http_host": LaunchConfiguration("http_host"),
                    "http_port": LaunchConfiguration("http_port"),
                    "inspection_base_url": LaunchConfiguration("inspection_base_url"),
                    "inspection_poll_sec": LaunchConfiguration("inspection_poll_sec"),
                    "ws_interval_sec": LaunchConfiguration("ws_interval_sec"),
                    "cyberdog_ns": LaunchConfiguration("cyberdog_ns"),
                    "workspace_dir": LaunchConfiguration("workspace_dir"),
                    "nav2_cyclonedds_xml": LaunchConfiguration("nav2_cyclonedds_xml"),
                    "vision_ws_url": LaunchConfiguration("vision_ws_url"),
                }
            ],
        ),
    ]

    return LaunchDescription(actions)
