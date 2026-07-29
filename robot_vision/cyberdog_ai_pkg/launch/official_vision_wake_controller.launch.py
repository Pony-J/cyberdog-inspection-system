from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    wake_delay_sec = LaunchConfiguration("wake_delay_sec")
    warmup_wait_sec = LaunchConfiguration("warmup_wait_sec")
    sleep_on_shutdown = LaunchConfiguration("sleep_on_shutdown")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace",
                default_value="/mi1035085",
                description="CyberDog robot namespace prefix",
            ),
            DeclareLaunchArgument(
                "wake_delay_sec",
                default_value="1.0",
                description="Seconds to wait before sending MODE_TRACK",
            ),
            DeclareLaunchArgument(
                "warmup_wait_sec",
                default_value="5.0",
                description="Seconds to wait for the vision stack warmup",
            ),
            DeclareLaunchArgument(
                "sleep_on_shutdown",
                default_value="true",
                description="Whether to switch back to manual mode on shutdown",
            ),
            Node(
                package="cyberdog_ai_pkg",
                executable="official_vision_wake_controller.py",
                name="official_vision_wake_controller",
                output="screen",
                parameters=[
                    {
                        "robot_namespace": robot_namespace,
                        "wake_delay_sec": wake_delay_sec,
                        "warmup_wait_sec": warmup_wait_sec,
                        "sleep_on_shutdown": sleep_on_shutdown,
                    }
                ],
            ),
        ]
    )
