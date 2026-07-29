from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Keep this launch focused on the WS relay itself.
            # For the full dog-side stack (camera enable + detectors +
            # vision_manager + gateway), use the helper starter script.
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/vision/detections",
                description="Aggregated detections topic forwarded over WS",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="/vision/status",
                description="Aggregated vision status topic forwarded over WS",
            ),
            DeclareLaunchArgument(
                "runtime_status_topic",
                default_value="/vision/runtime_status",
                description="Runtime detector status topic forwarded over WS",
            ),
            DeclareLaunchArgument(
                "annotated_ws_topic",
                default_value="/vision/annotated_ws",
                description="Base64 annotated image topic for WS relay",
            ),
            DeclareLaunchArgument(
                "ws_host",
                default_value="0.0.0.0",
                description="WebSocket bind host",
            ),
            DeclareLaunchArgument(
                "ws_port",
                default_value="9091",
                description="WebSocket bind port",
            ),
            DeclareLaunchArgument(
                "ws_path",
                default_value="/vision",
                description="WebSocket URL path",
            ),
            DeclareLaunchArgument(
                "include_annotated",
                default_value="true",
                description="Whether to push base64 annotated JPEG frames over WS",
            ),
            Node(
                package="cyberdog_ai_pkg",
                executable="vision_ws_gateway.py",
                name="vision_ws_gateway",
                output="screen",
                parameters=[
                    {
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "runtime_status_topic": LaunchConfiguration("runtime_status_topic"),
                        "annotated_ws_topic": LaunchConfiguration("annotated_ws_topic"),
                        "ws_host": LaunchConfiguration("ws_host"),
                        "ws_port": LaunchConfiguration("ws_port"),
                        "ws_path": LaunchConfiguration("ws_path"),
                        "include_annotated": LaunchConfiguration("include_annotated"),
                    }
                ],
            ),
        ]
    )
