from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    return [
        Node(
            package="cyberdog_ai_pkg",
            executable="vision_manager_node.py",
            name="vision_manager_node",
            output="screen",
            parameters=[
                {
                    "image_topic": LaunchConfiguration("image_topic").perform(context),
                    "body_topic": LaunchConfiguration("body_topic").perform(context),
                    "meter_topic": LaunchConfiguration("meter_topic").perform(context),
                    "detections_topic": LaunchConfiguration("detections_topic").perform(context),
                    "annotated_topic": LaunchConfiguration("annotated_topic").perform(context),
                    "annotated_ws_topic": LaunchConfiguration("annotated_ws_topic").perform(context),
                    "status_topic": LaunchConfiguration("status_topic").perform(context),
                    "extra_detection_topics": LaunchConfiguration(
                        "extra_detection_topics"
                    ).perform(context),
                    "available_detectors": LaunchConfiguration(
                        "available_detectors"
                    ).perform(context),
                    "enabled_detectors": LaunchConfiguration(
                        "enabled_detectors"
                    ).perform(context),
                    "annotated_ws_fps": LaunchConfiguration("annotated_ws_fps").perform(
                        context
                    ),
                    "alarm_ws_topic": LaunchConfiguration("alarm_ws_topic").perform(context),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/mi1035085/camera/color/image_raw",
                description="Input color image topic",
            ),
            DeclareLaunchArgument(
                "body_topic",
                default_value="/mi1035085/body",
                description="Body detection topic from dog-side detector",
            ),
            DeclareLaunchArgument(
                "meter_topic",
                default_value="/mi1035085/meter",
                description="Meter detection topic from dog-side detector",
            ),
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/vision/detections",
                description="Aggregated detections topic",
            ),
            DeclareLaunchArgument(
                "annotated_topic",
                default_value="/vision/annotated/compressed",
                description="Annotated image topic",
            ),
            DeclareLaunchArgument(
                "annotated_ws_topic",
                default_value="/vision/annotated_ws",
                description="Base64 annotated image topic for WS relay",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="/vision/status",
                description="Aggregated vision status topic",
            ),
            DeclareLaunchArgument(
                "extra_detection_topics",
                default_value="[]",
                description="Extra detector topic mappings like [fire_alarm=/mi1035085/fire_alarm]",
            ),
            DeclareLaunchArgument(
                "available_detectors",
                default_value="[person,meter]",
                description="Detector names exposed to the frontend",
            ),
            DeclareLaunchArgument(
                "enabled_detectors",
                default_value="[person]",
                description="Detectors enabled at startup",
            ),
            DeclareLaunchArgument(
                "annotated_ws_fps",
                default_value="5.0",
                description="Maximum WS annotated frame rate",
            ),
            DeclareLaunchArgument(
                "alarm_ws_topic",
                default_value="/vision/alarm_ws",
                description="Topic to publish alarm events for WS relay",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
