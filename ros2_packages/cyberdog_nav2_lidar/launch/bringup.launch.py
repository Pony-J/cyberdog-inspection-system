"""
CyberDog Nav2 + Cartographer bringup.

Modes (via 'mode' argument):
  slam   — Cartographer 建图 + Nav2 导航（边建图边导航）
  nav    — Cartographer 纯定位 + Nav2 导航（需要 pbstream 地图）
  amcl   — 传统 AMCL 定位 + Nav2 导航（向后兼容）

TF chain:
  map → odom_fixed → base_footprint_fixed → base_link_fixed → laser_frame_fixed

Usage:
  # 建图模式
  ros2 launch cyberdog_nav2_lidar bringup.launch.py mode:=slam

  # Cartographer 定位模式
  ros2 launch cyberdog_nav2_lidar bringup.launch.py mode:=nav \
      pbstream_file:=/path/to/map.pbstream

  # AMCL 定位模式（向后兼容）
  ros2 launch cyberdog_nav2_lidar bringup.launch.py mode:=amcl

  # 建图完成后保存地图：
  ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
      "{filename: '/path/to/cyberdog_map.pbstream', include_unfinished_submaps: true}"
  # 可选：导出 pgm/yaml
  ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
      -pbstream_filename=/path/to/cyberdog_map.pbstream \
      -map_filestem=/path/to/cyberdog_map -resolution=0.05
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


CYBERDOG_NS = 'mi1035085'


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    map_yaml = LaunchConfiguration('map_yaml')
    pbstream_file = LaunchConfiguration('pbstream_file').perform(context)

    bringup_dir = get_package_share_directory('cyberdog_nav2_lidar')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    param_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    lidar_params = os.path.join(ydlidar_dir, 'params', 'ydlidar.yaml')
    config_dir = os.path.join(bringup_dir, 'config')

    default_bt_xml = os.path.join(
        bringup_dir, 'config', 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    )
    default_bt_through_xml = os.path.join(
        bringup_dir, 'config', 'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml',
    )

    # --- Nav2 参数 -----------------------------------------------------------
    param_rewrites = {
        'default_nav_to_pose_bt_xml': default_bt_xml,
        'default_nav_through_poses_bt_xml': default_bt_through_xml,
    }
    if mode == 'slam':
        param_rewrites['map_subscribe_transient_local'] = 'false'

    configured_params = RewrittenYaml(
        source_file=param_file,
        root_key='',
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    # ── 公共节点 ─────────────────────────────────────────────────────────────

    common_nodes = [
        # 静态 TF: base_footprint_fixed → base_link_fixed
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_joint_fixed',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_footprint_fixed',
                       '--child-frame-id', 'base_link_fixed'],
        ),
        # 静态 TF: base_link_fixed → laser_frame_fixed
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_joint_fixed',
            arguments=['--x', '-0.10', '--y', '0.0', '--z', '0.25',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link_fixed',
                       '--child-frame-id', 'laser_frame_fixed'],
        ),
        # 静态 TF: base_link_fixed → imu_fixed
        # 相机安装位置 + ROS body→optical 旋转 (Z前X右Y下)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_joint_fixed',
            arguments=['--x', '0.3252', '--y', '0.0475', '--z', '-0.0795',
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                       '--frame-id', 'base_link_fixed',
                       '--child-frame-id', 'imu_fixed'],
        ),
        # TG30 激光雷达驱动
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[lidar_params],
        ),
        # 时间戳修正（odom + scan frame 隔离）
        Node(
            package='cyberdog_nav2_lidar',
            executable='fix_cyberdog_timestamps.py',
            name='timestamp_fixer',
            output='screen',
            parameters=[{'cyberdog_ns': CYBERDOG_NS}],
        ),
        # cmd_vel 桥接（Twist → CyberDog SE3VelocityCMD）
        Node(
            package='cyberdog_nav2_lidar',
            executable='cmd_vel_bridge.py',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{'cyberdog_ns': CYBERDOG_NS}],
        ),
    ]

    # ── 定位层节点 ───────────────────────────────────────────────────────────

    localization_nodes = []

    if mode == 'slam':
        localization_nodes = [
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-configuration_directory', config_dir,
                    '-configuration_basename', 'cyberdog_slam.lua',
                ],
                remappings=[
                    ('scan', '/scan_fixed'),
                    ('odom', f'/{CYBERDOG_NS}/odom_out_fixed'),
                    ('imu', '/imu/data_fixed'),
                ],
            ),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'resolution': 0.05,
                    'publish_period_sec': 1.0,
                }],
            ),
        ]
    elif mode == 'nav':
        carto_args = [
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cyberdog_localization.lua',
        ]
        if pbstream_file:
            carto_args += ['-load_state_filename', pbstream_file]

        localization_nodes = [
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=carto_args,
                remappings=[
                    ('scan', '/scan_fixed'),
                    ('odom', f'/{CYBERDOG_NS}/odom_out_fixed'),
                    ('imu', '/imu/data_fixed'),
                ],
            ),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'resolution': 0.05,
                    'publish_period_sec': 1.0,
                }],
                remappings=[('map', '/cartographer_map')],
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'yaml_filename': map_yaml,
                    'use_sim_time': use_sim_time,
                    'topic_name': 'map',
                    'frame_id': 'map',
                }],
            ),
        ]
    else:  # amcl
        localization_nodes = [
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'yaml_filename': map_yaml,
                    'use_sim_time': use_sim_time,
                    'topic_name': 'map',
                    'frame_id': 'map',
                }],
            ),
        ]

    # ── Nav2 导航栈 ──────────────────────────────────────────────────────────

    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
    ]

    # ── Lifecycle 管理 ───────────────────────────────────────────────────────

    nav_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    if mode == 'amcl':
        nav_lifecycle_nodes = ['map_server', 'amcl'] + nav_lifecycle_nodes
    elif mode == 'nav':
        nav_lifecycle_nodes = ['map_server'] + nav_lifecycle_nodes

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': nav_lifecycle_nodes,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── RViz（可选）──────────────────────────────────────────────────────────

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'config', 'nav2_rviz.rviz')],
        condition=IfCondition(launch_rviz),
    )

    return common_nodes + localization_nodes + nav2_nodes + [lifecycle_manager, rviz_node]


def generate_launch_description():
    bringup_dir = get_package_share_directory('cyberdog_nav2_lidar')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        DeclareLaunchArgument(
            'mode', default_value='slam',
            description='slam / nav / amcl',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(bringup_dir, 'maps', 'map.yaml'),
            description='Map YAML for AMCL mode',
        ),
        DeclareLaunchArgument(
            'pbstream_file',
            default_value=os.path.join(bringup_dir, 'maps', 'map.pbstream'),
            description='.pbstream map file for Cartographer nav mode',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),

        OpaqueFunction(function=_launch_setup),
    ])
