#!/usr/bin/env python3
"""
Gazebo SLAM + Nav2 Launch File

Launches Gazebo + SLAM Toolbox + Nav2 + RViz2 together.
Nav2 nodes are launched individually (no docking_server) so the
lifecycle_manager doesn't wait for an unused node at startup.

Robot can be commanded autonomously via RViz2 "2D Goal Pose" tool.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_nav_bringup = get_package_share_directory('gazebo_nav_bringup')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params_file = os.path.join(
        pkg_gazebo_nav_bringup, 'config', 'slam_params_turtlebot3.yaml'
    )
    nav2_params_file = os.path.join(
        pkg_gazebo_nav_bringup, 'config', 'nav2_params.yaml'
    )
    rviz_config = os.path.join(
        pkg_gazebo_nav_bringup, 'rviz', 'slam_config.rviz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # TF remappings required by all Nav2 nodes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Gazebo + Turtlebot3 ──────────────────────────────────────────────────
    gazebo_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': '0.5',
            'y_pose': '0.5'
        }.items()
    )

    # ── SLAM Toolbox ─────────────────────────────────────────────────────────
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    # ── Nav2 nodes ───────────────────────────────────────────────────────────
    # cmd_vel chain:
    #   controller_server → cmd_vel_nav
    #   → velocity_smoother (reads cmd_vel_nav, publishes cmd_vel_smoothed)
    #   → collision_monitor (reads cmd_vel_smoothed, publishes cmd_vel)
    #   → Turtlebot3 bridge (reads cmd_vel) → robot

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    # Lifecycle manager — only manages the nodes we actually launch
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
            ],
        }],
    )

    # ── RViz2 ────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo_turtlebot3)
    ld.add_action(slam_toolbox_node)
    ld.add_action(controller_server)
    ld.add_action(smoother_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)
    ld.add_action(collision_monitor)
    ld.add_action(lifecycle_manager)
    ld.add_action(rviz_node)
    return ld
