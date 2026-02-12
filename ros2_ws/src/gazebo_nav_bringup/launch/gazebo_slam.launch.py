#!/usr/bin/env python3
"""
Gazebo SLAM Launch File (SLAM only, no Nav2)

Launches Gazebo (house world) + SLAM Toolbox + RViz2.
Use this for manual mapping — drive with teleop to build the map,
then save it with: ros2 run nav2_map_server map_saver_cli -f ~/house_map

Author: Saman Aboutorab
Date: February 2026
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_nav_bringup = get_package_share_directory('gazebo_nav_bringup')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    slam_params_file = os.path.join(
        pkg_gazebo_nav_bringup, 'config', 'slam_params_turtlebot3.yaml'
    )
    house_world = os.path.join(
        pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_house.world'
    )
    rviz_config = os.path.join(
        pkg_gazebo_nav_bringup, 'rviz', 'slam_config.rviz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    # ── Gazebo model path — MUST be set before Gazebo starts ─────────────────
    set_gz_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_turtlebot3_gazebo, 'models')
    )

    # ── Gazebo server + client ────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', house_world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # ── Robot state publisher + spawn ─────────────────────────────────────────
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
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

    # ── RViz2 ─────────────────────────────────────────────────────────────────
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
    # env var FIRST — before any Gazebo process starts
    ld.add_action(set_gz_model_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot3)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    return ld
