#!/usr/bin/env python3
"""
Gazebo SLAM Launch File for Turtlebot3

This launch file integrates:
- Gazebo simulator with Turtlebot3 Waffle robot
- SLAM Toolbox for real-time mapping
- RViz2 for visualization

Author: Saman Aboutorab
Date: February 2026
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_gazebo_nav_bringup = get_package_share_directory('gazebo_nav_bringup')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Paths
    slam_params_file = os.path.join(
        pkg_gazebo_nav_bringup,
        'config',
        'slam_params_turtlebot3.yaml'
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='turtlebot3_world')

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='World name (turtlebot3_world, turtlebot3_house, empty_world)'
    )

    # Gazebo + Turtlebot3 launch
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

    # SLAM Toolbox launch
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    # RViz2 node
    rviz_config = os.path.join(
        pkg_gazebo_nav_bringup,
        'rviz',
        'slam_config.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    # Create launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)

    # Add nodes/launches
    ld.add_action(gazebo_turtlebot3)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld
