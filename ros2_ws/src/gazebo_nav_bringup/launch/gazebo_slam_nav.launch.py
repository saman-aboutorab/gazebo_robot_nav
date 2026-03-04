#!/usr/bin/env python3
"""
Gazebo SLAM + Nav2 Launch File

Launches Gazebo (house world) + SLAM Toolbox + Nav2 + RViz2 together.
Nav2 nodes are launched individually (no docking_server) so the
lifecycle_manager doesn't wait for an unused node at startup.

Robot can be commanded autonomously via RViz2 "2D Goal Pose" tool.
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
    nav2_params_file = os.path.join(
        pkg_gazebo_nav_bringup, 'config', 'nav2_params.yaml'
    )
    rviz_config = os.path.join(
        pkg_gazebo_nav_bringup, 'rviz', 'slam_config.rviz'
    )
    house_world = os.path.join(
        pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_house.world'
    )
    local_model_path = os.path.join(
        pkg_gazebo_nav_bringup, 'models', 'turtlebot3_waffle', 'model.sdf'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    # TF remappings required by all Nav2 nodes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Gazebo model path — MUST be set before Gazebo starts ─────────────────
    # turtlebot3_house.launch.py sets this AFTER gz starts (bug), so we do it
    # here first and launch Gazebo directly instead of including that file.
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

    # Spawn robot using our local model (lower-res camera)
    spawn_turtlebot3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'waffle',
            '-file', local_model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
        output='screen',
    )

    # Bridge Gazebo topics to ROS (same config the stock launch uses)
    bridge_params = os.path.join(
        pkg_turtlebot3_gazebo, 'params', 'turtlebot3_waffle_bridge.yaml'
    )
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    # Bridge camera images (Gazebo → ROS): RGB + Depth
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw', '/camera/depth'],
        output='screen',
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

    # Subscribes: /plan (Path), /odom (Odometry), /tf
    # Publishes:  cmd_vel_nav (TwistStamped) — raw velocity command to follow the path
    # Algorithm:  RegulatedPurePursuit — looks ahead on /plan and steers toward it
    # Note: 'cmd_vel' remapped to 'cmd_vel_nav' to avoid writing directly to the robot;
    #       velocity_smoother sits downstream before the command reaches the robot.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    # Subscribes: /plan (Path)
    # Publishes:  /plan (Path) — a smoothed version of the global plan
    # Purpose:    Optional post-processing step that smooths sharp corners in the
    #             raw A* path before the controller tries to follow it.
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    # Subscribes: /map (OccupancyGrid), /tf, goal pose (via bt_navigator)
    # Publishes:  /plan (Path) — global waypoint list from current pose to goal
    # Algorithm:  Navfn (A* on the occupancy grid map)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    # Subscribes: /tf, costmaps
    # Publishes:  cmd_vel_nav (TwistStamped) — used by recovery behaviors (spin, backup)
    # Purpose:    Runs built-in recovery behaviors (Spin, BackUp, Wait) when the robot
    #             gets stuck. bt_navigator triggers these via the BT on failure.
    # Note: 'cmd_vel' remapped to 'cmd_vel_nav' same as controller_server.
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    # Subscribes: navigate_to_pose ACTION goal (NavigateToPose) from find_and_go_node
    #             /tf, /map, feedback from planner_server and controller_server
    # Publishes:  navigate_to_pose ACTION feedback + result
    # Purpose:    Orchestrator — runs Nav2's internal Behavior Tree to coordinate
    #             planner_server, controller_server, and behavior_server until the
    #             robot reaches the goal or a recovery is needed.
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    # Subscribes: list of waypoints (via FollowWaypoints action)
    # Publishes:  navigate_to_pose ACTION goals — one per waypoint in sequence
    # Purpose:    High-level multi-waypoint missions; sends each waypoint to
    #             bt_navigator one at a time. Not used in find_and_go mode.
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings,
    )

    # Subscribes: cmd_vel_nav (TwistStamped) — raw command from controller_server
    # Publishes:  cmd_vel_smoothed (TwistStamped) — acceleration-limited command
    # Purpose:    Prevents jerky motion by ramping velocity up/down within configured
    #             max acceleration limits. collision_monitor sits downstream.
    # Note: 'cmd_vel' remapped to 'cmd_vel_nav' so it reads from controller output,
    #       not the final /cmd_vel topic.
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    # Subscribes: cmd_vel_smoothed (TwistStamped), /scan (LaserScan)
    # Publishes:  /cmd_vel (TwistStamped) — the final command sent to the robot wheels
    # Purpose:    Safety layer — if LiDAR detects an obstacle within a configured
    #             polygon, it scales down or zeroes the velocity before it reaches
    #             the robot. Last node in the cmd_vel chain.
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

    # ── Vision obstacle node (Phase 4) ───────────────────────────────────────
    # Publishes YOLO-detected obstacles as PointCloud2 → /vision_obstacles
    # consumed by the local costmap obstacle_layer for early obstacle awareness.
    vision_obstacle_node = Node(
        package='gazebo_nav_bringup',
        executable='vision_obstacles',
        name='vision_obstacle_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
    # env var FIRST — before any Gazebo process starts
    ld.add_action(set_gz_model_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot3)
    ld.add_action(gz_ros_bridge)
    ld.add_action(image_bridge)
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
    ld.add_action(vision_obstacle_node)
    ld.add_action(rviz_node)
    return ld
