# Autonomous Mobile Robot Navigation

SLAM, Nav2, and Vision-Based Navigation in Gazebo with ROS2 Jazzy

## Overview

This project implements a complete autonomous mobile robot navigation system using ROS2 Jazzy. The robot simultaneously builds a map of an unknown environment (SLAM) and navigates to user-defined goals (Nav2), all running in Gazebo simulation with a Turtlebot3 Waffle.

**Current capabilities:**
- Real-time SLAM mapping (SLAM Toolbox, online async mode)
- Autonomous path planning and obstacle avoidance (Nav2 stack)
- LiDAR-based reactive gap-following navigation
- Single-command launch orchestrating 15+ ROS2 nodes

**In progress:** Vision-based navigation with camera integration and object detection (see `feature/vision-nav` branch).

## Tech Stack

- **Framework:** ROS2 Jazzy
- **Simulator:** Gazebo Sim 8.x
- **Robot:** Turtlebot3 Waffle
- **SLAM:** SLAM Toolbox (Ceres solver, loop closure)
- **Navigation:** Nav2 (BT Navigator, RegulatedPurePursuit controller, Navfn planner)
- **Language:** Python 3

## Quick Start

### One-Time Setup
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
colcon build --symlink-install
```

### SLAM Only (map building + teleop)
```bash
# Terminal 1: Launch system
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch gazebo_nav_bringup gazebo_slam.launch.py

# Terminal 2: Drive the robot
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

### SLAM + Autonomous Navigation
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch gazebo_nav_bringup gazebo_slam_nav.launch.py
# Use RViz "2D Nav Goal" to send the robot to a target pose
```

### Save Map
```bash
cd ~/projects/Robotics/gazebo_robot_nav
source ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f my_map --use-sim-time
```

## Project Structure

```
gazebo_robot_nav/
├── ros2_ws/
│   ├── src/gazebo_nav_bringup/          # Main bringup package
│   │   ├── launch/
│   │   │   ├── gazebo_slam.launch.py    # SLAM only
│   │   │   └── gazebo_slam_nav.launch.py # SLAM + Nav2
│   │   ├── config/
│   │   │   ├── slam_params_turtlebot3.yaml
│   │   │   └── nav2_params.yaml
│   │   └── rviz/
│   │       └── slam_config.rviz
│   │
│   └── reactive_nav/                    # Reactive navigation package
│       └── reactive_nav/
│           ├── gap_follower.py          # 3-sector gap-following avoidance
│           └── scan_sanitizer.py        # LiDAR data preprocessing
│
├── src/                                 # Legacy Isaac Sim scripts (reference)
├── slam_map.pgm / slam_map.yaml        # Saved demo map
├── verify_gazebo_slam_setup.sh          # Pre-flight dependency check
├── ARCHITECTURE.md                      # System architecture deep dive
├── README_GAZEBO_SLAM.md               # Full SLAM demo documentation
├── README_ISAAC_SLAM_ATTEMPT.md        # Isaac Sim attempt (archived learning)
└── TEST_GAZEBO_SLAM.md                 # Testing checklist
```

## Architecture

```
┌─────────────┐     /scan      ┌──────────────┐     /map      ┌─────────────┐
│   Gazebo     │ ────────────> │ SLAM Toolbox  │ ───────────> │    RViz2     │
│  Simulator   │               │  (Mapping)    │              │ (Visualizer) │
│             │ <────────────  │              │              │              │
└─────────────┘    /cmd_vel    └──────────────┘              └─────────────┘
       │                              │
       │ /odom                       │ map→odom TF
       v                              v
┌─────────────────────────────────────────────┐
│              Nav2 Stack                      │
│  BT Navigator → Planner → Controller        │
│  Velocity Smoother → Collision Monitor       │
└─────────────────────────────────────────────┘
```

**TF Tree:** `map → odom → base_footprint → base_link → {base_scan, camera_link, wheels}`

## Branch Strategy

```
main                          # Stable, tagged releases
└── feature/vision-nav        # Camera + object detection + vision navigation
```

## Roadmap

- [x] SLAM Toolbox integration with Gazebo
- [x] Nav2 autonomous navigation
- [x] Reactive gap-following obstacle avoidance
- [ ] Camera sensor integration
- [ ] Object detection (YOLOv8) with ROS2
- [ ] Vision-based "find and go to object" behavior
- [ ] Depth camera + 3D perception
- [ ] ML training pipeline with synthetic Gazebo data

## Documentation

| Document | Description |
|----------|-------------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | System architecture, data flow, TF tree |
| [README_GAZEBO_SLAM.md](README_GAZEBO_SLAM.md) | Full SLAM demo docs |
| [TEST_GAZEBO_SLAM.md](TEST_GAZEBO_SLAM.md) | Testing procedures |
| [QUICK_START.md](QUICK_START.md) | One-page quick start |
| [README_ISAAC_SLAM_ATTEMPT.md](README_ISAAC_SLAM_ATTEMPT.md) | Isaac Sim learning experience |

## License

MIT
