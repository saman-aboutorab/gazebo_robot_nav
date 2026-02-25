# Autonomous Mobile Robot Navigation

SLAM, Nav2, and Vision-Based Navigation in Gazebo with ROS2 Jazzy

## Overview

This project implements a complete autonomous mobile robot navigation system using ROS2 Jazzy. The robot simultaneously builds a map of an unknown environment (SLAM) and navigates to user-defined goals (Nav2), all running in Gazebo simulation with a Turtlebot3 Waffle.

**Current capabilities:**
- Real-time SLAM mapping (SLAM Toolbox, online async mode)
- Autonomous path planning and obstacle avoidance (Nav2 stack)
- LiDAR-based reactive gap-following navigation
- Camera sensor with image bridging (640x480, ~5 Hz)
- YOLOv8 object detection with annotated image output
- Depth camera with real distance measurement
- Single-command launch orchestrating 15+ ROS2 nodes

## Tech Stack

- **Framework:** ROS2 Jazzy
- **Simulator:** Gazebo Sim 8.x
- **Robot:** Turtlebot3 Waffle
- **SLAM:** SLAM Toolbox (Ceres solver, loop closure)
- **Navigation:** Nav2 (BT Navigator, RegulatedPurePursuit controller, Navfn planner)
- **Vision:** YOLOv8 (Ultralytics), OpenCV, cv_bridge
- **Language:** Python 3

## Quick Start

### One-Time Setup
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Verify Setup
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 pkg list | grep gazebo_nav_bringup   # Should print: gazebo_nav_bringup
../verify_gazebo_slam_setup.sh             # All checks should pass
```

### SLAM Only (map building + teleop)

**Terminal 1** — Launch Gazebo (house world) + SLAM + RViz:
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch gazebo_nav_bringup gazebo_slam.launch.py
```

**Terminal 2** — Drive the robot with keyboard:
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

**Controls:** `w` forward, `x` backward, `a` turn left, `d` turn right, `s` stop

### SLAM + Autonomous Navigation
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch gazebo_nav_bringup gazebo_slam_nav.launch.py
# Use RViz "2D Nav Goal" to send the robot to a target pose
```

### Vision Detection

> Requires SLAM + Nav2 running in Terminal 1 (see above).

**Terminal 2** — Start YOLOv8 detector:
```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/Robotics/gazebo_robot_nav/ros2_ws/install/setup.bash
ros2 run gazebo_nav_bringup vision_detector
```

**View detections in RViz:** Add → By topic → `/camera/detections_image` → Image

**Spawn a person model** (gives the detector something to find):
```bash
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person", pose: {position: {x: 2.5, y: -2.5, z: 0}}'
```

### Find and Go to Object

Uses foreground-filtered 3D centroid depth + TF2 map-frame goal localization (Phase 2): detects object with YOLOv8, back-projects all depth pixels in the bounding box to camera-frame 3D points, isolates the nearest depth cluster (foreground object, not background wall), computes the 3D centroid, transforms to `/map` via TF2, and sends a Nav2 goal in map coordinates.

**Step 1 — Terminal 1:** Launch the full stack and wait for `Managed nodes are active`:
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash && export TURTLEBOT3_MODEL=waffle
ros2 launch gazebo_nav_bringup gazebo_slam_nav.launch.py
```

**Step 2 — Terminal 2:** Drive around until the map covers the area where the target will be (watch RViz — unexplored areas show as gray, mapped free space is white):
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

**Step 3 — Terminal 3:** Spawn a person model in the Gazebo world:
```bash
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person", pose: {position: {x: 2.5, y: -2.5, z: 0}}'
```

**Step 4 — Terminal 4:** Run the find-and-go node (`use_sim_time:=true` is required):
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run gazebo_nav_bringup find_and_go --ros-args -p target_object:=person -p use_sim_time:=true
```

The robot rotates to scan the room, detects the target with YOLOv8, computes its position in the `/map` frame via TF2, then sends a Nav2 goal. If navigation fails (e.g. goal outside current map bounds), the node automatically returns to searching.

Configurable parameters: `target_object` (default: `person`), `confidence_threshold` (default: `0.5`), `stop_distance` (default: `1.0` m).

### Save Map
```bash
cd ~/projects/Robotics/gazebo_robot_nav
source ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f slam_map --ros-args -p use_sim_time:=true
```

### Cleanup
```bash
# Kill all ROS2/Gazebo processes (run before each fresh launch)
~/projects/Robotics/gazebo_robot_nav/kill_all.sh
```

## Project Structure

```
gazebo_robot_nav/
├── ros2_ws/
│   ├── src/gazebo_nav_bringup/          # Main bringup package
│   │   ├── launch/
│   │   │   ├── gazebo_slam.launch.py    # SLAM only
│   │   │   └── gazebo_slam_nav.launch.py # SLAM + Nav2 + camera
│   │   ├── gazebo_nav_bringup/
│   │   │   ├── vision_detector_node.py  # YOLOv8 detection node
│   │   │   └── find_and_go_node.py     # Vision search + Nav2 goal node
│   │   ├── models/
│   │   │   └── turtlebot3_waffle/       # Local model (RGB 640x480 + depth 320x240)
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
┌──────────────┐    /scan     ┌──────────────┐    /map     ┌─────────────┐
│    Gazebo     │ ──────────> │ SLAM Toolbox  │ ─────────> │    RViz2     │
│   Simulator   │              │  (Mapping)    │             │ (Visualizer) │
│              │ <──────────  │              │             │              │
└──────────────┘   /cmd_vel   └──────────────┘             └─────────────┘
   │   │                             │
   │   │  /camera/image_raw          │ map→odom TF
   │   │  /camera/depth              v
   │   │                    ┌─────────────────────────────────────────────┐
   │   └──────────────────> │              Nav2 Stack                      │
   │  /odom                 │  BT Navigator → Planner → Controller        │
   v                        │  Velocity Smoother → Collision Monitor       │
┌──────────────┐            └─────────────────────────────────────────────┘
│  Find-and-Go  │                          ^
│  (YOLOv8 +    │  NavigateToPose action   │
│   Depth)      │ ─────────────────────────┘
└──────────────┘
```

**TF Tree:** `map → odom → base_footprint → base_link → {base_scan, camera_rgb_frame, wheels}`

## Branch Strategy

```
main                          # Stable, tagged releases
└── feature/vision-nav        # Camera + object detection + vision navigation
```

## Roadmap

### Completed
- [x] SLAM Toolbox integration with Gazebo
- [x] Nav2 autonomous navigation
- [x] Reactive gap-following obstacle avoidance
- [x] Camera sensor integration
- [x] Object detection (YOLOv8) with ROS2
- [x] Vision-based "find and go to object" behavior
- [x] Depth camera + 3D perception

### Upcoming Phases

- [x] **Phase 1 — Map-Frame Goal Localization**: Transform detected object positions into the `/map` frame using TF2 (`depth + bbox → camera_frame → base_link → odom → map`) before sending Nav2 goals. Fixes a fundamental flaw where the current node ignores SLAM localization and relies on drifting odometry instead.

- [x] **Phase 2 — Point Cloud Centroid for Depth**: Replace the single-pixel depth with a foreground-filtered 3D centroid: every depth pixel inside the YOLO bounding box is back-projected to a camera-frame 3D point, the nearest depth cluster (the foreground object) is isolated by discarding pixels more than 20 % farther than the minimum depth in the region (which strips the background wall), and the median 3D centroid of the remaining points is used as the goal point. Implemented in pure NumPy — no PCL or Open3D dependency — which is sufficient for the clean, noise-free depth data produced by Gazebo. For real-world deployment with physically noisy sensors (Intel RealSense, Azure Kinect), the simple depth-threshold foreground filter could be replaced with PCL's Statistical Outlier Removal (C++) or Open3D's Euclidean clustering (Python) for more robust foreground/background separation.

- [ ] **Phase 3 — Multi-Object Targeting with Selection Logic**: Support detecting multiple objects simultaneously and add a configurable selection policy — closest by depth, highest confidence, or user-specified target class via ROS2 parameter at runtime.

- [ ] **Phase 4 — Nav2 Behavior Tree Integration**: Encode the find-and-go behavior as a proper BT with nodes: `SearchForTarget → ComputeTargetPose → NavigateToPose → Recovery`. Makes the behavior maintainable, extensible, and failure-aware at the architecture level.

- [ ] **Phase 5 — Vision Obstacles into Nav2 Costmap**: Feed camera-detected obstacles into the Nav2 costmap — either via depth-to-point-cloud through the obstacle layer, or as semantic keepout polygons for detected classes (e.g. person, fragile object). Extends Phase 4's costmap infrastructure with live camera perception.

- [ ] **Phase 6 — Dynamic Obstacles with Moving Gazebo Actors**: Replace the static "Standing person" test model with a Gazebo `<actor>` that walks a scripted waypoint path. Verify that the Nav2 local costmap correctly marks and clears the moving obstacle, that the planner replans around it, and that the Phase 7 recovery behaviors (Wait, Spin) trigger correctly when the actor temporarily blocks the robot's path.

- [ ] **Phase 7 — Nav2 Recovery Behaviors**: Enable the built-in Nav2 recovery plugins (Spin, BackUp, Wait) in `nav2_params.yaml` and wire them into the behavior tree. When Nav2 gets stuck — path blocked, costmap artifact, unreachable goal — the robot will automatically attempt recovery before failing. Also tune local costmap inflation radius for smoother potential fields around obstacles.

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
