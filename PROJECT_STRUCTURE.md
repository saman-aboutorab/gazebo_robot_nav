# Project Structure & Resume Strategy

**Repository:** `gazebo_robot_nav`
**Focus:** Autonomous Mobile Robotics with ROS2 Jazzy
**Target Roles:** Software Engineer / AI Engineer in Robotics

## Git Branch Strategy

```
main (stable demos)
│
├── archive/isaac-slam-nav2-attempt ✅ ARCHIVED
│   └── "Learning Experience" — Documented failed attempt
│       - Shows debugging skills and systematic problem-solving
│       - 95% functional but blocked by Isaac Sim TF bug
│       - Led to strategic pivot to Gazebo
│
├── feature/slam-gazebo ✅ COMPLETE
│   └── Demo 1: "ROS2 SLAM & Autonomous Navigation"
│       - Gazebo + Turtlebot3 Waffle + SLAM Toolbox + Nav2
│       - Real-time map building (SLAM Toolbox)
│       - Autonomous path planning (Nav2 stack)
│       - Single-command launch
│
└── feature/isaac-cv 📋 PLANNED
    └── Demo 2: "Isaac Sim Computer Vision Pipeline"
        - Object detection in simulation
        - Synthetic data generation
        - Vision-based navigation
```

## Repository Structure (feature/slam-gazebo)

```
gazebo_robot_nav/
│
├── ros2_ws/
│   ├── reactive_nav/                        # Reactive navigation package
│   │   ├── gap_follower.py                  # 3-sector gap-following avoidance
│   │   └── scan_sanitizer.py                # LiDAR data preprocessing
│   │
│   └── src/gazebo_nav_bringup/               # Main bringup package
│       ├── launch/
│       │   ├── gazebo_slam.launch.py        # SLAM only (no Nav2)
│       │   └── gazebo_slam_nav.launch.py    # SLAM + Nav2 (full demo)
│       ├── config/
│       │   ├── slam_params_turtlebot3.yaml  # SLAM Toolbox configuration
│       │   └── nav2_params.yaml             # Full Nav2 stack parameters
│       └── rviz/
│           └── slam_config.rviz             # Pre-configured visualization
│
├── src/
│   ├── 00_basic_drive.py                    # Original Isaac Sim drive script
│   └── 20_ros2_laserscan.py                 # Isaac Sim ROS2 bridge script
│
├── slam_map.pgm / slam_map.yaml             # Saved SLAM map
├── kill_ros.sh                              # Kill all ROS2 processes (cleanup)
├── verify_gazebo_slam_setup.sh              # Pre-flight dependency check
│
├── README.md                                # Main project overview
├── README_GAZEBO_SLAM.md                    # Full demo documentation
├── README_ISAAC_SLAM_ATTEMPT.md             # Isaac Sim attempt (archived)
├── QUICK_START.md                           # One-page quick start
├── TEST_GAZEBO_SLAM.md                      # Manual testing checklist
├── ARCHITECTURE.md                          # System architecture explanation
└── PROJECT_STRUCTURE.md                     # ← You are here
```

## Resume Presentation Strategy

### Project Title
**"Autonomous Mobile Robot Navigation: SLAM & Nav2 Demo"**

### Project Description
> Built a complete autonomous navigation system using ROS2 Jazzy. The robot simultaneously
> builds a map of an unknown environment (SLAM) while navigating to user-defined goals
> (Nav2). Demonstrated in Gazebo simulation with Turtlebot3 Waffle.

### Key Technical Skills Demonstrated

**Demo 1: Gazebo SLAM + Nav2 (feature/slam-gazebo)**
- ROS2 Jazzy — nodes, topics, TF2 transforms, lifecycle nodes, launch files
- SLAM Toolbox — online async mapping, loop closure, sensor fusion
- Nav2 stack — BT Navigator, planner/controller servers, costmaps
- Gazebo Sim — robot simulation, sensor integration, world creation
- Full system integration — 15+ nodes coordinated via lifecycle manager

**Bonus: Learning from Failure (archive/isaac-slam-nav2-attempt)**
- Systematic debugging over multiple sessions
- Root cause analysis (identified Isaac Sim OmniGraph TF bug)
- Strategic pivot when facing a blocker
- Clear technical documentation of issues and decisions

## Interview Talking Points

### "Tell me about a challenging technical problem"
> "I was integrating NVIDIA Isaac Sim with ROS2's SLAM system. Everything worked except
> the map wouldn't build. After systematic debugging, I discovered Isaac Sim's Transform
> Tree node wasn't publishing the robot's dynamic position — the robot appeared frozen to
> SLAM even though it was moving in the simulation. Rather than spending weeks on a
> simulator bug, I pivoted to Gazebo where TF works reliably, and built a fully functional
> SLAM + autonomous navigation demo. This showed me the importance of knowing when to
> persist vs. when to pivot."

### "What's your experience with ROS?"
> "I've built a complete autonomous navigation system with ROS2 Jazzy — integrating SLAM
> Toolbox, Nav2, and Gazebo into a single-command launch. I'm comfortable with the full
> ROS2 stack: nodes, topics, actions, TF2 transforms, lifecycle management, and parameter
> configuration. I've debugged multi-node systems using ros2 introspection tools and
> have hands-on experience tuning costmaps, planner tolerances, and controller parameters."

### "Any experience with simulation?"
> "Yes — I've worked with both Gazebo Sim and NVIDIA Isaac Sim. I built a full SLAM and
> autonomous navigation demo in Gazebo. I also integrated Isaac Sim with ROS2 for a SLAM
> attempt, and learned how Isaac Sim's OmniGraph action graph and ROS2 bridge work at a
> deep level before identifying a platform-level TF publishing bug."

## Next Steps

1. **Record demo video** — 2-3 min showing map building + autonomous navigation
2. **Add git tag** — `v0.4-gazebo-slam-nav2`
3. **Commit current state** with clear message
4. **Merge to main** once demo is recorded and documented
5. **feature/isaac-cv** — computer vision demo (future)
6. **Update main README** with links to both demos
