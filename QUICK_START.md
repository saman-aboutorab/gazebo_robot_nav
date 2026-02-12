# Quick Start - Gazebo SLAM Demo

## One-Time Setup
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
colcon build --symlink-install
```

## Run Demo

### Terminal 1: Launch System
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch gazebo_nav_bringup gazebo_slam.launch.py
```

### Terminal 2: Control Robot
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

## Controls
- `w` - Forward
- `x` - Backward
- `a` - Turn left
- `d` - Turn right
- `s` - Stop

## Save Map
```bash
cd ~/projects/Robotics/gazebo_robot_nav
source ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f my_map --use-sim-time
```

## Verify Setup
```bash
./verify_gazebo_slam_setup.sh
```

## Full Documentation
See [README_GAZEBO_SLAM.md](README_GAZEBO_SLAM.md) for complete details.
