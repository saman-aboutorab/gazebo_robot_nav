#!/bin/bash
# Kill all ROS2, Gazebo, and related processes before a fresh launch

echo "Killing ROS2 nodes..."
pkill -9 -f "nav2_" 2>/dev/null
pkill -9 -f "slam_toolbox" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "ros2 run" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "teleop" 2>/dev/null

echo "Killing Gazebo..."
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gz " 2>/dev/null
pkill -9 -f "ruby.*gz" 2>/dev/null

echo "Waiting for DDS + Gazebo to fully stop..."
sleep 3

# Verify
REMAINING=$(ps aux | grep -E "gz|rviz|slam|nav2" | grep -v grep | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo "All clear. Safe to relaunch."
else
    echo "Warning: $REMAINING processes still running:"
    ps aux | grep -E "gz|rviz|slam|nav2" | grep -v grep
    echo "Try running this script again or use: kill -9 <PID>"
fi
