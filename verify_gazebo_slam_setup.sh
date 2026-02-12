#!/bin/bash

# Gazebo SLAM Setup Verification Script
# This script checks if all dependencies and configurations are correct

set -e

echo "========================================="
echo "Gazebo SLAM Setup Verification"
echo "========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Counters
PASSED=0
FAILED=0

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
    ((PASSED++))
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    ((FAILED++))
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# Check 1: ROS2 installation
echo "Checking ROS2 installation..."
if command -v ros2 &> /dev/null; then
    check_pass "ROS2 command found"
else
    check_fail "ROS2 not found"
fi

# Check 2: Gazebo installation
echo ""
echo "Checking Gazebo installation..."
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>&1 | head -1)
    check_pass "Gazebo found: $GZ_VERSION"
else
    check_fail "Gazebo (gz) not found"
fi

# Check 3: Turtlebot3 packages
echo ""
echo "Checking Turtlebot3 packages..."
if ros2 pkg list | grep -q "turtlebot3_gazebo"; then
    check_pass "turtlebot3_gazebo package found"
else
    check_fail "turtlebot3_gazebo package not found"
fi

if ros2 pkg list | grep -q "turtlebot3_description"; then
    check_pass "turtlebot3_description package found"
else
    check_fail "turtlebot3_description package not found"
fi

if ros2 pkg list | grep -q "turtlebot3_teleop"; then
    check_pass "turtlebot3_teleop package found"
else
    check_fail "turtlebot3_teleop package not found"
fi

# Check 4: SLAM Toolbox
echo ""
echo "Checking SLAM Toolbox..."
if ros2 pkg list | grep -q "slam_toolbox"; then
    check_pass "slam_toolbox package found"
else
    check_fail "slam_toolbox package not found"
fi

# Check 5: Nav2 packages
echo ""
echo "Checking Nav2 packages..."
if ros2 pkg list | grep -q "nav2_map_server"; then
    check_pass "nav2_map_server package found"
else
    check_fail "nav2_map_server package not found"
fi

# Check 6: gazebo_nav_bringup package
echo ""
echo "Checking gazebo_nav_bringup package..."
if ros2 pkg list | grep -q "gazebo_nav_bringup"; then
    check_pass "gazebo_nav_bringup package found"

    # Check if launch file exists
    PKG_SHARE=$(ros2 pkg prefix gazebo_nav_bringup)/share/gazebo_nav_bringup
    if [ -f "$PKG_SHARE/launch/gazebo_slam.launch.py" ]; then
        check_pass "gazebo_slam.launch.py found"
    else
        check_fail "gazebo_slam.launch.py not found in $PKG_SHARE/launch/"
    fi

    # Check if SLAM params exist
    if [ -f "$PKG_SHARE/config/slam_params_turtlebot3.yaml" ]; then
        check_pass "slam_params_turtlebot3.yaml found"
    else
        check_fail "slam_params_turtlebot3.yaml not found in $PKG_SHARE/config/"
    fi

    # Check if RViz config exists
    if [ -f "$PKG_SHARE/rviz/slam_config.rviz" ]; then
        check_pass "slam_config.rviz found"
    else
        check_warn "slam_config.rviz not found (RViz will use defaults)"
    fi
else
    check_fail "gazebo_nav_bringup package not found - did you build the workspace?"
fi

# Check 7: TURTLEBOT3_MODEL environment variable
echo ""
echo "Checking environment..."
if [ -z "$TURTLEBOT3_MODEL" ]; then
    check_warn "TURTLEBOT3_MODEL not set (will use default: waffle)"
    echo "         Set it with: export TURTLEBOT3_MODEL=waffle"
else
    check_pass "TURTLEBOT3_MODEL = $TURTLEBOT3_MODEL"
fi

# Check 8: Turtlebot3 models exist
echo ""
echo "Checking Turtlebot3 models..."
TB3_DESC_PATH=/opt/ros/jazzy/share/turtlebot3_description/urdf
if [ -f "$TB3_DESC_PATH/turtlebot3_waffle.urdf" ]; then
    check_pass "Turtlebot3 Waffle URDF found"
else
    check_fail "Turtlebot3 Waffle URDF not found"
fi

# Check 9: World files exist
echo ""
echo "Checking Gazebo world files..."
TB3_WORLDS=/opt/ros/jazzy/share/turtlebot3_gazebo/worlds
if [ -f "$TB3_WORLDS/turtlebot3_world.world" ]; then
    check_pass "turtlebot3_world.world found"
else
    check_fail "turtlebot3_world.world not found"
fi

if [ -f "$TB3_WORLDS/turtlebot3_house.world" ]; then
    check_pass "turtlebot3_house.world found"
else
    check_warn "turtlebot3_house.world not found"
fi

# Check 10: Launch file syntax
echo ""
echo "Checking launch file syntax..."
if ros2 launch gazebo_nav_bringup gazebo_slam.launch.py --show-args &> /dev/null; then
    check_pass "gazebo_slam.launch.py parses without errors"
else
    check_fail "gazebo_slam.launch.py has syntax errors"
fi

# Summary
echo ""
echo "========================================="
echo "Summary"
echo "========================================="
echo -e "${GREEN}Passed:${NC} $PASSED"
if [ $FAILED -gt 0 ]; then
    echo -e "${RED}Failed:${NC} $FAILED"
fi
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All critical checks passed!${NC}"
    echo ""
    echo "Ready to launch Gazebo SLAM:"
    echo "  1. cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws"
    echo "  2. source install/setup.bash"
    echo "  3. export TURTLEBOT3_MODEL=waffle"
    echo "  4. ros2 launch gazebo_nav_bringup gazebo_slam.launch.py"
    echo ""
    exit 0
else
    echo -e "${RED}✗ Some checks failed. Please fix the issues above.${NC}"
    echo ""
    exit 1
fi
