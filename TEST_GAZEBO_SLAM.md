# Gazebo SLAM Testing Guide

## Quick Start

### Terminal 1: Launch SLAM System
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle

ros2 launch gazebo_nav_bringup gazebo_slam.launch.py
```

**Expected behavior:**
- Gazebo window opens with Turtlebot3 in a world with obstacles
- RViz window opens showing:
  - Empty map (gray background)
  - Robot model (blue Turtlebot3)
  - Red laser scan points
  - TF frame axes

### Terminal 2: Teleoperation Control
```bash
cd ~/projects/Robotics/gazebo_robot_nav/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle

ros2 run turtlebot3_teleop teleop_keyboard
```

**Keyboard controls:**
- `w` - Move forward
- `x` - Move backward
- `a` - Turn left (counter-clockwise)
- `d` - Turn right (clockwise)
- `s` - Stop
- `q/z` - Increase/decrease linear speed
- `e/c` - Increase/decrease angular speed

## Testing Procedure

### Step 1: Verify Launch (2 minutes)
- [ ] Gazebo opens without errors
- [ ] Turtlebot3 robot is visible in Gazebo
- [ ] RViz opens with displays configured
- [ ] No error messages in terminal

### Step 2: Verify Data Flow (2 minutes)
Run in Terminal 3:
```bash
# Check topics are publishing
ros2 topic hz /scan --use-sim-time     # Should show ~5 Hz
ros2 topic hz /odom --use-sim-time     # Should show ~50 Hz
ros2 topic hz /map --use-sim-time      # Should show ~1 Hz after movement

# Check TF chain
ros2 run tf2_tools view_frames
# Open frames.pdf - should show: map → odom → base_footprint → base_link → base_scan
```

### Step 3: Test Mapping (5 minutes)
1. Start teleop in Terminal 2
2. Drive robot forward slowly with `w` key
3. Watch RViz map display - should see black walls appear
4. Turn robot left with `a` key
5. Continue exploring the environment
6. Map should gradually fill in as you move

**Success criteria:**
- [ ] Map updates in real-time as robot moves
- [ ] Walls appear as black lines/areas
- [ ] Free space appears as white/light gray
- [ ] Robot position updates on map
- [ ] Laser scan shows red points hitting obstacles

### Step 4: Test Loop Closure (3 minutes)
1. Drive robot in a large circle/loop
2. Return to starting area
3. Watch for map "snapping" - small corrections as SLAM recognizes the location

**Success criteria:**
- [ ] Map remains consistent
- [ ] No major distortions
- [ ] Robot correctly localized

### Step 5: Save Map (1 minute)
```bash
# In Terminal 3
cd ~/projects/Robotics/gazebo_robot_nav
ros2 run nav2_map_server map_saver_cli -f slam_map --ros-args -p use_sim_time:=true
```

**Expected output:**
```
Map saved to: turtlebot3_map.pgm
Map metadata saved to: turtlebot3_map.yaml
```

### Step 6: Test Navigation (Optional - 5 minutes)
1. In RViz, click "2D Nav Goal" button (top toolbar)
2. Click on map where you want robot to go
3. Drag to set orientation
4. Robot should plan path and start moving

**Note:** This requires Nav2 to be running. If SLAM only, skip this step.

## Verification Checklist

### Functionality
- [ ] Robot spawns in Gazebo correctly
- [ ] Laser scans visible in RViz
- [ ] Map builds as robot explores
- [ ] TF chain complete (no missing transforms)
- [ ] Teleop controls robot successfully
- [ ] Map saves successfully

### Performance
- [ ] /scan topic: ~5 Hz (expected)
- [ ] /odom topic: ~50 Hz (expected)
- [ ] /map topic: ~1 Hz (expected)
- [ ] No lag in RViz visualization
- [ ] Gazebo runs smoothly (30+ FPS)

### Visual Quality
- [ ] Map shows clear walls
- [ ] Minimal drift or distortion
- [ ] Consistent scale
- [ ] Loop closure works correctly

## Troubleshooting

### Issue: Gazebo doesn't start
**Solution:**
```bash
export TURTLEBOT3_MODEL=waffle
# Verify environment variable is set
echo $TURTLEBOT3_MODEL
```

### Issue: No map appearing in RViz
**Check:**
1. Is robot moving? (SLAM needs movement)
2. Is /map topic publishing? (`ros2 topic hz /map --use-sim-time`)
3. Is SLAM node running? (`ros2 node list` should show `/slam_toolbox`)

### Issue: TF errors
**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```
Should see complete chain: map → odom → base_footprint → base_link → base_scan

### Issue: Robot doesn't move with teleop
**Check:**
1. Is teleop sending commands? (should see "currently: ..." in terminal)
2. Is /cmd_vel being published? (`ros2 topic hz /cmd_vel`)
3. Is Gazebo running? (robot might be paused)

## Demo Recording Tips

### For Screenshots:
1. **Empty map** - Right after launch
2. **Partial exploration** - After 30 seconds of driving
3. **Complete map** - After exploring entire environment
4. **With laser scans** - Show red scan points hitting walls

### For Video:
1. Record RViz window
2. Show map building in real-time
3. Demonstrate robot control with teleop
4. Show final complete map
5. ~2-3 minutes total length

### Recording Commands:
```bash
# Screen recording (if using GNOME)
# Ctrl+Shift+Alt+R to start/stop recording

# Or use SimpleScreenRecorder
sudo apt install simplescreenrecorder
simplescreenrecorder
```

## Expected Results

### Successful Demo Shows:
✅ Turtlebot3 robot in Gazebo environment
✅ Real-time SLAM mapping in RViz
✅ Accurate wall detection
✅ Smooth robot motion
✅ Complete map after exploration
✅ Saved map files (.pgm + .yaml)

### Demo Duration:
- Setup: ~30 seconds
- Exploration: ~2-3 minutes
- Map save: ~10 seconds
- **Total: ~3-4 minutes**

## Next Steps After Testing

If all tests pass:
1. Record demo video
2. Take screenshots
3. Document findings
4. Move to Phase 4 (Documentation)

If issues found:
1. Document errors
2. Check logs in ~/.ros/log/
3. Debug using troubleshooting guide
4. Iterate on configuration
