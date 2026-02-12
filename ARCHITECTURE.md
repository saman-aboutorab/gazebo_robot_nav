# Gazebo SLAM Architecture & Data Flow

> **Purpose:** Understand how the SLAM system works conceptually before diving into code

**For:** Learning and reference
**Level:** Beginner to intermediate

## Table of Contents
1. [Big Picture Overview](#big-picture-overview)
2. [Detailed Architecture](#detailed-architecture)
3. [Complete Data Flow](#complete-data-flow)
4. [Topic & Transform Diagram](#topic--transform-diagram)
5. [What You Control](#what-you-control)
6. [How to Inspect Everything](#how-to-inspect-everything)
7. [Data Examples](#data-examples)

🤖 How the Gazebo SLAM Project Works
🎯 Big Picture Overview
When you run ros2 launch gazebo_nav_bringup gazebo_slam.launch.py, three major systems start up and talk to each other through ROS2 topics and transforms:


┌─────────────────┐         ┌──────────────────┐         ┌─────────────┐
│   GAZEBO SIM    │ ──────> │  SLAM TOOLBOX    │ ──────> │    RVIZ2    │
│   (Physics)     │ <────── │  (Brain/Mapper)  │ <────── │ (Visualizer)│
└─────────────────┘         └──────────────────┘         └─────────────┘
        ↑                            ↑                           ↑
        │                            │                           │
        └────────────────────────────┴───────────────────────────┘
                          You control via teleop
🏗️ Detailed Architecture
1. Gazebo Simulator (The "Real World")
What it does:

Simulates a physical robot (Turtlebot3 Waffle) in a world with walls/obstacles
Calculates physics (wheels turning, collisions, movement)
Simulates a 360° LiDAR sensor spinning and shooting laser beams
Nodes it runs:

gz_sim - The Gazebo server (physics engine)
robot_state_publisher - Publishes robot's URDF/structure
Topics it PUBLISHES:

/scan (LaserScan) - 360 laser measurements, ~360 points, 5 Hz
Example data: ranges: [0.5, 0.6, inf, 2.1, ...] (distances in meters)
/odom (Odometry) - Robot's position from wheel encoders, 50 Hz
Example: x: 1.2, y: 0.5, theta: 0.3 (position + rotation)
Topics it SUBSCRIBES to:

/cmd_vel (Twist) - Movement commands
Example: linear.x: 0.5, angular.z: 0.3 (forward 0.5 m/s, turn 0.3 rad/s)
TF Frames it publishes:

odom → base_footprint - Where the robot thinks it is (from wheel odometry)
base_footprint → base_link - Robot's body
base_link → base_scan - Where the LiDAR is mounted
2. SLAM Toolbox (The "Brain")
What it does:

Listens to laser scans and odometry
Builds a 2D map of the environment
Figures out where walls are
Corrects drift using "loop closure" (recognizing previously visited places)
Node name: /slam_toolbox

Topics it SUBSCRIBES to:

/scan - Gets laser measurements
/odom - Gets approximate position (has drift)
Topics it PUBLISHES:

/map (OccupancyGrid) - The 2D map, 1 Hz
Grid of cells: 0 = free space (white), 100 = wall (black), -1 = unknown (gray)
Resolution: 5 cm per cell (0.05 m)
TF Frame it publishes:

map → odom - Correction transform
This connects the "perfect map frame" to the "drifty odometry frame"
SLAM continuously adjusts this to fix drift
Parameters it uses:
From slam_params_turtlebot3.yaml:

odom_frame: odom - Which frame is odometry
base_frame: base_footprint - Which frame is the robot center
scan_topic: /scan - Where to get laser data
Loop closure settings - How to recognize previously visited areas
3. RViz2 (Your Eyes)
What it does:

Shows you everything visually
Displays the map as it builds
Shows laser scans as red dots
Shows robot model in 3D
Lets you click to set navigation goals
Node name: /rviz2

Topics it SUBSCRIBES to:

/map - Shows the occupancy grid (black walls, white space)
/scan - Shows laser points as red dots
/robot_description - Gets robot's 3D model (URDF)
/tf - Gets all transforms to position everything correctly
Displays you see:

Grid - Gray grid on the floor (reference)
Map - Black walls appear as robot explores
LaserScan - Red dots showing what LiDAR sees right now
RobotModel - Blue 3D Turtlebot3
TF - Small RGB axes showing frame positions
4. Teleop Node (Your Control)
What it does:

Converts keyboard presses to movement commands
Command: ros2 run turtlebot3_teleop teleop_keyboard

Topics it PUBLISHES:

/cmd_vel (Twist messages)
w key → linear.x: 0.22 (move forward)
a key → angular.z: 0.5 (turn left)
s key → linear.x: 0, angular.z: 0 (stop)
🔄 Complete Data Flow Example
Let's trace what happens when you press w to move forward:

Step 1: You press w

Teleop Node publishes:
  Topic: /cmd_vel
  Data: {linear: {x: 0.22}, angular: {z: 0.0}}
Step 2: Gazebo receives command

Gazebo's diff_drive plugin reads /cmd_vel
→ Calculates wheel speeds
→ Moves robot forward in simulation
→ Robot position updates: (0,0) → (0.1, 0)
Step 3: Gazebo publishes sensor data

LiDAR spins and measures distances:
  Topic: /scan
  Data: [0.5, 0.51, 0.52, ... 360 measurements]
  
Wheel encoders measure movement:
  Topic: /odom  
  Data: {x: 0.1, y: 0.0, theta: 0.0}
Step 4: SLAM Toolbox processes data

SLAM reads /scan and /odom
→ "I moved 0.1m forward"
→ "I see walls at 0.5m in front"
→ Updates map: Marks cells 0.5m ahead as "wall" (black)
→ Publishes updated map to /map
Step 5: RViz displays everything

RViz reads /map
→ Shows black pixels where walls detected

RViz reads /scan
→ Draws red dots 0.5m in front of robot

RViz reads /tf
→ Positions robot at (0.1, 0.0) on map
📡 Complete Topic & Transform Diagram

┌───────────────────────────────────────────────────────────┐
│                    TF TRANSFORM TREE                       │
├───────────────────────────────────────────────────────────┤
│                                                            │
│   map (fixed global frame)                                │
│    └─ odom (published by SLAM Toolbox)                    │
│        └─ base_footprint (published by Gazebo)            │
│            └─ base_link                                   │
│                ├─ base_scan (LiDAR sensor)                │
│                ├─ wheel_left_link                         │
│                ├─ wheel_right_link                        │
│                └─ camera_link                             │
│                                                            │
└───────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────┐
│                      ROS2 TOPICS                           │
├───────────────────────────────────────────────────────────┤
│                                                            │
│  /scan (sensor_msgs/LaserScan) ~5 Hz                      │
│    Publisher: Gazebo                                      │
│    Subscriber: SLAM Toolbox                               │
│    Data: 360 distance measurements (0.12m - 20m)          │
│                                                            │
│  /odom (nav_msgs/Odometry) ~50 Hz                         │
│    Publisher: Gazebo                                      │
│    Subscriber: SLAM Toolbox                               │
│    Data: Robot pose (x, y, theta) + velocities            │
│                                                            │
│  /cmd_vel (geometry_msgs/Twist) Variable                  │
│    Publisher: Teleop Keyboard                             │
│    Subscriber: Gazebo                                     │
│    Data: linear.x (m/s), angular.z (rad/s)                │
│                                                            │
│  /map (nav_msgs/OccupancyGrid) ~1 Hz                      │
│    Publisher: SLAM Toolbox                                │
│    Subscriber: RViz                                       │
│    Data: 2D grid, cells are 0-100 (free-occupied)         │
│                                                            │
│  /robot_description (std_msgs/String) Latched             │
│    Publisher: robot_state_publisher                       │
│    Subscriber: RViz                                       │
│    Data: URDF XML describing robot structure              │
│                                                            │
│  /tf (tf2_msgs/TFMessage) ~100 Hz                         │
│    Publishers: Gazebo, SLAM Toolbox                       │
│    Subscribers: RViz, SLAM Toolbox                        │
│    Data: All coordinate frame transformations             │
│                                                            │
└───────────────────────────────────────────────────────────┘
🎮 What You Control
Direct Control:
Robot Movement - Through teleop keyboard
w/x - Forward/backward
a/d - Turn left/right
s - Stop
Indirect Control:
Where Robot Explores - You drive it around
Map Coverage - More driving = more map
When to Save - Call map_saver_cli when done
👁️ What You See
In Gazebo Window:
3D world with walls/obstacles
Turtlebot3 robot (blue/white)
Robot moves as you press keys
In RViz Window:
Left side (Displays panel):

Checkboxes to show/hide elements
Expand to adjust colors, sizes
Center (3D View):

Gray grid - Floor reference
Black/white map - Walls (black) and free space (white)
Starts all gray (unknown)
Fills in as robot explores
Red dots - Current laser scan (360 points)
Shows what LiDAR sees RIGHT NOW
Updates 5 times per second
Blue robot - Turtlebot3 3D model
Position matches reality (via TF)
Wheels rotate as it moves
RGB axes - Coordinate frames (if enabled)
Red = X axis, Green = Y axis, Blue = Z axis
Top toolbar:

2D Nav Goal - Click to set destination (needs Nav2)
2D Pose Estimate - Click to correct robot position
Camera controls to zoom/rotate view
🔍 How to Inspect Everything
See all running nodes:

ros2 node list
Expected output:


/gz_sim
/robot_state_publisher
/rviz2
/slam_toolbox
See all topics:

ros2 topic list
Expected output:


/cmd_vel
/map
/odom
/robot_description
/scan
/tf
...
See data flowing on a topic:

# See laser scan data
ros2 topic echo /scan --use-sim-time

# See movement commands
ros2 topic echo /cmd_vel

# See robot position
ros2 topic echo /odom --use-sim-time
Check topic frequency:

ros2 topic hz /scan --use-sim-time    # Should be ~5 Hz
ros2 topic hz /odom --use-sim-time    # Should be ~50 Hz
ros2 topic hz /map --use-sim-time     # Should be ~1 Hz
Visualize TF tree:

ros2 run tf2_tools view_frames
evince frames.pdf  # Opens diagram
See SLAM parameters:

ros2 param list /slam_toolbox
ros2 param get /slam_toolbox base_frame  # Returns: base_footprint
🧠 How SLAM Works (Simplified)
Every scan cycle (5 Hz):
Get sensor data:

Read /scan: "I see walls at [0.5m, 0.6m, 0.7m, ...]"
Read /odom: "I'm at position (1.2, 0.5)"
Scan matching:

Compare new scan with existing map
"This scan looks like the hallway I saw earlier"
Calculate: "I'm probably actually at (1.15, 0.48)" (correcting odom drift)
Update map:

For each laser point, mark corresponding map cell
Point hits something → Mark cell as "occupied" (black)
Beam passes through → Mark cells as "free" (white)
Loop closure:

"Wait, this place looks familiar!"
Recognize we've been here before
Correct accumulated drift
Adjust map→odom transform
Publish results:

Publish updated /map
Publish corrected map→odom transform
📊 Data Examples
LaserScan message structure:

header:
  stamp: {sec: 1234, nanosec: 567890000}
  frame_id: "base_scan"
angle_min: -3.14159  # -π radians (behind robot)
angle_max: 3.14159   # +π radians  
angle_increment: 0.0174  # ~1 degree
ranges: [0.5, 0.51, 0.52, inf, 2.1, ...]  # 360 measurements
  # inf = no detection (too far or no object)
Odometry message structure:

header:
  stamp: {sec: 1234, nanosec: 567890000}
  frame_id: "odom"
pose:
  position: {x: 1.2, y: 0.5, z: 0.0}
  orientation: {x: 0, y: 0, z: 0.15, w: 0.99}  # Quaternion
twist:
  linear: {x: 0.22, y: 0, z: 0}   # Current velocity
  angular: {x: 0, y: 0, z: 0.5}
Map (OccupancyGrid) structure:

header:
  frame_id: "map"
info:
  resolution: 0.05  # 5cm per cell
  width: 384        # 384 cells wide
  height: 384       # 384 cells tall
  origin: {x: -9.6, y: -9.6, z: 0}  # Bottom-left corner
data: [0, 0, 0, 100, 100, -1, ...]  # Flattened grid
  # 0 = free, 100 = occupied, -1 = unknown
⚙️ Configuration Deep Dive
SLAM Toolbox knows which frames to use:

odom_frame: odom           # Where odometry is published
base_frame: base_footprint # Where robot center is
map_frame: map             # Where to publish map
How it finds transforms:

"Where is the robot relative to the map?"
→ Lookup: map → odom (SLAM provides this)
→ Lookup: odom → base_footprint (Gazebo provides this)
→ Multiply transforms = map → base_footprint
🎯 Summary: The Complete Loop

YOU press 'w' 
  → Teleop publishes /cmd_vel {linear.x: 0.22}
    → Gazebo receives command
      → Simulates wheels turning
        → Robot moves forward 0.22 m/s
          → LiDAR sensor spins
            → Publishes /scan {ranges: [0.5, 0.6, ...]}
              → SLAM receives scan + odom
                → Compares with existing map
                  → Updates map cells (walls = black)
                    → Publishes /map
                      → RViz receives map
                        → YOU SEE: Black walls appear on screen!
