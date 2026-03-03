# Complete Beginner's Learning Guide: gazebo_robot_nav

> A step-by-step explanation of every part of this project in plain language —
> how files, functions, topics, and data flow together.

---

## 0. Mental Model: The 5 Layers

Before touching a single file, understand the project as 5 stacked layers:

```
┌───────────────────────────────────────────────────────────┐
│  5. BEHAVIOR LAYER  — Behavior Tree (bt_leaves.py)        │
│     "Search → Locate → Navigate → repeat"                 │
├───────────────────────────────────────────────────────────┤
│  4. VISION LAYER    — YOLO + Depth (find_and_go,          │
│                        vision_obstacle_node,              │
│                        vision_detector_node)              │
│     "What do I see? Where is it in 3D space?"             │
├───────────────────────────────────────────────────────────┤
│  3. NAVIGATION LAYER — Nav2 stack                         │
│     "How do I drive to X without hitting anything?"       │
├───────────────────────────────────────────────────────────┤
│  2. MAPPING LAYER   — SLAM Toolbox                        │
│     "Where am I? What does the room look like?"           │
├───────────────────────────────────────────────────────────┤
│  1. SIMULATION LAYER — Gazebo + model.sdf + bridge        │
│     "Physics, sensors, the fake world"                    │
└───────────────────────────────────────────────────────────┘
```

---

## 1. Simulation Layer — Gazebo & model.sdf

### What Gazebo does

Gazebo is a physics simulator. It runs the world (`turtlebot3_house.world`) and the
robot (`model.sdf`). It produces **fake sensor readings** that are indistinguishable
from real hardware as far as the ROS2 code is concerned.

### How sensors are defined — model.sdf

`models/turtlebot3_waffle/model.sdf` defines three sensors:

```
Robot body (base_link)
│
├── base_scan (LiDAR)
│   type: gpu_lidar
│   range: 0.12 – 3.5 m
│   360° horizontal, 5 Hz
│   topic (Gazebo internal): /scan
│
├── camera_rgb_frame
│   type: camera (RGB)
│   resolution: 640 × 480 px, 5 Hz
│   HFOV: 1.02974 rad (~59°)
│   topic (Gazebo internal): /camera/image_raw
│
└── camera_rgb_optical_frame (depth sensor, same link)
    type: depth_camera
    resolution: 320 × 240 px, 10 Hz
    range: 0.1 – 10.0 m
    encoding: R_FLOAT32 (32-bit float metres per pixel)
    topic (Gazebo internal): /camera/depth
```

### The bridge problem

Gazebo topics live in a **Gazebo-internal transport** system, not in ROS2. The bridge
is what converts between them.

```
Gazebo Simulator world
├── /scan              ──[gz_ros_bridge]──► ROS2 /scan        (LaserScan)
├── /odom              ──[gz_ros_bridge]──► ROS2 /odom        (Odometry)
├── /imu               ──[gz_ros_bridge]──► ROS2 /imu         (Imu)
├── /tf                ──[gz_ros_bridge]──► ROS2 /tf          (TFMessage)
│
├── /camera/image_raw  ──[image_bridge]──► ROS2 /camera/image_raw  (Image)
└── /camera/depth      ──[image_bridge]──► ROS2 /camera/depth      (Image)
```

`gz_ros_bridge` handles most topics via a YAML config file. `image_bridge` is separate
because camera images need a specialized image transport protocol — they are large and
benefit from a dedicated transport layer.

---

## 2. What is a ROS2 Topic?

**A topic is a named message channel.** Any node can publish to it. Any node can
subscribe to it. Publishers and subscribers don't know about each other directly —
they just agree on the topic name and message type.

```
Publisher ──[message]──► /topic/name ──[message]──► Subscriber A
                                     └──[message]──► Subscriber B
```

### Message types used in this project

| Message Type | What it contains | Example use |
|---|---|---|
| `sensor_msgs/LaserScan` | Array of 360 distance readings (floats) | LiDAR data |
| `sensor_msgs/Image` | Raw pixel buffer + metadata | Camera frames |
| `sensor_msgs/PointCloud2` | Array of 3D points (x,y,z) | Vision obstacles |
| `geometry_msgs/TwistStamped` | linear + angular velocity | Drive commands |
| `geometry_msgs/PoseStamped` | Position + orientation in a frame | Navigation goals |
| `nav_msgs/Odometry` | Robot's estimated position + velocity | Where robot thinks it is |
| `nav_msgs/OccupancyGrid` | 2D map as a grid of 0-100 values | The built map |
| `tf2_msgs/TFMessage` | Transform between two coordinate frames | "Where is camera relative to map?" |

### Sample: LaserScan message (what `/scan` looks like)

```yaml
header:
  stamp: {sec: 1234, nanosec: 567000000}
  frame_id: "base_scan"

angle_min: -3.14159    # radians, start of scan
angle_max:  3.14159    # radians, end of scan
angle_increment: 0.01745  # ~1 degree per reading

ranges: [
  0.35, 0.36, 0.38, ...,  # 360 readings, one per degree
  inf, inf, inf,           # inf = no return (beyond max range)
  1.20, 1.21, ...
]
                           # each float = distance in metres
```

### Sample: Image message (what `/camera/image_raw` looks like)

```yaml
header:
  stamp: {sec: 1234, nanosec: 0}
  frame_id: "camera_rgb_optical_frame"

height: 480
width:  640
encoding: "bgr8"    # Blue-Green-Red, 8 bits each
step: 1920          # bytes per row (640 * 3)
data: [...]         # raw bytes, 640*480*3 = 921,600 bytes total
```

### Sample: TwistStamped message (what `/cmd_vel` looks like)

This is what the controller sends to drive the robot. `linear.x` moves it forward or
backward. `angular.z` turns it left or right. All other fields are always zero for a
two-wheeled ground robot.

```yaml
header:
  stamp: {sec: 1234, nanosec: 0}
  frame_id: "base_link"      # whose perspective this velocity is in
twist:
  linear:
    x:  0.22   # m/s — forward  (negative = backward)
    y:  0.0    # m/s — sideways (always 0 for differential drive)
    z:  0.0    # m/s — vertical (always 0)
  angular:
    x:  0.0    # rad/s — roll  (always 0)
    y:  0.0    # rad/s — pitch (always 0)
    z:  0.3    # rad/s — yaw   (positive = turn left / counter-clockwise)
```

A stopped robot:
```yaml
twist:
  linear:  {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}
```

### Sample: PoseStamped message (what a navigation goal looks like)

This is what `ComputeTargetPose` writes to the blackboard and what `NavigateToGoal`
sends to Nav2. It says "go to this position, facing this direction, in this frame."

```yaml
header:
  stamp: {sec: 1234, nanosec: 0}
  frame_id: "map"           # position is expressed in the map coordinate frame
pose:
  position:
    x:  0.49   # metres east of map origin
    y:  0.23   # metres north of map origin
    z:  0.0    # always 0 for a ground robot
  orientation:             # quaternion — encodes a rotation angle
    x:  0.0
    y:  0.0
    z:  0.231  # sin(yaw/2) — yaw = 0.47 rad ≈ 27° counter-clockwise from east
    w:  0.973  # cos(yaw/2)
```

> **Quaternion tip:** For a 2D robot you only need the `z` and `w` components.
> A robot facing "east" (0°) has `z=0, w=1`. Facing "north" (90°) has `z=0.707, w=0.707`.

### Sample: Odometry message (what `/odom` looks like)

Odometry is the robot's best estimate of where it is, computed purely from wheel
encoder ticks. It drifts over time (wheels slip, encoders aren't perfect), which is
why SLAM is needed to correct it against LiDAR scans.

```yaml
header:
  stamp: {sec: 1234, nanosec: 0}
  frame_id: "odom"           # position is in the odom frame
child_frame_id: "base_footprint"

pose:
  pose:
    position:
      x:  1.35   # metres from where the robot started (odom origin)
      y: -0.22
      z:  0.0
    orientation:
      x:  0.0
      y:  0.0
      z:  0.156  # currently facing ~18° left of starting direction
      w:  0.988
  covariance: [0.001, 0, 0, 0, 0, 0,   # 6×6 uncertainty matrix (flattened)
               0, 0.001, 0, 0, 0, 0,   # diagonal values = position uncertainty
               0, 0, 0, 0, 0, 0,       # in metres² and rad²
               0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0.001]

twist:
  twist:
    linear:  {x: 0.18, y: 0.0, z: 0.0}   # current velocity (m/s)
    angular: {x: 0.0,  y: 0.0, z: 0.06}  # current turn rate (rad/s)
  covariance: [...]   # same 6×6 pattern for velocity uncertainty
```

### Sample: Imu message (what `/imu` looks like)

The IMU (Inertial Measurement Unit) measures raw acceleration and rotation rate. In
this project it runs at 200 Hz. SLAM Toolbox and Nav2 can use it to improve estimates
between slower LiDAR updates.

```yaml
header:
  stamp: {sec: 1234, nanosec: 5000000}
  frame_id: "imu_link"

orientation:             # estimated absolute orientation (if sensor provides it)
  x:  0.0
  y:  0.0
  z:  0.156
  w:  0.988
orientation_covariance: [0.0, 0.0, 0.0,   # 3×3, row-major
                         0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0]   # all-zero = "not provided"

angular_velocity:        # gyroscope — how fast each axis is rotating (rad/s)
  x:  0.0001             # rolling  (should be ~0 on flat ground)
  y: -0.0002             # pitching (should be ~0 on flat ground)
  z:  0.0612             # yawing   (turning left at 0.06 rad/s)
angular_velocity_covariance: [0.0001, 0, 0,
                              0, 0.0001, 0,
                              0, 0, 0.0001]

linear_acceleration:     # accelerometer — force per unit mass on each axis (m/s²)
  x:  0.08               # tiny forward acceleration
  y: -0.02               # tiny sideways (centripetal from turning)
  z:  9.82               # gravity! always ~9.81 m/s² downward
linear_acceleration_covariance: [0.01, 0, 0,
                                 0, 0.01, 0,
                                 0, 0, 0.01]
```

### Sample: TFMessage (what `/tf` looks like)

A single `/tf` message can carry multiple transforms at once. The TF system
broadcasts these continuously so any node can look up any frame relationship.

```yaml
transforms:
  - header:
      stamp: {sec: 1234, nanosec: 0}
      frame_id: "odom"           # parent frame
    child_frame_id: "base_footprint"   # child frame
    transform:
      translation: {x: 1.35, y: -0.22, z: 0.0}
      rotation:    {x: 0.0,  y: 0.0,   z: 0.156, w: 0.988}

  - header:
      stamp: {sec: 1234, nanosec: 0}
      frame_id: "base_footprint"
    child_frame_id: "base_link"
    transform:
      translation: {x: 0.0, y: 0.0, z: 0.01}   # base_link is 1cm above ground
      rotation:    {x: 0.0, y: 0.0, z: 0.0, w: 1.0}   # no rotation

  - header:
      stamp: {sec: 1234, nanosec: 0}
      frame_id: "base_link"
    child_frame_id: "camera_rgb_frame"
    transform:
      translation: {x: 0.064, y: -0.065, z: 0.094}   # from model.sdf
      rotation:    {x: 0.0,   y: 0.0,    z: 0.0, w: 1.0}
```

> `/tf` carries dynamic transforms (things that move, like odom → base_footprint).
> `/tf_static` carries fixed transforms (things bolted together, like base_link → camera).

### Sample: OccupancyGrid message (what `/map` looks like)

This is the full SLAM-built map. Each integer cell value means:
- `-1` = unknown (not yet scanned by LiDAR)
- `0` = free space (LiDAR passed through here without hitting anything)
- `1–99` = partially occupied (rarely used)
- `100` = definitely occupied (wall or obstacle)

```yaml
header:
  stamp: {sec: 1234, nanosec: 0}
  frame_id: "map"

info:
  map_load_time: {sec: 0, nanosec: 0}
  resolution: 0.05       # each cell = 5cm × 5cm in the real world
  width:  400            # cells horizontally
  height: 400            # cells vertically
  origin:                # where cell [0,0] is in real-world coordinates
    position:    {x: -10.0, y: -10.0, z: 0.0}
    orientation: {x: 0.0,  y: 0.0,   z: 0.0, w: 1.0}

data: [                  # flat array, width × height = 160,000 integers
  -1, -1, -1, -1,        # top-left corner — never scanned yet
  -1,  0,  0,  0,        # free space begins
   0,  0, 100, 100,      # wall starts
  100, 100, 100, -1,
  ...
]
```

To find what real-world (x, y) a cell at index `i` represents:
```
col = i % width                        # = 2
row = i // width                       # = 1
real_x = origin.x + col * resolution  # = -10.0 + 2 * 0.05 = -9.9 m
real_y = origin.y + row * resolution  # = -10.0 + 1 * 0.05 = -9.95 m
```

### Sample: Path message (what `/plan` looks like)

This is the global path the `planner_server` computes. It is a sequence of
`PoseStamped` waypoints from the robot's current position to the goal. The
`controller_server` then steers the robot along this path in real time.

```yaml
header:
  stamp: {sec: 1234, nanosec: 0}
  frame_id: "map"

poses:
  - header: {frame_id: "map"}
    pose:
      position:    {x: -0.95, y: -0.51, z: 0.0}   # waypoint 1 — current position
      orientation: {x: 0.0,  y: 0.0,   z: 0.156, w: 0.988}

  - header: {frame_id: "map"}
    pose:
      position:    {x: -0.80, y: -0.45, z: 0.0}   # waypoint 2 — step forward
      orientation: {x: 0.0,  y: 0.0,   z: 0.231, w: 0.973}

  # ... typically 20–100 waypoints depending on path length ...

  - header: {frame_id: "map"}
    pose:
      position:    {x: 0.49, y: 0.23, z: 0.0}     # last waypoint — the goal
      orientation: {x: 0.0, y: 0.0,  z: 0.231, w: 0.973}
```

### Sample: PointCloud2 message (what `/vision_obstacles` looks like)

A PointCloud2 is a flat binary blob of 3D points. Each point is 12 bytes
(3 × float32: x, y, z). The `fields` array describes the layout.

```yaml
header:
  stamp: {sec: 1234, nanosec: 0}
  frame_id: "camera_rgb_frame"   # all points are in camera coordinates

height: 1         # unordered cloud (1 row = flat list of points)
width:  847       # number of points in this cloud

fields:
  - name: "x"  offset: 0   datatype: 7  count: 1   # FLOAT32 = type 7
  - name: "y"  offset: 4   datatype: 7  count: 1
  - name: "z"  offset: 8   datatype: 7  count: 1

is_bigendian: false
point_step:  12    # bytes per point (3 floats × 4 bytes)
row_step:  10164   # bytes per row (847 points × 12 bytes)
is_dense:  true    # no NaN or Inf values

data: [            # raw binary — each row of 4 bytes is one float32
  # Point 0: x=2.81, y=-0.15, z=0.02
  #   x: 0x40 0x33 0xD7 0x0A  → 2.81 as IEEE 754
  #   y: 0xBE 0x19 0x85 0x1E  → -0.15 as IEEE 754
  #   z: 0x3C 0x03 0xD7 0x0A  → 0.02 as IEEE 754
  64, 51, 215, 10,  190, 25, 133, 30,  60, 3, 215, 10,
  # Point 1: x=2.83, y=-0.13, z=0.04
  ...
]
```

Conceptually, if you decoded the binary, you would see a list like:
```
point[0]:  (x=2.81, y=-0.15, z=0.02)   # 2.81m forward, 0.15m right, 2cm up
point[1]:  (x=2.83, y=-0.13, z=0.04)
point[2]:  (x=2.80, y=-0.16, z=0.01)
...
point[846]: (x=2.79, y=-0.11, z=0.05)
```

### Sample: NavigateToPose action (what find_and_go sends to Nav2)

An **action** has three parts: Goal (sent once), Feedback (received repeatedly while
running), and Result (received once at the end).

**Goal** — sent by `NavigateToGoal.initialise()`:
```yaml
pose:                         # where to go
  header: {frame_id: "map"}
  pose:
    position:    {x: 0.49, y: 0.23, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.231, w: 0.973}
behavior_tree: ""             # empty = use Nav2's default BT
```

**Feedback** — sent repeatedly by Nav2 while driving (~1 Hz):
```yaml
current_pose:
  header: {frame_id: "map"}
  pose:
    position:    {x: -0.42, y: -0.18, z: 0.0}   # robot's current position
    orientation: {x: 0.0,  y: 0.0,   z: 0.231, w: 0.973}
navigation_time:  {sec: 5, nanosec: 210000000}   # time spent navigating so far
estimated_time_to_arrival: {sec: 4, nanosec: 0}  # how much longer
number_of_recoveries: 0        # how many times Nav2 had to recover from being stuck
distance_remaining: 1.45       # metres left to the goal
```

**Result** — sent once when Nav2 finishes:
```yaml
# The result message itself is empty for NavigateToPose.
# The status comes from the action goal status field:

status: 4    # GoalStatus values:
             #   1 = STATUS_ACCEPTED
             #   2 = STATUS_EXECUTING
             #   4 = STATUS_SUCCEEDED  ← NavigateToGoal checks for this
             #   5 = STATUS_CANCELED
             #   6 = STATUS_ABORTED
```

---

## 3. Mapping Layer — SLAM Toolbox

### What SLAM means

**SLAM = Simultaneous Localisation and Mapping.** The robot doesn't know the map and
doesn't know where it is — it figures out both at the same time using only sensor data.

### Data flow through SLAM Toolbox

```
Gazebo LiDAR sensor
      │
      │  (gz_ros_bridge translates)
      ▼
/scan  [sensor_msgs/LaserScan]  @ 5 Hz
      │
      ▼
  slam_toolbox_node
  (online_async_launch.py)
  config: slam_params_turtlebot3.yaml
      │
      ├──► /map  [nav_msgs/OccupancyGrid]
      │    (the growing 2D map, updated every few seconds)
      │
      └──► /tf   (map → odom transform)
           (tells Nav2 where the "odom" frame sits within the map)
```

### What the SLAM config controls

`config/slam_params_turtlebot3.yaml` sets:

- `solver_plugin: solver_plugins::CeresSolver` — math solver for loop closure
- `scan_topic: /scan` — which topic to listen to
- `odom_frame: odom`, `map_frame: map` — coordinate frame names
- `map_update_interval: 5.0` — seconds between full map updates

### The OccupancyGrid — what the map looks like

```yaml
nav_msgs/OccupancyGrid:

  resolution: 0.05  # each cell = 5cm × 5cm
  width: 400
  height: 400
  origin: {x: -10.0, y: -10.0}  # real-world position of cell [0,0]

  data: [
    -1,  -1,  -1,  -1,   # -1 = unknown (not yet scanned)
    -1,   0,   0,   0,   #  0 = free space
     0,   0, 100, 100,   # 100 = occupied (wall)
    100, 100, 100,  -1,
  ]
```

Visualised in RViz:

```
? ? ? ?     (grey  — unknown)
? . . .     (white — free)
. . █ █     (black — wall)
█ █ █ ?
```

---

## 4. TF2 — The Coordinate Frame System

### Why TF2 exists

The robot has many parts — map, odom, base_link, camera, wheels — each with its own
coordinate system. TF2 is a library that tracks the **transform (position + rotation)
between every pair of frames** over time.

### The TF tree in this project

```
map  ◄── published by SLAM Toolbox (map → odom)
 │
odom ◄── published by Gazebo wheel encoders (odom → base_footprint)
 │
base_footprint
 │
base_link
 ├── base_scan        (LiDAR: 0.0m x, 0.0m y, 0.2m z above base_link)
 ├── camera_rgb_frame (camera: 0.064m x, -0.065m y, 0.094m z)
 └── wheels (×2)
```

### What a transform looks like (example: base_link → camera_rgb_frame)

```yaml
TransformStamped:
  header.frame_id: "base_link"
  child_frame_id:  "camera_rgb_frame"
  transform:
    translation: {x: 0.064, y: -0.065, z: 0.094}  # metres
    rotation:    {x: 0.0, y: 0.0, z: 0.0, w: 1.0}  # quaternion (no rotation)
```

**Why does this matter?** When YOLO finds an object at pixel (320, 240) of the
camera, the robot needs to know where that is in the **map frame** to navigate there.
TF2 does that math automatically by chaining transforms:
`camera_rgb_frame → base_link → base_footprint → odom → map`.

---

## 5. Navigation Layer — Nav2

### The cmd_vel chain

Every step in this chain is a separate ROS2 node. This is the path a velocity command
takes from "Nav2 wants to move" all the way to the physical wheels.

```
find_and_go_node                    (sends NavigateToPose ACTION goal)
          │
          ▼
     bt_navigator                   (Nav2's internal BT orchestrator)
          │
    ┌─────┴─────────────────┐
    │                       │
planner_server         controller_server
(global path)          (local tracking)
    │                       │
    ▼                       ▼
/plan (Path)       cmd_vel_nav (TwistStamped)
                           │
                    velocity_smoother         (smooths jerky commands)
                           │
                    cmd_vel_smoothed (TwistStamped)
                           │
                    collision_monitor         (emergency braking)
                           │
                    /cmd_vel (TwistStamped)
                           │
                    gz_ros_bridge             (ROS2 → Gazebo)
                           │
                    Gazebo wheel actuators → robot moves
```

### Sample cmd_vel message (what drives the wheels)

```yaml
TwistStamped:
  header:
    stamp: {sec: 1234, nanosec: 0}
  twist:
    linear:
      x: 0.22   # forward speed in m/s  (positive = forward)
      y: 0.0    # lateral (wheeled robots = always 0)
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.3    # turning rate rad/s (positive = counter-clockwise / left)
```

### Nav2 costmaps

Nav2 maintains two costmaps — both are grid maps used for obstacle avoidance:

```
Global costmap:
  - Covers the whole known map
  - Updated slowly
  - Used by the planner to find a safe path around walls

Local costmap:
  - Small window around the robot (e.g. 3m × 3m)
  - Updated every sensor tick
  - Sources: /scan (LiDAR) + /vision_obstacles (camera detections)
  - Used by the controller to steer around nearby obstacles
```

**Why vision obstacles matter:** LiDAR detects objects only at beam height. A low table
or a person partially obscured might be seen by the camera first. The
`vision_obstacle_node` feeds camera detections into the costmap so Nav2 knows about
them before LiDAR confirms them.

---

## 6. Vision Layer — Camera, YOLO, Depth

### Step-by-step: how an image becomes a 3D position

#### Step 1 — Gazebo renders a camera frame

The simulated camera captures a 640×480 BGR frame at 5 Hz. Internally it is
transmitted via Gazebo's transport system.

#### Step 2 — image_bridge converts it to ROS2

```yaml
# ROS2 topic after bridge:
/camera/image_raw  [sensor_msgs/Image]
  encoding: "bgr8"
  width: 640,  height: 480
  data: [bytes of 307,200 pixels...]
```

#### Step 3 — vision_detector_node (simple YOLO demo)

`vision_detector_node.py` is the simplest node — subscribe, run YOLO, publish
annotated image:

```
/camera/image_raw ──► image_callback()
                           │
                      cv_bridge.imgmsg_to_cv2()   ← ROS Image → NumPy BGR array
                           │
                      YOLO('yolov8n.pt')(cv_image)
                           │
                      results[0].boxes  ← list of detected bounding boxes
                           │
                      for each box:
                        cls_name = model.names[int(box.cls)]  # e.g. "person"
                        conf = float(box.conf)                # e.g. 0.87
                           │
                      results[0].plot()  ← draws boxes on image
                           │
                      cv_bridge.cv2_to_imgmsg()   ← NumPy → ROS Image
                           │
                      /camera/detections_image  (viewable in RViz2)
```

**Sample YOLO output (bounding box data):**

```python
box.xyxy   → tensor([[124.3, 89.1, 298.7, 421.5]])  # x1,y1,x2,y2 in pixels
box.cls    → tensor([0.0])     # class index (0 = person in COCO dataset)
box.conf   → tensor([0.87])    # confidence: 87%
```

#### Step 4 — find_and_go_node: YOLO + Depth = 3D position

`find_and_go_node.py` does the same YOLO call but then uses the depth camera to find
the object's **3D position in camera space**.

**The `_get_3d_centroid_at_bbox()` function — step by step:**

```
Input: YOLO bounding box, e.g. x1=124, y1=89, x2=298, y2=421
       Depth image: 320×240, each pixel = float32 metres

Step 1: Scale bbox from RGB resolution (640×480) to depth resolution (320×240)
        sx = 320/640 = 0.5,  sy = 240/480 = 0.5
        depth_x1 = 124*0.5 = 62,  etc.

Step 2: Extract depth values for all pixels in the bbox region
        d_arr = depth_image[62:149, 45:210]  ← 2D NumPy slice

Step 3: Filter valid depth readings (0.1m < d < 10.0m, not NaN/inf)
        d_valid = d_arr[(d_arr > 0.1) & (d_arr < 10.0) & isfinite(d_arr)]

Step 4: Foreground filter — keep only the nearest 20% cluster
        (This strips the background wall behind the person)
        min_d = d_valid.min()               # e.g. 2.8m
        foreground = d_valid < 2.8 * 1.2   # keep d < 3.36m → just the person

Step 5: Back-project to 3D using the pinhole camera model
        Camera intrinsics (from model.sdf):
          fx = fy = 640 / (2 * tan(1.02974/2)) ≈ 589.4
          cx = 320.0,  cy = 240.0

        For each foreground pixel (u, v) with depth d:
          cam_x = d               ← forward distance (optical Z)
          cam_y = -(u - cx)*d/fx  ← left/right  (negative optical X)
          cam_z = -(v - cy)*d/fy  ← up/down     (negative optical Y)

Step 6: Return the median of all foreground pixels
        centroid = (median(cam_x), median(cam_y), median(cam_z))
                 = (2.83, -0.12, 0.04)
                   ↑ 2.83m ahead, 0.12m to the right, 4cm up
```

**Coordinate frames — important:**

```
Camera optical frame (what the sensor uses natively):
  Z = forward (into the image)
  X = right
  Y = down

ROS camera body frame (what TF2 and the rest of ROS uses):
  X = forward
  Y = left
  Z = up

The conversion used in code:
  cam_x = d           (depth Z  → ROS X, forward)
  cam_y = -X_optical  (flip sign for left/right)
  cam_z = -Y_optical  (flip sign for up/down)
```

---

## 7. Behavior Layer — The Behavior Tree

### What is a Behavior Tree?

A Behavior Tree (BT) is an alternative to a state machine for robot decision-making.
Every node in the tree returns one of three statuses:

- **SUCCESS** — task completed
- **FAILURE** — task failed, something went wrong
- **RUNNING** — task in progress, call me again next tick

### This project's tree structure

```
Retry(num_failures=-1)         ← retries forever on FAILURE
  └── Sequence(memory=True)    ← runs children left-to-right;
        │                         stops at the first non-SUCCESS
        ├── SearchForTarget    ← LEAF: spin + detect with YOLO
        ├── ComputeTargetPose  ← LEAF: convert camera coords → map goal
        └── NavigateToGoal     ← LEAF: send goal to Nav2
```

### How a Sequence unfolds over time

```
Tick 1:  Run SearchForTarget  → RUNNING (still searching, robot rotates)
         Sequence returns RUNNING, does not touch next children yet.

Tick 11: Run SearchForTarget  → SUCCESS (person found, centroid on blackboard)
         Sequence moves to ComputeTargetPose.

Tick 11: Run ComputeTargetPose → SUCCESS (goal pose computed, on blackboard)
         Sequence moves to NavigateToGoal.

Tick 11: Run NavigateToGoal → RUNNING (goal sent to Nav2, robot driving)
         memory=True: Sequence skips re-running SearchForTarget (important —
         prevents the robot from spinning away while navigating).

Ticks 12–100: NavigateToGoal → RUNNING
Tick 101:     NavigateToGoal → SUCCESS (arrived)
              Sequence returns SUCCESS → Retry restarts from SearchForTarget.
```

### The Blackboard — shared memory between BT leaves

The blackboard is a key-value store. Leaves communicate through it rather than calling
each other directly.

```
SearchForTarget  ──writes──► blackboard["centroid"]  ──reads──► ComputeTargetPose
                                                                       │
                                                                    writes
                                                                       ▼
                                                             blackboard["goal_pose"]
                                                                  ──reads──► NavigateToGoal
```

**Sample blackboard values:**

```python
blackboard["centroid"] = (2.83, -0.12, 0.04)
# (cam_x=2.83m forward, cam_y=-0.12m right, cam_z=0.04m up)
# coordinate frame: camera_rgb_frame

blackboard["goal_pose"] = PoseStamped(
    frame_id="map",
    position=Point(x=1.45, y=0.87, z=0.0),
    orientation=Quaternion(z=0.31, w=0.95)  # yaw ~36°
)
```

### ComputeTargetPose — the geometry math

`bt_leaves.py` `ComputeTargetPose.update()` does this:

```
Step 1: Read centroid from blackboard
        centroid = (2.83, -0.12, 0.04)  ← camera_rgb_frame

Step 2: Wrap as PointStamped, ask TF2 to transform to "map" frame
        cam_point.header.frame_id = "camera_rgb_frame"
        map_point = tf_buffer.transform(cam_point, "map")
        → map_point = (1.82, 0.91)  ← now in world coordinates

Step 3: Look up robot's current position in the "map" frame
        tf = tf_buffer.lookup_transform("map", "base_footprint", ...)
        robot = (-0.95, -0.51)

Step 4: Compute angle and distance to the object
        dx = 1.82 − (−0.95) = 2.77
        dy = 0.91 − (−0.51) = 1.42
        target_angle = atan2(1.42, 2.77) ≈ 0.47 rad (27°)
        full_dist = sqrt(2.77² + 1.42²) ≈ 3.12m

Step 5: Stop stop_distance (1.5m) short of the object
        nav_dist = 3.12 − 1.5 = 1.62m
        goal_x = −0.95 + 1.62 * cos(0.47) ≈ 0.49
        goal_y = −0.51 + 1.62 * sin(0.47) ≈ 0.23

Step 6: Build PoseStamped and write to blackboard["goal_pose"]
        pose.position    = (0.49, 0.23, 0.0) in map frame
        pose.orientation = yaw quaternion at 0.47 rad
```

---

## 8. The Vision Obstacle Node — Parallel Protection

`vision_obstacle_node.py` runs **in parallel** with `find_and_go_node` at all times.
It does not use the BT — it reacts to every camera frame independently.

```
Every camera frame (5 Hz):
  /camera/image_raw ──► image_cb()
  /camera/depth     ──► depth_cb()   (stored for use in image_cb)
          │
          ▼
      YOLO detects all objects (not just "person")
          │
          ▼
      For each detection above confidence threshold:
        _backproject_bbox() → list of (x,y,z) 3D points in camera_rgb_frame
          │
          ▼
      All points merged → PointCloud2 message
          │
          ▼
      /vision_obstacles [sensor_msgs/PointCloud2] published

Nav2 local costmap obstacle_layer subscribes to /vision_obstacles:
  Each (x,y,z) point is transformed to map frame via TF2
  and marked as occupied in the costmap grid.
```

**What a PointCloud2 message looks like (conceptually):**

```yaml
PointCloud2:
  header.frame_id: "camera_rgb_frame"
  height: 1
  width: 847           # number of 3D points
  point_step: 12       # bytes per point (3 × float32 = 12 bytes)
  fields: [x, y, z]   # each field is a 4-byte float32
  data: [              # raw binary: x,y,z for each of the 847 points
    # Point 1: (x=2.81, y=-0.15, z=0.02)
    # Point 2: ...
  ]
```

---

## 9. Full End-to-End Scenario: Robot Finds a Person

A complete trace of one "find and go" cycle from launch to arrival.

```
t=0.0s  Launch: ros2 launch gazebo_nav_bringup gazebo_slam_nav.launch.py
        │
        ├── Gazebo starts, loads turtlebot3_house.world
        ├── Robot spawned at (-2.0, -0.5) from model.sdf
        ├── gz_ros_bridge + image_bridge start translating Gazebo → ROS2
        ├── slam_toolbox starts building /map from /scan data
        ├── Nav2 nodes start; lifecycle_manager activates them
        ├── vision_obstacle_node starts listening to camera
        └── find_and_go_node starts, builds BT, starts 10 Hz tick timer

──────────────────────────── PHASE 1: SEARCHING ─────────────────────

t=0.5s  BT tick #5: SearchForTarget.update() called
        ├── detect_target() called
        ├── self._latest_image is None (no frame arrived yet) → return None
        └── SearchForTarget returns RUNNING

t=1.0s  First /camera/image_raw frame arrives from Gazebo
        └── _image_cb() stores it in self._latest_image

t=1.0s  First /camera/depth frame arrives
        └── _depth_cb() stores it in self.latest_depth

t=1.1s  BT tick #11: SearchForTarget.update()
        ├── detect_target() called — image available
        ├── YOLO runs on 640×480 BGR frame
        ├── results[0].boxes = []  (person not in view)
        ├── no_detect_count = 1 → < 10, no rotation yet
        └── returns RUNNING

t=2.1s  BT tick #21: no_detect_count = 11 > patience threshold (10)
        ├── _publish_rotation() called:
        │     TwistStamped → /cmd_vel
        │     twist.angular.z = 0.3 rad/s  (slow left rotation)
        └── returns RUNNING
        [Robot slowly rotates, scanning the room...]

──────────────────────────── PHASE 2: DETECTION ─────────────────────

t=4.5s  Camera sweeps to face a person at ~2.8m distance.
        YOLO results on this frame:
          box.xyxy = [[124, 89, 298, 421]]   # bounding box in pixels
          box.cls  = [0]                      # class 0 = "person"
          box.conf = [0.87]                   # 87% confidence

        _get_3d_centroid_at_bbox() runs:
          - Scales bbox to depth resolution: [62, 44, 149, 210]
          - Reads depth pixels in region: median ≈ 2.83m
          - Foreground filter: keeps pixels < 2.83 * 1.2 = 3.4m
          - Back-projects using pinhole model:
              centroid = (2.83, -0.12, 0.04) m  ← camera frame

        detect_target() returns (centroid=(2.83,-0.12,0.04), n_candidates=1)

t=4.5s  SearchForTarget.update():
        ├── bb.centroid = (2.83, -0.12, 0.04)   ← written to blackboard
        └── returns SUCCESS

        terminate() called → _stop_rotation()
          TwistStamped(all zeros) → /cmd_vel    ← robot stops spinning

──────────────────────── PHASE 3: COMPUTE GOAL POSE ─────────────────

t=4.5s  Sequence advances to ComputeTargetPose.update():
        │
        ├── Read bb.centroid = (2.83, -0.12, 0.04) from blackboard
        ├── Build PointStamped in camera_rgb_frame
        ├── tf_buffer.transform(..., "map")
        │     TF2 chains: camera_rgb_frame→base_link→base_footprint→odom→map
        │     → map_point = (1.82, 0.91)
        │
        ├── tf_buffer.lookup_transform("map", "base_footprint")
        │     → robot position in map = (-0.95, -0.51)
        │
        ├── dx=2.77, dy=1.42, angle=0.47 rad, full_dist=3.12m
        ├── full_dist (3.12m) > stop_dist (1.5m) → need to navigate
        ├── nav_dist = 3.12 - 1.5 = 1.62m
        ├── goal = (0.49, 0.23) in map frame, yaw = 0.47 rad
        │
        ├── bb.goal_pose = PoseStamped(map, 0.49, 0.23, ...)
        └── returns SUCCESS

──────────────────────── PHASE 4: NAVIGATE TO GOAL ──────────────────

t=4.5s  Sequence advances to NavigateToGoal.initialise():
        ├── Read bb.goal_pose from blackboard
        ├── nav_client.wait_for_server() → Nav2 available ✓
        └── nav_client.send_goal_async(NavigateToPose.Goal(pose=...))

        Nav2 bt_navigator receives the action goal:
          ├── planner_server computes global path:
          │     /plan = [pose1, pose2, ..., pose47]  (waypoints to goal)
          ├── controller_server (RegulatedPurePursuit) follows path:
          │     cmd_vel_nav:      linear.x=0.22, angular.z=0.08
          ├── velocity_smoother smooths the command:
          │     cmd_vel_smoothed: linear.x=0.18, angular.z=0.06
          └── collision_monitor passes it through (no obstacle nearby):
                /cmd_vel: linear.x=0.18, angular.z=0.06
                → gz_ros_bridge → Gazebo wheels → robot drives

t=4.5–14.2s  NavigateToGoal.update() called every 0.1s:
        ├── _result_future.done() == False → returns RUNNING
        └── (memory=True: Sequence stays here, doesn't re-run SearchForTarget)

t=14.2s  Robot arrives 1.5m from person.
         Nav2 reports STATUS_SUCCEEDED.
         NavigateToGoal.update() → returns SUCCESS.

         Sequence returns SUCCESS → Retry restarts the whole tree.

t=14.3s  SearchForTarget.initialise() resets no_detect_count = 0.
         New cycle begins: is the person still there? Did they move?
```

---

## 10. Node-Topic-Node Full Map

```
┌─────────────────────────────────────────────────────────────────────┐
│                          GAZEBO SIMULATOR                           │
│  Physics + turtlebot3_house.world + robot (from model.sdf)          │
│                                                                     │
│  Sensors:  [LiDAR]──/scan   [RGB cam]──/camera/image_raw            │
│            [Depth]──/camera/depth   [Wheels]◄──/cmd_vel             │
│            [IMU]──/imu   [Odometry]──/odom                          │
└──────────┬──────────────────────────────────────────────────────────┘
           │ gz_ros_bridge                    image_bridge
           │ (converts most topics)           (converts camera images)
           ▼                                           ▼
    /scan (LaserScan)             /camera/image_raw (Image, 640×480 bgr8)
    /odom (Odometry)              /camera/depth    (Image, 320×240 float32)
    /tf   (TFMessage)
    /imu  (Imu)
           │
    ┌──────┴───────────────────────────────────────────────────┐
    │                    slam_toolbox_node                     │
    │  input:  /scan, /odom, /tf                               │
    │  output: /map (OccupancyGrid)                            │
    │          /tf  (map → odom transform)                     │
    └──────────────────────────────────────────────────────────┘
           │
    ┌──────┴───────────────────────────────────────────────────┐
    │                      Nav2 stack                          │
    │  planner_server    ← /map + goal → /plan (Path)          │
    │  controller_server ← /plan + /odom → cmd_vel_nav         │
    │  velocity_smoother ← cmd_vel_nav → cmd_vel_smoothed      │
    │  collision_monitor ← cmd_vel_smoothed + /scan → /cmd_vel │
    │  bt_navigator      ← NavigateToPose ACTION               │
    │  local costmap     ← /scan + /vision_obstacles           │
    └──────────────────────────────────────────────────────────┘
                              ▲ NavigateToPose action
                              │
    ┌─────────────────────────┴────────────────────────────────┐
    │                   find_and_go_node                       │
    │  Behavior Tree ticked at 10 Hz                           │
    │                                                          │
    │  inputs:  /camera/image_raw                              │
    │           /camera/depth                                  │
    │           /tf (via tf2_ros.Buffer)                       │
    │                                                          │
    │  outputs: /cmd_vel  (TwistStamped) ← search rotation     │
    │           /camera/detections_image ← debug view          │
    │           NavigateToPose action goal ← to Nav2           │
    └──────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────┐
    │          vision_obstacle_node  (runs in parallel)       │
    │  inputs:  /camera/image_raw, /camera/depth              │
    │  output:  /vision_obstacles (PointCloud2)               │
    │           → consumed by Nav2 local costmap              │
    └─────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────┐
    │       vision_detector_node  (standalone demo)           │
    │  input:  /camera/image_raw                              │
    │  output: /camera/detections_image                       │
    └─────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────┐
    │                     RViz2                               │
    │  Subscribes to: /map, /scan, /tf,                       │
    │  /camera/image_raw, /camera/detections_image,           │
    │  /plan, /vision_obstacles                               │
    │  (visualisation only — no output topics)                │
    └─────────────────────────────────────────────────────────┘
```

---

## 11. All Topics at a Glance

### 11a. Topic Flow in Scenario Order (publisher → subscriber)

Topics listed in the order they come alive during a real "find and go" run,
from the first sensor tick to the final wheel command.

```
══════════════════════════════════════════════════════════════
 PHASE 1 — SIMULATION PRODUCES SENSOR DATA
══════════════════════════════════════════════════════════════

 Gazebo physics engine (model.sdf sensors fire every tick)
    │
    ├─[gz_ros_bridge]─► /scan          LaserScan    → slam_toolbox
    │                                               → Nav2 costmaps
    │                                               → collision_monitor
    │
    ├─[gz_ros_bridge]─► /odom          Odometry     → slam_toolbox
    │                                               → Nav2 (controller)
    │
    ├─[gz_ros_bridge]─► /imu           Imu          → (available, not
    │                                                  actively used here)
    │
    ├─[gz_ros_bridge]─► /tf            TFMessage    → Everyone
    │   (Gazebo publishes wheel-encoder odom→base_footprint)
    │
    ├─[image_bridge]──► /camera/image_raw  Image    → vision_detector_node
    │                                               → vision_obstacle_node
    │                                               → find_and_go_node
    │
    └─[image_bridge]──► /camera/depth      Image    → vision_obstacle_node
                                                    → find_and_go_node

══════════════════════════════════════════════════════════════
 PHASE 2 — LOCALIZATION & STATIC FRAMES
══════════════════════════════════════════════════════════════

 robot_state_publisher (reads robot URDF/SDF joint angles)
    └─► /tf_static     TFMessage    → Everyone
        (base_link → camera_rgb_frame, base_link → base_scan, etc.)
        These never change — published once at startup.

 slam_toolbox  (consumes /scan + /odom)
    ├─► /map           OccupancyGrid → Nav2 planner_server
    │                                → Nav2 costmaps
    │                                → RViz2
    │
    └─► /tf            TFMessage     → Everyone
        (publishes the corrected map → odom transform,
         closing the loop: map → odom → base_footprint → base_link)

══════════════════════════════════════════════════════════════
 PHASE 3 — VISION: OBSTACLE AWARENESS (parallel, always on)
══════════════════════════════════════════════════════════════

 vision_obstacle_node  (consumes /camera/image_raw + /camera/depth)
    └─► /vision_obstacles  PointCloud2 → Nav2 local costmap obstacle_layer
        (YOLO detects objects → depth back-project → 3D obstacle points)

══════════════════════════════════════════════════════════════
 PHASE 4 — BEHAVIOR TREE: SEARCHING (target not yet found)
══════════════════════════════════════════════════════════════

 find_and_go_node  SearchForTarget leaf
  (consumes /camera/image_raw + /camera/depth → runs YOLO → no match yet)
    │
    ├─► /camera/detections_image  Image → RViz2  (annotated debug view)
    │
    └─► /cmd_vel   TwistStamped → gz_ros_bridge → Gazebo wheels
        angular.z = 0.3 rad/s  (slow rotation to scan the room)

══════════════════════════════════════════════════════════════
 PHASE 5 — BEHAVIOR TREE: TARGET FOUND → GOAL COMPUTED
══════════════════════════════════════════════════════════════

 find_and_go_node  SearchForTarget leaf
  (YOLO finds "person" + depth centroid computed → SUCCESS)
    │
    └─► /cmd_vel   TwistStamped → gz_ros_bridge
        all zeros  (stop rotation)

 find_and_go_node  ComputeTargetPose leaf
  (reads /tf via tf2_ros.Buffer — no topic publish, internal lookup)
  (transforms camera-frame centroid → map-frame goal PoseStamped)
  (result lives only on the BT blackboard — not published as a topic)

══════════════════════════════════════════════════════════════
 PHASE 6 — NAV2: PATH PLANNING
══════════════════════════════════════════════════════════════

 find_and_go_node  NavigateToGoal leaf
    └─► navigate_to_pose  ACTION GOAL  → bt_navigator (Nav2)
        (NavigateToPose goal: PoseStamped in map frame)

 bt_navigator  (Nav2's internal BT orchestrator, uses /map + /odom)
    └─calls─► planner_server

 planner_server  (Navfn — A* / Dijkstra on the OccupancyGrid)
    └─► /plan   Path  → controller_server
                      → RViz2  (the green path line)

══════════════════════════════════════════════════════════════
 PHASE 7 — NAV2: PATH FOLLOWING → WHEELS
══════════════════════════════════════════════════════════════

 controller_server  (RegulatedPurePursuit, consumes /plan + /odom)
    └─► cmd_vel_nav   TwistStamped → velocity_smoother

 velocity_smoother  (limits jerk / acceleration rate)
    └─► cmd_vel_smoothed   TwistStamped → collision_monitor

 collision_monitor  (safety: reads /scan, cuts speed near obstacles)
    └─► /cmd_vel   TwistStamped → gz_ros_bridge → Gazebo wheels
        ↑ same topic as Phase 4 — one topic, two publishers,
          only one active at a time (search spin OR Nav2 driving)

══════════════════════════════════════════════════════════════
 PHASE 8 — LOOP: ROBOT MOVES → NEW SENSOR DATA
══════════════════════════════════════════════════════════════

 Gazebo wheels receive /cmd_vel → robot moves in simulation
    └─► new /scan, /odom, /camera/* published → back to Phase 1

 navigate_to_pose  ACTION FEEDBACK  ← bt_navigator → find_and_go_node
    (distance_remaining, current_pose, navigation_time — ~1 Hz)

 When Nav2 finishes:
 navigate_to_pose  ACTION RESULT  ← bt_navigator → find_and_go_node
    status = STATUS_SUCCEEDED → BT Sequence returns SUCCESS
    → Retry restarts from SearchForTarget → back to Phase 4
```

> **Key insight:** `/cmd_vel` has two publishers (`SearchForTarget` and
> `collision_monitor`) but only one is ever active at a time. The BT enforces this —
> `SearchForTarget` sends zeros the moment a target is found, then hands Nav2 full
> control of the wheels.

---

### 11b. Quick-Reference Table (alphabetical)

| Topic | Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/camera/depth` | `Image` (float32) | image_bridge ← Gazebo depth | vision_obstacle_node, find_and_go_node |
| `/camera/detections_image` | `Image` (bgr8) | vision_detector_node OR find_and_go_node | RViz2 |
| `/camera/image_raw` | `Image` (bgr8) | image_bridge ← Gazebo RGB cam | vision_detector_node, vision_obstacle_node, find_and_go_node |
| `cmd_vel_nav` | `TwistStamped` | controller_server | velocity_smoother |
| `cmd_vel_smoothed` | `TwistStamped` | velocity_smoother | collision_monitor |
| `/cmd_vel` | `TwistStamped` | collision_monitor + find_and_go_node (search spin) | gz_ros_bridge → Gazebo wheels |
| `/imu` | `Imu` | gz_ros_bridge ← Gazebo IMU | (available; not actively used) |
| `/map` | `OccupancyGrid` | slam_toolbox | Nav2 planner, costmaps, RViz2 |
| `navigate_to_pose` (action) | `NavigateToPose` | find_and_go_node | bt_navigator |
| `/odom` | `Odometry` | gz_ros_bridge ← Gazebo wheels | slam_toolbox, Nav2 |
| `/plan` | `Path` | planner_server | controller_server, RViz2 |
| `/scan` | `LaserScan` | gz_ros_bridge ← Gazebo LiDAR | slam_toolbox, Nav2 costmaps, collision_monitor |
| `/tf` | `TFMessage` | Gazebo bridge + slam_toolbox (dynamic) | Everyone |
| `/tf_static` | `TFMessage` | robot_state_publisher (fixed joints) | Everyone |
| `/vision_obstacles` | `PointCloud2` | vision_obstacle_node | Nav2 local costmap |

---

## 12. Key Concepts Summary

| Concept | One-line definition | Where in this project |
|---|---|---|
| ROS2 node | A process that subscribes/publishes topics | Every `.py` file with a `Node` class |
| Topic | Named message channel (no memory, fire-and-forget) | `/scan`, `/cmd_vel`, etc. |
| Action | Request-response with feedback + cancellation support | `navigate_to_pose` |
| TF2 | Tracks position+rotation of every frame over time | Used in `ComputeTargetPose` |
| SLAM | Build map + locate self simultaneously | slam_toolbox |
| Costmap | Inflated obstacle grid used for path planning | Nav2 local + global costmap |
| Behavior Tree | Decision tree; each node returns SUCCESS/RUNNING/FAILURE | `find_and_go_node.py` + `bt_leaves.py` |
| Blackboard | Shared key-value memory for BT leaves | `centroid`, `goal_pose` |
| Pinhole camera model | Math to convert pixel + depth → 3D point | `_get_3d_centroid_at_bbox()` |
| Back-projection | Applying pinhole model to go pixel → 3D space | `_backproject_bbox()` in vision_obstacle |
| cv_bridge | Converts `sensor_msgs/Image` ↔ NumPy array | Every camera callback |

---

## 13. Suggested Learning Path

Follow these steps in order for the smoothest learning curve:

**1. Start with the minimal launch file**
Read `launch/gazebo_slam.launch.py`. This is the simplest stack: just Gazebo, SLAM,
and RViz2. Understand what each line launches and why the order matters (environment
variable must be set before Gazebo starts).

**2. Understand sensing**
Look at what `/scan` contains (LaserScan format in Section 2). Then read
`config/slam_params_turtlebot3.yaml`. How does SLAM Toolbox turn 360° laser readings
into a 2D grid map?

**3. Understand navigation**
Read `launch/gazebo_slam_nav.launch.py` focusing on the `cmd_vel` chain comment
(lines 137–141). Trace the full path: `cmd_vel_nav → cmd_vel_smoothed → cmd_vel`.

**4. Understand vision (simple)**
Read `vision_detector_node.py` — 55 lines, the simplest possible YOLO node.
Understand the subscribe → YOLO → publish pattern.

**5. Understand vision (with depth)**
Read `vision_obstacle_node.py`. Focus on `_backproject_bbox()` — this is the
pinhole camera math that converts pixels + depth into 3D points.

**6. Understand the full behavior**
Read `find_and_go_node.py` focusing on `_build_tree()` and `detect_target()`.
Then read each leaf in `bt_leaves.py` in order:
`SearchForTarget` → `ComputeTargetPose` → `NavigateToGoal`.

**7. Watch live data flow**
After building and launching the full stack, run these in a terminal:

```bash
ros2 topic echo /scan                    # watch LiDAR readings
ros2 topic echo /cmd_vel                 # watch velocity commands
ros2 topic echo /vision_obstacles --no-arr  # watch obstacle cloud headers
ros2 topic hz /camera/image_raw          # check camera publishing rate
ros2 topic list                          # see all active topics
```

---

## 14. Useful Commands for Exploration

```bash
# Build the package
cd ros2_ws && colcon build --symlink-install

# Launch SLAM only (manual mapping with teleop)
ros2 launch gazebo_nav_bringup gazebo_slam.launch.py

# Launch full stack (SLAM + Nav2 + vision + find-and-go)
ros2 launch gazebo_nav_bringup gazebo_slam_nav.launch.py

# Print the current TF tree to a PDF
ros2 run tf2_tools view_frames

# See all active topics and their types
ros2 topic list -t

# Watch a topic live (Ctrl+C to stop)
ros2 topic echo /scan
ros2 topic echo /cmd_vel

# Check how fast a topic is publishing
ros2 topic hz /camera/image_raw

# See all active nodes
ros2 node list

# Inspect a node's subscriptions and publications
ros2 node info /find_and_go
```

---

## 15. Complete Cycle: First Signal to Last Actuator

A full walkthrough of every step from sensor to wheel, with the exact file, function,
and data at each hop. This is the same system from Section 9 but told at the
implementation level instead of the scenario level.

---

### Step 1 — Gazebo fires sensors

**Source:** physics engine, sensor definitions in
[models/turtlebot3_waffle/model.sdf](ros2_ws/src/gazebo_nav_bringup/models/turtlebot3_waffle/model.sdf)

No ROS code runs here. Gazebo internally simulates physics and produces raw sensor
streams. The bridge in the next step translates them.

---

### Step 2 — Bridge translates Gazebo → ROS topics

**Nodes:** `gz_ros_bridge` and `image_bridge`
**Launched at:** [gazebo_slam_nav.launch.py:108–121](ros2_ws/src/gazebo_nav_bringup/launch/gazebo_slam_nav.launch.py#L108)
External packages — no custom code.

**`/scan`** — LaserScan
```yaml
header: {frame_id: "base_scan"}
angle_min: -3.14159    # full 360°
angle_max:  3.14159
angle_increment: 0.017 # ~1° per beam
range_min: 0.12
range_max: 3.5
ranges: [0.45, 0.46, 0.47, ..., 3.5, 3.5, 1.2, ...]
        # 360 floats — distance per beam, 3.5 = beyond range
```

**`/odom`** — Odometry
```yaml
header: {frame_id: "odom"}
child_frame_id: "base_footprint"
pose.pose.position:    {x: -1.95, y: -0.48, z: 0.0}
pose.pose.orientation: {z: 0.0, w: 1.0}   # facing east
```

**`/tf`** — odom → base_footprint (wheel-encoder dead reckoning)
```yaml
transforms[0]:
  header.frame_id: "odom"
  child_frame_id:  "base_footprint"
  transform.translation: {x: -1.95, y: -0.48, z: 0.0}
  transform.rotation:    {z: 0.0, w: 1.0}
```

**`/camera/image_raw`** — Image (640×480 bgr8)
```yaml
header: {frame_id: "camera_rgb_frame"}
height: 480  width: 640  encoding: "bgr8"
data: [...]   # 921,600 bytes of raw pixel values
```

**`/camera/depth`** — Image (320×240 float32)
```yaml
header: {frame_id: "camera_rgb_frame"}
height: 240  width: 320  encoding: "32FC1"
data: [...]   # each 4 bytes = one float32 depth in metres
```

---

### Step 3 — robot_state_publisher broadcasts static frames

**Node:** `robot_state_publisher`
**Launched at:** [gazebo_slam_nav.launch.py:83–88](ros2_ws/src/gazebo_nav_bringup/launch/gazebo_slam_nav.launch.py#L83)
External package. Publishes once at startup.

**`/tf_static`** — fixed joint positions (never change)
```yaml
# base_link → camera_rgb_frame
transform.translation: {x: 0.064, y: -0.065, z: 0.094}
transform.rotation:    {x: 0.0,   y: 0.0,    z: 0.0, w: 1.0}

# base_link → base_scan  (LiDAR position)
transform.translation: {x: -0.064, y: 0.0, z: 0.122}
```

---

### Step 4 — SLAM Toolbox builds the map

**Node:** `slam_toolbox`
**Launched at:** [gazebo_slam_nav.launch.py:125–133](ros2_ws/src/gazebo_nav_bringup/launch/gazebo_slam_nav.launch.py#L125)
**Config:** [config/slam_params_turtlebot3.yaml](ros2_ws/src/gazebo_nav_bringup/config/slam_params_turtlebot3.yaml)
External package.

**Reads:** `/scan` + `/odom`

**Publishes `/map`** — OccupancyGrid
```yaml
info.resolution: 0.05    # 5 cm per cell
info.width:  400
info.height: 400
data: [-1, -1, 0, 0, 100, 100, ...]
      # -1=unknown   0=free space   100=wall
```

**Publishes `/tf`** — map → odom correction (closes the TF chain)
```yaml
header.frame_id: "map"
child_frame_id:  "odom"
transform.translation: {x: 0.03, y: -0.01, z: 0.0}
# SLAM found the robot is slightly offset from raw odometry
```

Full TF chain is now complete:
`map → odom → base_footprint → base_link → camera_rgb_frame`

---

### Step 5 — find_and_go_node receives sensor data

**File:** [find_and_go_node.py](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/find_and_go_node.py)

Two callbacks run continuously via the ROS2 executor (independent of the BT timer):

**[`_depth_cb()` line 117](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/find_and_go_node.py#L117)**
- Receives `/camera/depth`
- `cv_bridge.imgmsg_to_cv2()` → NumPy float32 array, shape (240, 320)
- Stored in `self.latest_depth`

**[`_image_cb()` line 122](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/find_and_go_node.py#L122)**
- Receives `/camera/image_raw`
- Stores the raw ROS message in `self._latest_image` (decoding deferred to `detect_target()`)

---

### Step 6 — BT timer fires: SearchForTarget runs

Every 0.1 s, **[`_tick_tree()` line 266](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/find_and_go_node.py#L266)**
calls `self._tree.tick()`.

The BT walks to `SearchForTarget` and calls
**[`update()` line 71](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/bt_leaves.py#L71)**,
which calls
**[`detect_target()` line 127](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/find_and_go_node.py#L127)**:

```
_latest_image (raw ROS msg)
    │
    ├─ imgmsg_to_cv2() → cv_image  (NumPy BGR array, 480×640×3)
    │
    ├─ YOLO(cv_image) → results[0].boxes
    │     each box: xyxy=[120,80,310,420], cls=0 ("person"), conf=0.87
    │
    ├─ for each "person" box above threshold:
    │     _get_3d_centroid_at_bbox(box) →
    │         scale bbox 640→320 (depth resolution mismatch)
    │         read depth pixels in bbox: [2.79, 2.81, 2.83, ...]
    │         foreground filter: keep d < min_d * 1.2
    │         back-project: X=d, Y=-(u-cx)*d/fx, Z=-(v-cy)*d/fy
    │         median → centroid = (2.81, -0.08, 0.05) m
    │                              forward  left   up
    │
    └─ returns (centroid=(2.81, -0.08, 0.05), n_candidates=1)
```

Also publishes **`/camera/detections_image`** (annotated debug image → RViz):
```yaml
encoding: "bgr8"   # same 640×480 frame, with bounding boxes drawn on it
```

`SearchForTarget.update()` writes centroid to blackboard → returns **SUCCESS**.

---

### Step 7 — SearchForTarget stops rotation

**[`terminate()` line 96](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/bt_leaves.py#L96)**
→ **`_stop_rotation()`** via
**[`_node.cmd_pub.publish()` line 109](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/bt_leaves.py#L109)**:

**`/cmd_vel`** — TwistStamped (stop)
```yaml
twist.linear:  {x: 0.0, y: 0.0, z: 0.0}
twist.angular: {x: 0.0, y: 0.0, z: 0.0}
```

> If no target was found this tick, `_publish_rotation()` (line 102) sends
> `angular.z = 0.3` instead — the slow scan rotation.

---

### Step 8 — ComputeTargetPose transforms centroid to map frame

**[`update()` line 134](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/bt_leaves.py#L134)**:

```
blackboard.centroid = (2.81, -0.08, 0.05)
    │
    ├─ build PointStamped in "camera_rgb_frame": point={x:2.81, y:-0.08, z:0.05}
    │
    ├─ tf_buffer.transform(cam_point, "map")        ← line 145
    │     chain: camera_rgb_frame→base_link→base_footprint→odom→map
    │     result: map_point = {x: -0.94, y: -1.21}
    │
    ├─ tf_buffer.lookup_transform("map","base_footprint")   ← line 159
    │     robot is at map: {x: -1.95, y: -0.48}
    │
    ├─ dx=1.01, dy=-0.73, full_dist=1.25m, target_angle=-0.63 rad
    │
    ├─ (assume full_dist > stop_dist for this example)
    │   nav_dist = full_dist - stop_dist
    │   goal_x = robot_x + nav_dist * cos(angle)
    │   goal_y = robot_y + nav_dist * sin(angle)
    │
    └─ blackboard.goal_pose = PoseStamped:
         header.frame_id: "map"
         position: {x: -0.94, y: -1.24, z: 0.0}
         orientation: {z: -0.310, w: 0.951}   # facing toward the person
```

Returns **SUCCESS**.

---

### Step 9 — NavigateToGoal sends action to Nav2

**[`initialise()` line 233](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/bt_leaves.py#L233)**
runs once (first tick of this leaf):

**`navigate_to_pose` ACTION GOAL** → `bt_navigator`
```yaml
pose:
  header.frame_id: "map"
  pose.position:    {x: -0.94, y: -1.24, z: 0.0}
  pose.orientation: {z: -0.310, w: 0.951}
behavior_tree: ""    # empty = use Nav2's default BT
```

**[`update()` line 262](ros2_ws/src/gazebo_nav_bringup/gazebo_nav_bringup/bt_leaves.py#L262)**
polls `_result_future` each tick → returns **RUNNING** while Nav2 drives.

---

### Step 10 — Nav2 plans a path

**Node:** `bt_navigator` → calls `planner_server`
**Config:** [config/nav2_params.yaml](ros2_ws/src/gazebo_nav_bringup/config/nav2_params.yaml)
External package.

`planner_server` (Navfn/A*) reads `/map` + robot pose → publishes **`/plan`** — Path:
```yaml
header.frame_id: "map"
poses:
  - pose.position: {x: -1.95, y: -0.48}   # start — current robot position
  - pose.position: {x: -1.80, y: -0.55}   # step forward
  - pose.position: {x: -1.60, y: -0.68}
  # ... ~30 waypoints ...
  - pose.position: {x: -0.94, y: -1.24}   # goal
```

---

### Step 11 — controller_server follows the path

**Node:** `controller_server` (RegulatedPurePursuit algorithm)
**Launched at:** [gazebo_slam_nav.launch.py:142–149](ros2_ws/src/gazebo_nav_bringup/launch/gazebo_slam_nav.launch.py#L142)

Reads `/plan` + `/odom` → publishes **`cmd_vel_nav`** — TwistStamped:
```yaml
twist.linear.x:  0.22   # m/s forward
twist.angular.z: -0.15  # slight right turn to follow path curve
```

---

### Step 12 — velocity_smoother limits jerk

**Node:** `velocity_smoother`
**Launched at:** [gazebo_slam_nav.launch.py:196–203](ros2_ws/src/gazebo_nav_bringup/launch/gazebo_slam_nav.launch.py#L196)

Reads `cmd_vel_nav`, applies acceleration ramp → publishes **`cmd_vel_smoothed`**:
```yaml
twist.linear.x:  0.18   # ramping up from 0 — still accelerating
twist.angular.z: -0.14
```

---

### Step 13 — collision_monitor applies safety cutoff

**Node:** `collision_monitor`
**Launched at:** [gazebo_slam_nav.launch.py:205–212](ros2_ws/src/gazebo_nav_bringup/launch/gazebo_slam_nav.launch.py#L205)

Reads `cmd_vel_smoothed` + `/scan`. If LiDAR sees an obstacle too close, it scales
down the velocity. Otherwise passes through unchanged.

Publishes **`/cmd_vel`** — TwistStamped (the final wheel command):
```yaml
twist.linear.x:  0.18   # passes through — no obstacles nearby
twist.angular.z: -0.14
```

---

### Step 14 — gz_ros_bridge drives the wheels

`gz_ros_bridge` reads `/cmd_vel` → sends to Gazebo's differential drive plugin →
**wheels spin → robot moves in simulation**.

---

### Step 15 — Loop closes

Robot has moved. Gazebo physics updates:
- LiDAR sees new scene → new `/scan` → **back to Step 2**
- Wheel encoders tick → new `/odom` → SLAM corrects map → `/tf` updates
- Camera sees new view → new `/camera/image_raw` + `/camera/depth`

`NavigateToGoal.update()` keeps returning **RUNNING** (because `memory=True` in the
Sequence, so SearchForTarget and ComputeTargetPose are NOT re-ticked during Nav2
driving — only NavigateToGoal is polled each tick).

When Nav2 finishes:
```
_result_future.result().status == STATUS_SUCCEEDED (4)
NavigateToGoal.update() → SUCCESS
Sequence → SUCCESS
Retry → resets all children → next tick starts from SearchForTarget again
```

---

### One-glance summary

```
model.sdf           → Gazebo physics
gz_ros_bridge       → /scan, /odom, /tf
image_bridge        → /camera/image_raw, /camera/depth
robot_state_pub     → /tf_static  (fixed joints, once at startup)
slam_toolbox        → /map, /tf   (map→odom correction, continuous)

find_and_go_node:
  _depth_cb()       ← /camera/depth        (store latest_depth)
  _image_cb()       ← /camera/image_raw    (store _latest_image)

BT tick (10 Hz):
  SearchForTarget   → detect_target()      (YOLO + back-project → centroid)
                    → /cmd_vel             (rotation or stop)
                    → /camera/detections_image  (debug view)
  ComputeTargetPose → tf_buffer.transform() (centroid → map-frame goal)
  NavigateToGoal    → navigate_to_pose ACTION → bt_navigator

Nav2 (external, config in nav2_params.yaml):
  bt_navigator  → planner_server → /plan
  controller_server ← /plan     → cmd_vel_nav
  velocity_smoother              → cmd_vel_smoothed
  collision_monitor ← /scan     → /cmd_vel

gz_ros_bridge       ← /cmd_vel → Gazebo wheels → robot moves → loop
```
