#!/usr/bin/env python3
"""
Behavior Tree leaf nodes for the Find-and-Go behavior (Phase 5).

Tree structure:
    Retry(num_failures=-1)          # retry forever
     └── Sequence(memory=False)
           ├── SearchForTarget      # rotate + YOLO → blackboard centroid
           ├── ComputeTargetPose    # centroid → TF2 map frame → blackboard goal
           └── NavigateToGoal      # Nav2 NavigateToPose action client

Blackboard keys
---------------
  centroid  : tuple (cam_x, cam_y, cam_z) in camera_rgb_frame metres
  goal_pose : geometry_msgs/PoseStamped in /map frame
"""

import math

import py_trees
import rclpy.duration
import rclpy.time
import tf2_geometry_msgs  # noqa: F401  registers PointStamped TF2 support
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose


# ── helpers ───────────────────────────────────────────────────────────────────

def _bb_client(node_name, keys_write=(), keys_read=()):
    """Return a blackboard client for the given leaf node name."""
    # The blackboard is a shared key-value store for the whole BT.
    # Each leaf registers which keys it will read or write so py_trees
    # can detect accidental cross-leaf conflicts at setup time.
    client = py_trees.blackboard.Client(name=node_name)
    for key in keys_write:
        client.register_key(key=key, access=py_trees.common.Access.WRITE)
    for key in keys_read:
        client.register_key(key=key, access=py_trees.common.Access.READ)
    return client


# ── SearchForTarget ───────────────────────────────────────────────────────────

class SearchForTarget(py_trees.behaviour.Behaviour):
    """
    Rotates the robot and runs YOLO on every camera frame.

    Returns:
        SUCCESS  — a valid 3-D centroid was found; written to blackboard key
                   'centroid' as (cam_x, cam_y, cam_z).
        RUNNING  — still searching; robot keeps rotating.
        FAILURE  — should not occur (search runs indefinitely until found).
    """

    # How many consecutive missed detections to tolerate before rotating.
    # At 10 Hz tick rate, 10 ticks = 1 second of patience.  This prevents
    # the robot from spinning away when YOLO briefly loses the target at
    # close range (e.g. during the "holding position" retry loop).
    _ROTATE_PATIENCE_TICKS = 10

    def __init__(self, ros_node):
        super().__init__(name='SearchForTarget')
        self._node = ros_node                               # shared FindAndGoNode — owns camera subs, YOLO, publishers
        self._bb = _bb_client('SearchForTarget', keys_write=('centroid',))  # write-only access to 'centroid' key
        self._no_detect_count = 0                          # counts consecutive ticks with no detection

    def initialise(self):
        # Called once each time the Sequence re-enters this leaf (after a Retry reset).
        self._no_detect_count = 0
        self._node.get_logger().info('[BT] SearchForTarget: searching…')

    def update(self):
        # Called on every BT tick (10 Hz). Runs YOLO on the latest camera frame.
        result = self._node.detect_target()
        if result is not None:
            self._no_detect_count = 0
            centroid, n_candidates = result

            # Write centroid to blackboard so ComputeTargetPose can read it next.
            self._bb.centroid = centroid

            cam_x, cam_y, cam_z = centroid
            dist = math.hypot(cam_x, math.hypot(cam_y, cam_z))  # 3-D distance from camera
            self._node.get_logger().info(
                f'[BT] SearchForTarget: found target '
                f'({n_candidates} candidate(s)), '
                f'centroid cam ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f}) m, '
                f'dist {dist:.2f} m → SUCCESS'
            )
            return py_trees.common.Status.SUCCESS   # Sequence will advance to ComputeTargetPose

        # No target this tick — increment miss counter.
        self._no_detect_count += 1
        if self._no_detect_count > self._ROTATE_PATIENCE_TICKS:
            # Patience expired: start rotating so the camera sweeps a new area.
            self._publish_rotation()

        return py_trees.common.Status.RUNNING   # Sequence stays here; robot keeps spinning

    def terminate(self, new_status):
        # Called when this leaf exits for any reason (SUCCESS, FAILURE, or INVALID).
        # Always stop rotation so we don't fight Nav2 once navigation starts.
        self._node.get_logger().info(
            f'[BT] SearchForTarget: terminate ({new_status.name}) — stopping rotation'
        )
        self._stop_rotation()

    def _publish_rotation(self):
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.twist.angular.z = 0.3       # 0.3 rad/s slow left rotation to scan the room
        self._node.cmd_pub.publish(msg) # publishes to /cmd_vel → gz_ros_bridge → Gazebo wheels

    def _stop_rotation(self):
        # Publishing an empty TwistStamped sends all-zero velocities = full stop.
        self._node.cmd_pub.publish(TwistStamped())


# ── ComputeTargetPose ─────────────────────────────────────────────────────────

class ComputeTargetPose(py_trees.behaviour.Behaviour):
    """
    Reads the camera-frame centroid from the blackboard and transforms it to
    a Nav2-ready PoseStamped in the /map frame, offset by stop_distance.

    Returns:
        SUCCESS  — goal pose written to blackboard key 'goal_pose'.
        FAILURE  — TF2 transform failed; tree retries from SearchForTarget.
    """

    def __init__(self, ros_node):
        super().__init__(name='ComputeTargetPose')
        self._node = ros_node
        self._bb = _bb_client(
            'ComputeTargetPose',
            keys_read=('centroid',),        # reads centroid written by SearchForTarget
            keys_write=('goal_pose',),      # writes goal_pose for NavigateToGoal to read
        )
        self._last_hold_log_time = 0.0   # throttle "holding position" log

    def update(self):
        # Step 1: read the 3-D centroid in camera frame from the blackboard.
        cam_x, cam_y, cam_z = self._bb.centroid

        # Step 2: wrap the centroid in a stamped message so TF2 knows which frame it lives in.
        cam_point = PointStamped()
        cam_point.header.frame_id = 'camera_rgb_frame'  # the frame the centroid is expressed in
        cam_point.header.stamp = rclpy.time.Time().to_msg()  # time=0 → use the latest available TF transform
        cam_point.point.x = cam_x
        cam_point.point.y = cam_y
        cam_point.point.z = cam_z

        try:
            # Step 3: transform centroid from camera_rgb_frame → map frame.
            # TF2 chains: camera_rgb_frame → base_link → base_footprint → odom → map
            # All those intermediate transforms come from robot_state_publisher (/tf_static)
            # and slam_toolbox (/tf).
            map_point = self._node.tf_buffer.transform(
                cam_point, 'map',
                timeout=rclpy.duration.Duration(seconds=1.0),  # fail fast if TF is stale
            )
        except Exception as e:
            self._node.get_logger().error(
                f'[BT] ComputeTargetPose: TF2 object transform failed: {e}'
            )
            return py_trees.common.Status.FAILURE   # Retry will restart from SearchForTarget

        # Map-frame position of the detected object.
        obj_x = map_point.point.x
        obj_y = map_point.point.y

        try:
            # Step 4: look up where the robot itself is in the map frame.
            # base_footprint is the robot's footprint on the ground plane.
            tf = self._node.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self._node.get_logger().error(
                f'[BT] ComputeTargetPose: TF2 robot pose lookup failed: {e}'
            )
            return py_trees.common.Status.FAILURE

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        # Step 5: compute distance and bearing from robot to object.
        dx = obj_x - robot_x
        dy = obj_y - robot_y
        target_angle = math.atan2(dy, dx)   # bearing to object in map frame (radians)
        full_dist = math.hypot(dx, dy)      # straight-line distance to object (metres)

        # Step 6: if already close enough, don't navigate — hold position.
        # Returning FAILURE lets Retry restart from SearchForTarget, which will
        # keep re-detecting the (still-visible) target and returning here until
        # the target moves away and full_dist > stop_dist again.
        if full_dist <= self._node.stop_dist:
            now = self._node.get_clock().now().nanoseconds * 1e-9
            if now - self._last_hold_log_time >= 1.0:   # log at most once/sec
                self._node.get_logger().info(
                    f'[BT] ComputeTargetPose: holding position '
                    f'({full_dist:.2f} m ≤ stop_dist {self._node.stop_dist:.2f} m)'
                )
                self._last_hold_log_time = now
            return py_trees.common.Status.FAILURE

        # Step 7: compute a goal point stop_distance short of the object so the
        # robot stops in front of it rather than trying to drive into it.
        # Clamp to at least 0.3 m so Nav2 never gets a trivially close goal.
        nav_dist = max(full_dist - self._node.stop_dist, 0.3)
        goal_x = robot_x + nav_dist * math.cos(target_angle)
        goal_y = robot_y + nav_dist * math.sin(target_angle)

        # Step 8: build the PoseStamped goal with orientation facing the object.
        # Quaternion for a pure yaw rotation: z=sin(yaw/2), w=cos(yaw/2), x=y=0.
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.z = math.sin(target_angle / 2.0)
        pose.pose.orientation.w = math.cos(target_angle / 2.0)

        # Write goal to blackboard for NavigateToGoal to consume on the next tick.
        self._bb.goal_pose = pose
        self._node.get_logger().info(
            f'[BT] ComputeTargetPose: object at map ({obj_x:.2f}, {obj_y:.2f}), '
            f'{full_dist:.1f} m away → goal ({goal_x:.2f}, {goal_y:.2f}), '
            f'travel {nav_dist:.1f} m → SUCCESS'
        )
        return py_trees.common.Status.SUCCESS   # Sequence advances to NavigateToGoal


# ── NavigateToGoal ────────────────────────────────────────────────────────────

class NavigateToGoal(py_trees.behaviour.Behaviour):
    """
    Sends a Nav2 NavigateToPose action goal and polls until it completes.

    Returns:
        RUNNING  — Nav2 is still driving.
        SUCCESS  — Nav2 reported STATUS_SUCCEEDED.
        FAILURE  — goal rejected, cancelled, or Nav2 reported an error status;
                   tree retries from SearchForTarget.
    """

    def __init__(self, ros_node):
        super().__init__(name='NavigateToGoal')
        self._node = ros_node
        self._bb = _bb_client('NavigateToGoal', keys_read=('goal_pose',))  # read-only access to goal_pose
        self._goal_handle = None    # Nav2 goal handle — used later to cancel if needed
        self._result_future = None  # async future that resolves when Nav2 finishes

    def initialise(self):
        # Called once when the Sequence first enters this leaf.
        # Sends the Nav2 action goal asynchronously (non-blocking).
        pose = self._bb.goal_pose
        self._goal_handle = None
        self._result_future = None

        # Build the NavigateToPose goal message.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose    # the PoseStamped computed by ComputeTargetPose

        # Check Nav2 is alive before sending (2 s timeout).
        if not self._node.nav_client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().error(
                '[BT] NavigateToGoal: Nav2 action server not available'
            )
            return  # _goal_handle stays None → update() will return FAILURE

        # send_goal_async() returns immediately with a future.
        # _goal_response_cb is called when Nav2 accepts or rejects the goal.
        send_future = self._node.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)
        self._send_future = send_future
        self._node.get_logger().info('[BT] NavigateToGoal: goal sent')

    def _goal_response_cb(self, future):
        # Called by ROS2 executor when Nav2 responds to the goal request.
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().error(
                '[BT] NavigateToGoal: goal rejected by Nav2'
            )
            return  # _goal_handle stays None → update() returns FAILURE
        self._goal_handle = handle
        # get_result_async() returns a future that resolves when Nav2 finishes driving.
        self._result_future = handle.get_result_async()

    def update(self):
        # Called every BT tick (10 Hz) while this leaf is active.

        # Goal not yet accepted or was rejected — wait one more tick or fail.
        if self._goal_handle is None:
            if hasattr(self, '_send_future') and not self._send_future.done():
                return py_trees.common.Status.RUNNING   # still waiting for Nav2 to accept
            return py_trees.common.Status.FAILURE       # goal was rejected

        # Nav2 is still driving — result future not resolved yet.
        if not self._result_future.done():
            return py_trees.common.Status.RUNNING

        # Nav2 finished — check the outcome.
        status = self._result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info('[BT] NavigateToGoal: SUCCESS')
            self._goal_handle = None    # clear handle so terminate() doesn't try to cancel
            self._result_future = None
            return py_trees.common.Status.SUCCESS   # Sequence returns SUCCESS → Retry resets all children

        # Any other status (ABORTED, CANCELED, etc.) is treated as failure.
        self._node.get_logger().warn(
            f'[BT] NavigateToGoal: Nav2 status {status} → FAILURE, retrying search'
        )
        return py_trees.common.Status.FAILURE   # Retry restarts from SearchForTarget

    def terminate(self, new_status):
        # Called when the BT cuts this branch early (e.g. external preemption).
        # INVALID means the leaf was interrupted before it could finish naturally.
        if (
            self._goal_handle is not None
            and new_status == py_trees.common.Status.INVALID
        ):
            self._node.get_logger().info(
                '[BT] NavigateToGoal: cancelling in-flight Nav2 goal'
            )
            self._goal_handle.cancel_goal_async()   # tell Nav2 to stop driving
            self._goal_handle = None
            self._result_future = None
