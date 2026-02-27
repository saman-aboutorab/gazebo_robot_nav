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
        self._node = ros_node
        self._bb = _bb_client('SearchForTarget', keys_write=('centroid',))
        self._no_detect_count = 0

    def initialise(self):
        self._no_detect_count = 0
        self._node.get_logger().info('[BT] SearchForTarget: searching…')

    def update(self):
        # Try to get a detection from the shared node's latest image
        result = self._node.detect_target()
        if result is not None:
            self._no_detect_count = 0
            centroid, n_candidates = result
            self._bb.centroid = centroid
            cam_x, cam_y, cam_z = centroid
            dist = math.hypot(cam_x, math.hypot(cam_y, cam_z))
            self._node.get_logger().info(
                f'[BT] SearchForTarget: found target '
                f'({n_candidates} candidate(s)), '
                f'centroid cam ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f}) m, '
                f'dist {dist:.2f} m → SUCCESS'
            )
            return py_trees.common.Status.SUCCESS

        # No target this tick — wait patiently before rotating so that brief
        # YOLO misses near the target don't spin the robot away.
        self._no_detect_count += 1
        if self._no_detect_count > self._ROTATE_PATIENCE_TICKS:
            self._publish_rotation()

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._node.get_logger().info(
            f'[BT] SearchForTarget: terminate ({new_status.name}) — stopping rotation'
        )
        self._stop_rotation()

    def _publish_rotation(self):
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.twist.angular.z = 0.3
        self._node.cmd_pub.publish(msg)

    def _stop_rotation(self):
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
            keys_read=('centroid',),
            keys_write=('goal_pose',),
        )
        self._last_hold_log_time = 0.0   # throttle "holding position" log

    def update(self):
        cam_x, cam_y, cam_z = self._bb.centroid

        cam_point = PointStamped()
        cam_point.header.frame_id = 'camera_rgb_frame'
        cam_point.header.stamp = rclpy.time.Time().to_msg()   # time=0 → latest TF
        cam_point.point.x = cam_x
        cam_point.point.y = cam_y
        cam_point.point.z = cam_z

        try:
            map_point = self._node.tf_buffer.transform(
                cam_point, 'map',
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self._node.get_logger().error(
                f'[BT] ComputeTargetPose: TF2 object transform failed: {e}'
            )
            return py_trees.common.Status.FAILURE

        obj_x = map_point.point.x
        obj_y = map_point.point.y

        try:
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

        dx = obj_x - robot_x
        dy = obj_y - robot_y
        target_angle = math.atan2(dy, dx)
        full_dist = math.hypot(dx, dy)

        # Already within stop_distance — don't navigate closer.
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

        nav_dist = max(full_dist - self._node.stop_dist, 0.3)
        goal_x = robot_x + nav_dist * math.cos(target_angle)
        goal_y = robot_y + nav_dist * math.sin(target_angle)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.z = math.sin(target_angle / 2.0)
        pose.pose.orientation.w = math.cos(target_angle / 2.0)

        self._bb.goal_pose = pose
        self._node.get_logger().info(
            f'[BT] ComputeTargetPose: object at map ({obj_x:.2f}, {obj_y:.2f}), '
            f'{full_dist:.1f} m away → goal ({goal_x:.2f}, {goal_y:.2f}), '
            f'travel {nav_dist:.1f} m → SUCCESS'
        )
        return py_trees.common.Status.SUCCESS


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
        self._bb = _bb_client('NavigateToGoal', keys_read=('goal_pose',))
        self._goal_handle = None
        self._result_future = None

    def initialise(self):
        pose = self._bb.goal_pose
        self._goal_handle = None
        self._result_future = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        if not self._node.nav_client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().error(
                '[BT] NavigateToGoal: Nav2 action server not available'
            )
            return  # will return FAILURE on first update()

        send_future = self._node.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)
        self._send_future = send_future
        self._node.get_logger().info('[BT] NavigateToGoal: goal sent')

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().error(
                '[BT] NavigateToGoal: goal rejected by Nav2'
            )
            return
        self._goal_handle = handle
        self._result_future = handle.get_result_async()

    def update(self):
        # Goal not yet accepted or was rejected
        if self._goal_handle is None:
            if hasattr(self, '_send_future') and not self._send_future.done():
                return py_trees.common.Status.RUNNING   # waiting for response
            return py_trees.common.Status.FAILURE       # rejected

        if not self._result_future.done():
            return py_trees.common.Status.RUNNING

        status = self._result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info('[BT] NavigateToGoal: SUCCESS')
            self._goal_handle = None    # prevent spurious cancel in terminate()
            self._result_future = None
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().warn(
            f'[BT] NavigateToGoal: Nav2 status {status} → FAILURE, retrying search'
        )
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        # Cancel the Nav2 goal if the tree cuts this branch early
        if (
            self._goal_handle is not None
            and new_status == py_trees.common.Status.INVALID
        ):
            self._node.get_logger().info(
                '[BT] NavigateToGoal: cancelling in-flight Nav2 goal'
            )
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self._result_future = None
