#!/usr/bin/env python3
"""
Find-and-Go-To-Object Node  (Phase 5: Behavior Tree)

Replaces the manual state machine from Phase 3 with a py_trees Behavior Tree:

    Retry(num_failures=-1)          # retry forever
     └── Sequence(memory=False)
           ├── SearchForTarget      # rotate + YOLO + selection policy → blackboard centroid
           ├── ComputeTargetPose    # centroid → TF2 map frame → blackboard goal
           └── NavigateToGoal      # Nav2 NavigateToPose action client

The ROS2 node owns all shared infrastructure (subscriptions, YOLO, TF2, Nav2
action client) and exposes the detect_target() helper used by SearchForTarget.
The BT is ticked at 10 Hz via a ROS2 timer.

All parameters, topics, and the launch entry-point are identical to previous
phases — nothing in the launch file or config needs to change.

Phase history
-------------
  Phase 1 — map-frame goals via TF2
  Phase 2 — foreground-filtered 3-D centroid from depth image
  Phase 3 — multi-object detection with configurable selection policy
  Phase 4 — vision obstacles fed into Nav2 local costmap (separate node)
  Phase 5 — this file: manual state machine replaced by Behavior Tree
"""

import math

import numpy as np
import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PointStamped TF2 support
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from ultralytics import YOLO

from gazebo_nav_bringup.bt_leaves import (
    SearchForTarget,
    ComputeTargetPose,
    NavigateToGoal,
)


class FindAndGoNode(Node):
    """
    Thin ROS2 node that owns all shared resources and drives the BT via timer.
    """

    def __init__(self):
        super().__init__('find_and_go')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('target_object', 'person')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('stop_distance', 1.5)
        self.declare_parameter('selection_policy', 'closest')

        self.target = self.get_parameter('target_object').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.stop_dist = self.get_parameter('stop_distance').value
        self.policy = self.get_parameter('selection_policy').value

        valid_policies = ('closest', 'highest_confidence', 'largest_bbox')
        if self.policy not in valid_policies:
            self.get_logger().warn(
                f'Unknown selection_policy "{self.policy}" — falling back to "closest". '
                f'Valid options: {valid_policies}'
            )
            self.policy = 'closest'

        # ── Shared sensor state ────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.latest_depth = None
        self._latest_image = None

        self.get_logger().info(f'Searching for: {self.target}')

        # Camera intrinsics (from model.sdf): 640×480, HFOV 1.02974 rad
        _w, _h = 640.0, 480.0
        _hfov = 1.02974
        self.fx = _w / (2.0 * math.tan(_hfov / 2.0))
        self.fy = self.fx
        self.cx = _w / 2.0
        self.cy = _h / 2.0

        # ── TF2 ───────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.det_pub = self.create_publisher(Image, '/camera/detections_image', 10)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self.create_subscription(Image, '/camera/depth', self._depth_cb, 10)

        # ── Nav2 action client ────────────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Behavior Tree ─────────────────────────────────────────────────────
        self._tree = self._build_tree()
        self._tree.setup(timeout=10.0)
        self.get_logger().info('[BT] Behavior tree ready — starting tick loop')

        # Tick the tree at 10 Hz
        self.create_timer(0.1, self._tick_tree)

    # ── Sensor callbacks ───────────────────────────────────────────────────────

    def _depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    def _image_cb(self, msg):
        self._latest_image = msg

    # ── Detection helper (called by SearchForTarget leaf) ─────────────────────

    def detect_target(self):
        """
        Run YOLO on the most recent camera frame.

        Returns (centroid, n_candidates) on success, or None if nothing found.
        centroid is (cam_x, cam_y, cam_z) in camera_rgb_frame metres.
        """
        if self._latest_image is None:
            return None

        msg = self._latest_image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)

        # Publish annotated image
        det_msg = self.bridge.cv2_to_imgmsg(results[0].plot(), 'bgr8')
        det_msg.header = msg.header
        self.det_pub.publish(det_msg)

        # Collect all valid candidates for the target class
        candidates = []
        for box in results[0].boxes:
            if self.model.names[int(box.cls)] != self.target:
                continue
            conf = float(box.conf)
            if conf < self.conf_threshold:
                continue
            centroid = self._get_3d_centroid_at_bbox(box, cv_image.shape)
            if centroid is not None:
                candidates.append((box, conf, centroid))

        if not candidates:
            return None

        _, _, centroid = self._select_target(candidates)
        return centroid, len(candidates)

    # ── Target selection (Phase 3, unchanged) ─────────────────────────────────

    def _select_target(self, candidates):
        if self.policy == 'highest_confidence':
            return max(candidates, key=lambda c: c[1])
        if self.policy == 'largest_bbox':
            def _area(c):
                x1, y1, x2, y2 = c[0].xyxy[0].tolist()
                return (x2 - x1) * (y2 - y1)
            return max(candidates, key=_area)
        # default: 'closest'
        return min(candidates, key=lambda c: c[2][0])

    # ── 3-D centroid (Phase 2, unchanged) ─────────────────────────────────────

    def _get_3d_centroid_at_bbox(self, box, image_shape):
        """
        Back-project depth pixels inside the YOLO bbox to 3-D camera-frame
        points, isolate the foreground cluster (nearest depth ±20 %), and
        return the median centroid (cam_x, cam_y, cam_z) in metres.

        Returns None if there are not enough valid foreground pixels.
        """
        if self.latest_depth is None:
            self.get_logger().warn('No depth data yet — cannot compute centroid')
            return None

        x1, y1, x2, y2 = box.xyxy[0].tolist()

        rgb_h, rgb_w = image_shape[:2]
        depth_h, depth_w = self.latest_depth.shape[:2]
        sx, sy = depth_w / rgb_w, depth_h / rgb_h

        cols = np.arange(int(x1), int(x2) + 1, dtype=np.float32)
        rows = np.arange(int(y1), int(y2) + 1, dtype=np.float32)
        uu, vv = np.meshgrid(cols, rows)

        uu_d = np.clip((uu * sx).astype(int), 0, depth_w - 1)
        vv_d = np.clip((vv * sy).astype(int), 0, depth_h - 1)
        d_arr = self.latest_depth[vv_d, uu_d]

        valid = (d_arr > 0.1) & (d_arr < 10.0) & np.isfinite(d_arr)
        if not valid.any():
            self.get_logger().warn('No valid depth in bbox — cannot compute centroid')
            return None

        d_v = d_arr[valid]
        uu_v = uu[valid]
        vv_v = vv[valid]

        min_d = d_v.min()
        fg = d_v < min_d * 1.2
        d_fg = d_v[fg]
        uu_fg = uu_v[fg]
        vv_fg = vv_v[fg]

        if len(d_fg) < 5:
            self.get_logger().warn(
                f'Only {len(d_fg)} foreground pixels — cannot compute centroid'
            )
            return None

        X = d_fg
        Y = -(uu_fg - self.cx) * d_fg / self.fx
        Z = -(vv_fg - self.cy) * d_fg / self.fy

        cam_x = float(np.median(X))
        cam_y = float(np.median(Y))
        cam_z = float(np.median(Z))

        self.get_logger().info(
            f'3-D centroid: ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f}) m '
            f'({len(d_fg)} foreground px / {valid.sum()} valid px)'
        )
        return cam_x, cam_y, cam_z

    # ── Behavior Tree wiring ──────────────────────────────────────────────────

    def _build_tree(self):
        sequence = py_trees.composites.Sequence(
            name='FindAndGoSequence',
            memory=True,    # memory=True: while NavigateToGoal is RUNNING, don't
                            # re-tick SearchForTarget (prevents spurious rotation
                            # commands fighting Nav2).  Retry resets all children
                            # on FAILURE anyway, so retry behaviour is unchanged.
        )
        sequence.add_children([
            SearchForTarget(self),
            ComputeTargetPose(self),
            NavigateToGoal(self),
        ])

        # Wrap sequence in Retry so any FAILURE restarts from SearchForTarget.
        # num_failures=-1 means retry indefinitely.
        retry = py_trees.decorators.Retry(
            child=sequence,
            name='RetryForever',
            num_failures=-1,
        )

        return py_trees.trees.BehaviourTree(root=retry)

    def _tick_tree(self):
        self._tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = FindAndGoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
