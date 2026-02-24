#!/usr/bin/env python3
"""
Find-and-Go-To-Object Node  (Phase 1: Map-Frame Goal Localization)

Rotates the robot to search for a target object using YOLOv8,
reads the depth camera to measure real distance, then computes
the object's position in the /map frame via TF2 and sends a
Nav2 NavigateToPose goal.

Phase 1 fix: goal poses are now derived from the full TF tree
  depth + bbox  →  camera_rgb_frame  →  map
instead of the previous approach of adding depth to raw odometry,
which accumulated drift from rotations and wheel slip.
"""

import math
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
import tf2_geometry_msgs  # registers PointStamped support for tf_buffer.transform()
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from ultralytics import YOLO


class FindAndGoNode(Node):
    def __init__(self):
        super().__init__('find_and_go')

        # Parameters
        self.declare_parameter('target_object', 'person')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('stop_distance', 1.0)
        self.target = self.get_parameter('target_object').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.stop_dist = self.get_parameter('stop_distance').value

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info(f'Searching for: {self.target}')

        # Camera intrinsics — derived from model.sdf
        # RGB: 640x480, horizontal_fov = 1.02974 rad, square pixels
        _w, _h = 640.0, 480.0
        _hfov = 1.02974
        self.fx = _w / (2.0 * math.tan(_hfov / 2.0))
        self.fy = self.fx
        self.cx = _w / 2.0
        self.cy = _h / 2.0

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State: 'searching' | 'navigating' | 'done'
        self.state = 'searching'
        self.latest_depth = None

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.det_pub = self.create_publisher(Image, '/camera/detections_image', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(Image, '/camera/depth', self.depth_cb, 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Rotate timer (search phase)
        self.search_timer = self.create_timer(0.1, self.search_tick)

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def depth_cb(self, msg):
        """Store latest depth image as a float32 numpy array (metres)."""
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    def image_cb(self, msg):
        if self.state != 'searching':
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)

        # Publish annotated image
        det_msg = self.bridge.cv2_to_imgmsg(results[0].plot(), 'bgr8')
        det_msg.header = msg.header
        self.det_pub.publish(det_msg)

        for box in results[0].boxes:
            cls_name = self.model.names[int(box.cls)]
            conf = float(box.conf)
            if cls_name == self.target and conf >= self.conf_threshold:
                self.cmd_pub.publish(TwistStamped())   # stop rotating
                self.state = 'navigating'
                self.search_timer.cancel()

                distance = self.get_depth_at_bbox(box, cv_image.shape)
                self.get_logger().info(
                    f'Found {cls_name} ({conf:.2f}) at {distance:.2f} m'
                )
                self.navigate_to_target(box, distance)
                return

    # ── Search rotation ───────────────────────────────────────────────────────

    def search_tick(self):
        if self.state != 'searching':
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = 0.3
        self.cmd_pub.publish(msg)

    # ── Depth measurement ─────────────────────────────────────────────────────

    def get_depth_at_bbox(self, box, image_shape):
        """Return median depth (metres) from the centre region of the bbox."""
        if self.latest_depth is None:
            self.get_logger().warn('No depth data yet — using fallback 3.0 m')
            return 3.0

        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # Depth image may differ in resolution from RGB (320x240 vs 640x480)
        rgb_h, rgb_w = image_shape[:2]
        depth_h, depth_w = self.latest_depth.shape[:2]
        sx, sy = depth_w / rgb_w, depth_h / rgb_h

        dx1, dy1 = int(x1 * sx), int(y1 * sy)
        dx2, dy2 = int(x2 * sx), int(y2 * sy)

        # Centre 50 % of the bbox to avoid edge noise
        cx = (dx1 + dx2) // 2
        cy = (dy1 + dy2) // 2
        hw = max((dx2 - dx1) // 4, 1)
        hh = max((dy2 - dy1) // 4, 1)
        roi = self.latest_depth[cy - hh:cy + hh, cx - hw:cx + hw]

        valid = roi[(roi > 0.1) & (roi < 10.0) & np.isfinite(roi)]
        if len(valid) == 0:
            self.get_logger().warn('No valid depth in bbox — using fallback 3.0 m')
            return 3.0

        median = float(np.median(valid))
        self.get_logger().info(
            f'Depth: {median:.2f} m ({len(valid)} valid pixels)'
        )
        return median

    # ── Map-frame goal (Phase 1) ──────────────────────────────────────────────

    def navigate_to_target(self, box, depth):
        """
        Convert the detected bbox + depth into a /map-frame Nav2 goal.

        Pipeline:
          pixel (u, v) + depth  →  3-D point in camera_rgb_frame
                                 →  tf_buffer.transform() to /map
                                 →  stop_distance short of the object
                                 →  NavigateToPose action
        """
        # Bbox centre in image pixels
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0

        # Back-project to 3-D using the pinhole model.
        # camera_rgb_frame uses the ROS body convention (X fwd, Y left, Z up)
        # so we map optical axes (Z fwd, X right, Y down) accordingly:
        #   body X  = depth              (forward)
        #   body Y  = -(u-cx)*d/fx      (left is +Y; rightward pixel = -Y)
        #   body Z  = -(v-cy)*d/fy      (up is +Z; downward pixel = -Z)
        cam_point = PointStamped()
        cam_point.header.frame_id = 'camera_rgb_frame'
        cam_point.header.stamp = self.get_clock().now().to_msg()
        cam_point.point.x =  depth
        cam_point.point.y = -(u - self.cx) * depth / self.fx
        cam_point.point.z = -(v - self.cy) * depth / self.fy

        # Transform object position from camera frame → map frame
        try:
            map_point = self.tf_buffer.transform(
                cam_point, 'map',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f'TF2 object transform failed: {e}')
            self.state = 'searching'
            return

        obj_x = map_point.point.x
        obj_y = map_point.point.y

        # Get robot's current pose in map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f'TF2 robot pose lookup failed: {e}')
            self.state = 'searching'
            return

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        # Direction from robot to object in map frame
        dx = obj_x - robot_x
        dy = obj_y - robot_y
        target_angle = math.atan2(dy, dx)
        full_dist = math.hypot(dx, dy)

        # Stop stop_dist before the object; never closer than 0.3 m
        nav_dist = max(full_dist - self.stop_dist, 0.3)
        goal_x = robot_x + nav_dist * math.cos(target_angle)
        goal_y = robot_y + nav_dist * math.sin(target_angle)

        self.get_logger().info(
            f'Object at map ({obj_x:.2f}, {obj_y:.2f}), '
            f'{full_dist:.1f} m away → '
            f'Nav2 goal ({goal_x:.2f}, {goal_y:.2f}), travel {nav_dist:.1f} m'
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.z = math.sin(target_angle / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(target_angle / 2.0)

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_cb)

    # ── Nav2 callbacks ────────────────────────────────────────────────────────

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected — returning to search')
            self.state = 'searching'
            return
        self.get_logger().info('Nav2 goal accepted, navigating…')
        goal_handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        self.get_logger().info('Navigation complete!')
        self.state = 'done'


def main(args=None):
    rclpy.init(args=args)
    node = FindAndGoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
