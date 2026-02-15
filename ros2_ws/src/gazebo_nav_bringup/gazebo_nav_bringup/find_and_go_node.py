#!/usr/bin/env python3
"""
Find-and-Go-To-Object Node

Rotates the robot to search for a target object using YOLOv8,
reads the depth camera to measure real distance, then sends
a Nav2 goal to navigate toward it.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
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

        # State: 'searching', 'navigating', 'done'
        self.state = 'searching'
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.latest_depth = None

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.det_pub = self.create_publisher(Image, '/camera/detections_image', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(Image, '/camera/depth', self.depth_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Rotate timer (search phase)
        self.search_timer = self.create_timer(0.1, self.search_tick)

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def depth_cb(self, msg):
        """Store latest depth image as a numpy array of meters."""
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def search_tick(self):
        """Rotate slowly while searching."""
        if self.state != 'searching':
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = 0.3
        self.cmd_pub.publish(msg)

    def image_cb(self, msg):
        if self.state != 'searching':
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)
        annotated = results[0].plot()

        # Publish annotated image
        det_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        det_msg.header = msg.header
        self.det_pub.publish(det_msg)

        # Check for target object
        for box in results[0].boxes:
            cls_name = self.model.names[int(box.cls)]
            conf = float(box.conf)
            if cls_name == self.target and conf >= self.conf_threshold:
                # Stop rotating
                self.cmd_pub.publish(TwistStamped())
                self.state = 'navigating'
                self.search_timer.cancel()

                # Measure distance with depth camera
                distance = self.get_depth_at_bbox(box, cv_image.shape)
                self.get_logger().info(
                    f'Found {cls_name} ({conf:.2f}) at {distance:.2f}m! Navigating toward it.'
                )
                self.navigate_to_target(box, cv_image.shape[1], distance)
                return

    def get_depth_at_bbox(self, box, image_shape):
        """Read median depth from the center region of the bounding box."""
        if self.latest_depth is None:
            self.get_logger().warn('No depth data yet, using fallback distance 3.0m')
            return 3.0

        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # Depth image may be different resolution than RGB (320x240 vs 640x480)
        rgb_h, rgb_w = image_shape[:2]
        depth_h, depth_w = self.latest_depth.shape[:2]
        scale_x = depth_w / rgb_w
        scale_y = depth_h / rgb_h

        # Scale bbox to depth image coordinates
        dx1 = int(x1 * scale_x)
        dy1 = int(y1 * scale_y)
        dx2 = int(x2 * scale_x)
        dy2 = int(y2 * scale_y)

        # Take center 50% of bbox to avoid edge noise
        cx = (dx1 + dx2) // 2
        cy = (dy1 + dy2) // 2
        half_w = max((dx2 - dx1) // 4, 1)
        half_h = max((dy2 - dy1) // 4, 1)
        roi = self.latest_depth[cy - half_h:cy + half_h, cx - half_w:cx + half_w]

        # Filter out invalid readings (inf, nan, zero)
        valid = roi[(roi > 0.1) & (roi < 10.0) & np.isfinite(roi)]
        if len(valid) == 0:
            self.get_logger().warn('No valid depth in bbox, using fallback 3.0m')
            return 3.0

        median_dist = float(np.median(valid))
        self.get_logger().info(f'Depth measurement: {median_dist:.2f}m (from {len(valid)} pixels)')
        return median_dist

    def navigate_to_target(self, box, image_width, measured_distance):
        """Use real depth distance to place Nav2 goal in front of the object."""
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        bbox_center_x = (x1 + x2) / 2.0

        # Estimate angle offset from image center
        hfov = 1.02974
        angle_offset = (bbox_center_x - image_width / 2) / image_width * hfov
        target_angle = self.robot_yaw - angle_offset

        # Navigate to (measured_distance - stop_distance) along that angle
        nav_distance = max(measured_distance - self.stop_dist, 0.5)
        goal_x = self.robot_x + nav_distance * math.cos(target_angle)
        goal_y = self.robot_y + nav_distance * math.sin(target_angle)

        self.get_logger().info(
            f'Object at {measured_distance:.1f}m, stopping {self.stop_dist:.1f}m away → '
            f'Nav2 goal: ({goal_x:.2f}, {goal_y:.2f}), travel: {nav_distance:.1f}m'
        )

        # Build and send Nav2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.z = math.sin(target_angle / 2)
        goal_msg.pose.pose.orientation.w = math.cos(target_angle / 2)

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected!')
            self.state = 'searching'
            return
        self.get_logger().info('Nav2 goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

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
