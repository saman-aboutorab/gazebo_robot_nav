#!/usr/bin/env python3
"""
Find-and-Go-To-Object Node  (Phase 3: Multi-Object Targeting)

Rotates the robot to search for a target object using YOLOv8,
evaluates ALL detections of the target class in each frame,
selects the best one via a configurable policy, then sends a
Nav2 NavigateToPose goal in the /map frame via TF2.

Phase 3 upgrade over Phase 2:
  Phase 2 stopped at the first matching detection.  Phase 3 collects
  every detection of the target class that passes the confidence
  threshold, computes a foreground-filtered 3D centroid for each, and
  picks one according to selection_policy:

    'closest'            — smallest forward depth (cam_x)  [default]
    'highest_confidence' — highest YOLO confidence score
    'largest_bbox'       — largest bounding-box pixel area
                           (usually the closest but more stable
                           than raw depth for partially occluded
                           objects)

  When multiple candidates are found the selection is logged so
  behaviour is fully observable.

Phase 2 depth pipeline (unchanged):
  foreground-filtered 3D centroid  →  camera_rgb_frame  →  map
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

        # Collect all detections of the target class that pass the threshold
        # and have a valid 3-D centroid.  Each entry: (box, conf, centroid).
        candidates = []
        for box in results[0].boxes:
            if self.model.names[int(box.cls)] != self.target:
                continue
            conf = float(box.conf)
            if conf < self.conf_threshold:
                continue
            centroid = self.get_3d_centroid_at_bbox(box, cv_image.shape)
            if centroid is not None:
                candidates.append((box, conf, centroid))

        if not candidates:
            return  # nothing valid this frame — keep searching

        # Apply selection policy to pick the best candidate
        selected_box, selected_conf, selected_centroid = \
            self._select_target(candidates)

        n = len(candidates)
        cam_x, cam_y, cam_z = selected_centroid
        dist = math.hypot(cam_x, math.hypot(cam_y, cam_z))
        self.get_logger().info(
            f'Detected {n} {self.target}(s) — selected by "{self.policy}": '
            f'conf {selected_conf:.2f}, '
            f'centroid cam ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f}) m, '
            f'dist {dist:.2f} m'
        )

        self.cmd_pub.publish(TwistStamped())   # stop rotating
        self.state = 'navigating'
        self.search_timer.cancel()
        self.navigate_to_target(cam_x, cam_y, cam_z)

    # ── Target selection (Phase 3) ────────────────────────────────────────────

    def _select_target(self, candidates):
        """
        Pick the best candidate from a list of (box, conf, (cam_x, cam_y, cam_z))
        tuples according to self.policy.

          'closest'            — smallest cam_x (forward depth)
          'highest_confidence' — highest YOLO confidence score
          'largest_bbox'       — largest bounding-box pixel area
        """
        if self.policy == 'highest_confidence':
            return max(candidates, key=lambda c: c[1])

        if self.policy == 'largest_bbox':
            def _area(c):
                x1, y1, x2, y2 = c[0].xyxy[0].tolist()
                return (x2 - x1) * (y2 - y1)
            return max(candidates, key=_area)

        # default: 'closest'
        return min(candidates, key=lambda c: c[2][0])  # cam_x = forward depth

    # ── Search rotation ───────────────────────────────────────────────────────

    def search_tick(self):
        if self.state != 'searching':
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = 0.3
        self.cmd_pub.publish(msg)

    # ── 3-D centroid measurement (Phase 2) ───────────────────────────────────

    def get_3d_centroid_at_bbox(self, box, image_shape):
        """
        Back-project every depth pixel inside the YOLO bbox to a 3-D point
        in camera_rgb_frame, isolate the foreground cluster (nearest depth ±20 %),
        and return the median 3-D centroid (cam_x, cam_y, cam_z) in metres.

        Returns None if there are not enough valid foreground pixels.

        Coordinate convention (ROS body frame, same as navigate_to_target):
          cam_x  = forward  (depth Z_optical)
          cam_y  = left     (-X_optical)
          cam_z  = up       (-Y_optical)
        """
        if self.latest_depth is None:
            self.get_logger().warn('No depth data yet — cannot compute centroid')
            return None

        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # Pixel grid over the bbox in RGB image space
        rgb_h, rgb_w = image_shape[:2]
        depth_h, depth_w = self.latest_depth.shape[:2]
        sx, sy = depth_w / rgb_w, depth_h / rgb_h

        cols = np.arange(int(x1), int(x2) + 1, dtype=np.float32)
        rows = np.arange(int(y1), int(y2) + 1, dtype=np.float32)
        uu, vv = np.meshgrid(cols, rows)          # RGB pixel coords, shape (H, W)

        # Look up depth for each RGB pixel (scale to depth image resolution)
        uu_d = np.clip((uu * sx).astype(int), 0, depth_w - 1)
        vv_d = np.clip((vv * sy).astype(int), 0, depth_h - 1)
        d_arr = self.latest_depth[vv_d, uu_d]     # shape (H, W)

        # Valid range mask
        valid = (d_arr > 0.1) & (d_arr < 10.0) & np.isfinite(d_arr)
        if not valid.any():
            self.get_logger().warn('No valid depth in bbox — cannot compute centroid')
            return None

        d_v = d_arr[valid]
        uu_v = uu[valid]
        vv_v = vv[valid]

        # Foreground filter: keep only the nearest depth cluster (object, not wall).
        # Pixels more than 20 % farther than the minimum are treated as background.
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

        # Back-project to camera_rgb_frame (ROS body convention)
        #   cam_x = depth (forward)
        #   cam_y = -(u - cx) * d / fx   (left is +Y)
        #   cam_z = -(v - cy) * d / fy   (up is +Z)
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

    # ── Map-frame goal (Phase 2) ──────────────────────────────────────────────

    def navigate_to_target(self, cam_x, cam_y, cam_z):
        """
        Send a Nav2 goal from a 3-D point already expressed in camera_rgb_frame.

        Pipeline (Phase 2):
          3-D centroid in camera_rgb_frame  →  tf_buffer.transform() to /map
                                            →  stop_distance short of the object
                                            →  NavigateToPose action
        """
        cam_point = PointStamped()
        cam_point.header.frame_id = 'camera_rgb_frame'
        cam_point.header.stamp = rclpy.time.Time().to_msg()  # time=0 → latest TF
        cam_point.point.x = cam_x
        cam_point.point.y = cam_y
        cam_point.point.z = cam_z

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
        from action_msgs.msg import GoalStatus
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation complete!')
            self.state = 'done'
        else:
            self.get_logger().warn(
                f'Navigation failed (status {status}) — returning to search'
            )
            self.state = 'searching'
            self.search_timer = self.create_timer(0.1, self.search_tick)


def main(args=None):
    rclpy.init(args=args)
    node = FindAndGoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
