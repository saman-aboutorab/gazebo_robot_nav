#!/usr/bin/env python3
"""
Vision Obstacle Node  (Phase 4: Vision Obstacles into Nav2 Costmap)

Publishes visually-detected obstacles as a PointCloud2 stream consumed by
the Nav2 local costmap obstacle layer, so the planner routes around
camera-detected objects even before the LiDAR beam reaches them.

Pipeline per RGB frame:
  1. YOLOv8 detects all bounding boxes above the confidence threshold
  2. For each bbox, depth pixels are back-projected to 3-D points in
     camera_rgb_frame using the foreground filter from Phase 2
     (nearest depth cluster ± 20 % — strips background wall)
  3. All foreground points from all detections are merged into a single
     sensor_msgs/PointCloud2 on /vision_obstacles

Nav2's local costmap obstacle_layer subscribes to /vision_obstacles
(data_type: PointCloud2, marking: true, clearing: false) and marks the
points as obstacles.  LiDAR (360°) continues to handle clearing.

No changes to find_and_go_node.py are required; this node runs alongside
the rest of the stack.
"""

import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ultralytics import YOLO


class VisionObstacleNode(Node):
    def __init__(self):
        super().__init__('vision_obstacle_node')

        self.declare_parameter('confidence_threshold', 0.5)
        self.conf_threshold = self.get_parameter('confidence_threshold').value

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.latest_depth = None

        # Camera intrinsics — derived from model.sdf RGB camera
        # 640×480, horizontal_fov = 1.02974 rad, square pixels
        _w, _h = 640.0, 480.0
        _hfov = 1.02974
        self.fx = _w / (2.0 * math.tan(_hfov / 2.0))
        self.fy = self.fx
        self.cx = _w / 2.0
        self.cy = _h / 2.0

        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(Image, '/camera/depth', self.depth_cb, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/vision_obstacles', 10)

        self.get_logger().info(
            f'Vision obstacle node ready (conf_threshold={self.conf_threshold})'
        )

    # ── Sensor callbacks ───────────────────────────────────────────────────────

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    def image_cb(self, msg):
        if self.latest_depth is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)

        all_points = []
        for box in results[0].boxes:
            if float(box.conf) < self.conf_threshold:
                continue
            pts = self._backproject_bbox(box, cv_image.shape)
            if pts is not None:
                all_points.extend(pts)

        cloud_msg = self._make_pointcloud2(msg.header.stamp, all_points)
        self.cloud_pub.publish(cloud_msg)

        if all_points:
            self.get_logger().debug(
                f'Published {len(all_points)} obstacle points '
                f'({len(results[0].boxes)} detections)'
            )

    # ── Back-projection ────────────────────────────────────────────────────────

    def _backproject_bbox(self, box, image_shape):
        """
        Back-project foreground depth pixels inside a YOLO bbox to 3-D points
        in camera_rgb_frame.

        Foreground filter: keep only the nearest depth cluster (object) by
        discarding pixels more than 20 % farther than the minimum depth in the
        region.  This strips the background wall behind the detected object.

        Coordinate convention (ROS body frame, same as find_and_go_node.py):
          cam_x = forward  (depth Z_optical)
          cam_y = left     (-X_optical)
          cam_z = up       (-Y_optical)

        Returns a list of (cam_x, cam_y, cam_z) float tuples, or None if
        there are fewer than 5 valid foreground pixels.
        """
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
            return None

        d_v = d_arr[valid]
        min_d = d_v.min()
        fg = d_v < min_d * 1.2

        d_fg = d_v[fg]
        if len(d_fg) < 5:
            return None

        uu_fg = uu[valid][fg]
        vv_fg = vv[valid][fg]

        X = d_fg
        Y = -(uu_fg - self.cx) * d_fg / self.fx
        Z = -(vv_fg - self.cy) * d_fg / self.fy

        return list(zip(X.tolist(), Y.tolist(), Z.tolist()))

    # ── PointCloud2 builder ────────────────────────────────────────────────────

    def _make_pointcloud2(self, stamp, points):
        """Build a sensor_msgs/PointCloud2 from a list of (x, y, z) tuples."""
        header = Header()
        header.stamp = stamp
        header.frame_id = 'camera_rgb_frame'

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 12  # 3 × float32 = 12 bytes
        data = bytearray(point_step * len(points))
        for i, (x, y, z) in enumerate(points):
            struct.pack_into('fff', data, i * point_step, x, y, z)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(points)
        msg.data = bytes(data)
        msg.is_dense = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = VisionObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
