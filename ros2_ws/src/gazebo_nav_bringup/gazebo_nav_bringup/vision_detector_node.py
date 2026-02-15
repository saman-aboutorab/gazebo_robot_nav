#!/usr/bin/env python3
"""
YOLOv8 Vision Detector Node

Subscribes to /camera/image_raw, runs YOLOv8 inference,
publishes annotated image to /camera/detections_image.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class VisionDetectorNode(Node):
    def __init__(self):
        super().__init__('vision_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLOv8n model loaded')

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.pub = self.create_publisher(
            Image, '/camera/detections_image', 10
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)
        annotated = results[0].plot()

        for box in results[0].boxes:
            cls_name = self.model.names[int(box.cls)]
            conf = float(box.conf)
            self.get_logger().info(f'Detected: {cls_name} ({conf:.2f})')

        det_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        det_msg.header = msg.header
        self.pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
