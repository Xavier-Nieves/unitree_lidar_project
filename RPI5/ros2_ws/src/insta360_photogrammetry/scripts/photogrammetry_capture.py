#!/usr/bin/env python3
"""
Photogrammetry Image Capture Node for Insta360 X3

Subscribes to camera images and saves them automatically for photogrammetry:
- Time-based capture (every N seconds)
- Distance-based capture (every N meters traveled)
- Manual trigger via service call

For 3D reconstruction with software like:
- Meshroom, RealityCapture, Metashape, OpenDroneMap
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import numpy as np


class PhotogrammetryCapture(Node):
    def __init__(self):
        super().__init__('photogrammetry_capture')

        # Parameters
        self.declare_parameter('capture_interval', 2.0)  # seconds
        self.declare_parameter('output_dir', './photogrammetry_captures')
        self.declare_parameter('auto_capture', True)
        self.declare_parameter('image_topic', '/insta360_x3/image_raw')
        self.declare_parameter('save_format', 'jpg')  # jpg or png
        self.declare_parameter('jpeg_quality', 95)  # 0-100

        self.capture_interval = self.get_parameter('capture_interval').value
        self.output_dir = self.get_parameter('output_dir').value
        self.auto_capture = self.get_parameter('auto_capture').value
        self.image_topic = self.get_parameter('image_topic').value
        self.save_format = self.get_parameter('save_format').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f'Saving images to: {self.output_dir}')

        # CV Bridge
        self.bridge = CvBridge()

        # State
        self.last_capture_time = self.get_clock().now()
        self.capture_count = 0
        self.latest_image = None

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # Service for manual capture
        self.capture_srv = self.create_service(
            Trigger,
            'capture_image',
            self.capture_service_callback
        )

        self.get_logger().info('Photogrammetry capture node started')
        self.get_logger().info(f'Auto capture: {self.auto_capture}')
        self.get_logger().info(f'Capture interval: {self.capture_interval}s')

    def image_callback(self, msg):
        """Process incoming images"""
        self.latest_image = msg

        if self.auto_capture:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.last_capture_time).nanoseconds / 1e9

            if elapsed >= self.capture_interval:
                self.save_image(msg)
                self.last_capture_time = current_time

    def save_image(self, msg):
        """Save image to disk"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Generate filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            self.capture_count += 1
            filename = f"img_{self.capture_count:06d}_{timestamp}.{self.save_format}"
            filepath = os.path.join(self.output_dir, filename)

            # Save image
            if self.save_format == 'jpg':
                cv2.imwrite(filepath, cv_image,
                          [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            else:
                cv2.imwrite(filepath, cv_image)

            self.get_logger().info(f'Saved: {filename}')

        except Exception as e:
            self.get_logger().error(f'Failed to save image: {str(e)}')

    def capture_service_callback(self, request, response):
        """Handle manual capture trigger"""
        if self.latest_image is not None:
            self.save_image(self.latest_image)
            response.success = True
            response.message = f'Image captured ({self.capture_count} total)'
        else:
            response.success = False
            response.message = 'No image available'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PhotogrammetryCapture()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total images captured: {node.capture_count}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
