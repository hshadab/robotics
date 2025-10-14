#!/usr/bin/env python3
"""
Simple Linux/WSL2 Camera Publisher for ROS2
Publishes webcam frames to /image topic

Usage:
    python3 linux_camera_publisher.py [camera_id]

    camera_id: Camera device ID (default: 0)
               Corresponds to /dev/video0, /dev/video1, etc.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import sys
import os


class LinuxCameraPublisher(Node):
    def __init__(self, camera_id=0):
        super().__init__('linux_camera_publisher')

        # Check if camera device exists
        device_path = f'/dev/video{camera_id}'
        if not os.path.exists(device_path):
            self.get_logger().error(f'Camera device {device_path} not found')
            self.get_logger().error('WSL2 typically does not have USB camera access')
            self.get_logger().info('Solutions:')
            self.get_logger().info('  1. Use burger mode (test patterns) - already works!')
            self.get_logger().info('  2. Run camera on Windows with ROS2')
            self.get_logger().info('  3. Enable USB passthrough (advanced)')
            sys.exit(1)

        self.publisher_ = self.create_publisher(Image, '/image', 10)

        # Try to open camera with V4L2 backend
        self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {device_path}')
            self.get_logger().error('Even though device exists, cannot access it')
            sys.exit(1)

        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Get actual resolution
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))

        self.get_logger().info('='*50)
        self.get_logger().info(f'Camera {device_path} opened successfully!')
        self.get_logger().info(f'Resolution: {width}x{height} @ {fps} FPS')
        self.get_logger().info('Publishing to /image topic')
        self.get_logger().info('Press Ctrl+C to stop')
        self.get_logger().info('='*50)

        # Create timer for 30 FPS
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.frame_count = 0

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error('Failed to read frame from camera')
            return

        # Convert to ROS Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()

        self.publisher_.publish(msg)

        self.frame_count += 1
        if self.frame_count % 300 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()


def main(args=None):
    print("\n" + "="*50)
    print("Linux/WSL2 Camera Publisher for zkML Demo")
    print("="*50 + "\n")

    # Get camera ID from command line
    camera_id = 0
    if len(sys.argv) > 1:
        try:
            camera_id = int(sys.argv[1])
        except ValueError:
            print(f"Error: Invalid camera ID '{sys.argv[1]}'")
            print("Usage: python3 linux_camera_publisher.py [camera_id]")
            sys.exit(1)

    print(f"Attempting to open /dev/video{camera_id}...\n")

    rclpy.init(args=args)

    try:
        node = LinuxCameraPublisher(camera_id)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nStopping camera...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        rclpy.shutdown()
        print("Camera publisher stopped.\n")


if __name__ == '__main__':
    main()
