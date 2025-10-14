#!/usr/bin/env python3
"""
HTTP Camera Bridge for ROS2
Fetches frames from Windows HTTP camera server and publishes to ROS2

This runs in WSL2 and bridges the gap between Windows camera and ROS2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import urllib.request
import numpy as np
import cv2
import time


class HttpCameraBridge(Node):
    def __init__(self, camera_url='http://localhost:8080/frame.jpg'):
        super().__init__('http_camera_bridge')

        self.camera_url = camera_url
        self.publisher_ = self.create_publisher(Image, '/image', 10)

        self.get_logger().info('='*50)
        self.get_logger().info('HTTP Camera Bridge Started')
        self.get_logger().info(f'Fetching from: {camera_url}')
        self.get_logger().info('Publishing to: /image')
        self.get_logger().info('='*50)

        # Test connection
        if not self.test_connection():
            self.get_logger().error('Cannot connect to camera server!')
            self.get_logger().error('Make sure Windows camera is running')
            raise RuntimeError('Camera server not accessible')

        # Create timer for 30 FPS
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.frame_count = 0
        self.error_count = 0

    def test_connection(self):
        """Test if we can reach the camera server"""
        try:
            self.get_logger().info('Testing connection to camera server...')
            response = urllib.request.urlopen(self.camera_url, timeout=2)
            self.get_logger().info('✓ Connection successful!')
            return True
        except Exception as e:
            self.get_logger().error(f'✗ Connection failed: {e}')
            return False

    def timer_callback(self):
        try:
            # Fetch frame from HTTP server
            response = urllib.request.urlopen(self.camera_url, timeout=1)
            img_array = np.asarray(bytearray(response.read()), dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().error('Failed to decode frame')
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
            self.error_count = 0  # Reset error count on success

            if self.frame_count % 300 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')

        except Exception as e:
            self.error_count += 1
            if self.error_count % 30 == 1:  # Log every 30 errors (once per second)
                self.get_logger().error(f'Error fetching frame: {e}')


def main(args=None):
    import sys
    import subprocess

    print()
    print("="*50)
    print("HTTP Camera Bridge for ROS2")
    print("="*50)
    print()
    print("This bridge fetches frames from Windows camera")
    print("and publishes them to ROS2 /image topic")
    print()
    print("Make sure Windows camera is running first!")
    print("Press Ctrl+C to stop")
    print()

    # Auto-detect Windows host IP
    try:
        result = subprocess.run(['cat', '/etc/resolv.conf'], capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'nameserver' in line:
                windows_ip = line.split()[1]
                break
        else:
            windows_ip = 'localhost'

        print(f"Detected Windows host IP: {windows_ip}")
        print()
    except:
        windows_ip = 'localhost'

    # Allow command line override
    camera_url = f'http://{windows_ip}:8080/frame.jpg'
    if len(sys.argv) > 1:
        camera_url = sys.argv[1]

    rclpy.init(args=args)

    try:
        node = HttpCameraBridge(camera_url)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nStopping bridge...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        rclpy.shutdown()
        print("Bridge stopped.")


if __name__ == '__main__':
    main()
