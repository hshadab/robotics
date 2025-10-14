# Automated Windows Camera Setup for zkML Demo
# Run this in PowerShell as Administrator

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Windows Camera Setup for zkML Demo" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Step 1: Create directory
Write-Host "[1/5] Creating directory..." -ForegroundColor Yellow
New-Item -Path "C:\zkml-camera" -ItemType Directory -Force | Out-Null
cd C:\zkml-camera

# Step 2: Install required Python packages
Write-Host "[2/5] Installing Python packages (opencv, rclpy)..." -ForegroundColor Yellow
python -m pip install --upgrade pip --quiet
python -m pip install opencv-python rclpy sensor-msgs-py --quiet

if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Failed to install Python packages" -ForegroundColor Red
    Write-Host "Make sure Python is installed and in PATH" -ForegroundColor Red
    pause
    exit 1
}

# Step 3: Create Python camera publisher script
Write-Host "[3/5] Creating camera publisher script..." -ForegroundColor Yellow

$cameraScript = @'
#!/usr/bin/env python3
"""
Simple Windows Camera Publisher for ROS2
Publishes webcam frames to /image topic
"""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys

class WindowsCameraPublisher(Node):
    def __init__(self, camera_id=0):
        super().__init__('windows_camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.bridge = CvBridge()

        # Try to open camera
        self.cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {camera_id}')
            sys.exit(1)

        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.get_logger().info(f'Camera {camera_id} opened successfully')
        self.get_logger().info('Publishing to /image topic at 30 FPS')

        # Create timer for 30 FPS
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.frame_count = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.publisher_.publish(msg)

            self.frame_count += 1
            if self.frame_count % 300 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
        else:
            self.get_logger().error('Failed to read frame')

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    print("Starting Windows Camera Publisher...")
    print("Press Ctrl+C to stop")

    rclpy.init(args=args)

    # Try camera IDs in order (0, 1, 2)
    camera_id = 0
    if len(sys.argv) > 1:
        camera_id = int(sys.argv[1])

    node = WindowsCameraPublisher(camera_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping camera...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'@

$cameraScript | Out-File -FilePath "camera_publisher.py" -Encoding UTF8

# Step 4: Install cv_bridge (might need special handling)
Write-Host "[4/5] Installing cv_bridge..." -ForegroundColor Yellow
python -m pip install opencv-python-headless --quiet

# Try to install cv_bridge from source if package not available
$cvBridgeCode = @'
import cv2
import numpy as np

class CvBridge:
    def cv2_to_imgmsg(self, cv_image, encoding='bgr8'):
        from sensor_msgs.msg import Image
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = encoding
        img_msg.step = cv_image.shape[1] * cv_image.shape[2] if len(cv_image.shape) > 2 else cv_image.shape[1]
        img_msg.data = cv_image.tobytes()
        return img_msg
'@

$cvBridgeCode | Out-File -FilePath "cv_bridge.py" -Encoding UTF8

# Step 5: Create launcher batch file
Write-Host "[5/5] Creating launcher..." -ForegroundColor Yellow

$launcherBat = @'
@echo off
echo ========================================
echo Windows Camera for zkML Robotics Demo
echo ========================================
echo.
echo Starting camera publisher...
echo Your webcam light should turn on now.
echo Press Ctrl+C to stop.
echo.

set ROS_DOMAIN_ID=0
python camera_publisher.py

pause
'@

$launcherBat | Out-File -FilePath "start_camera.bat" -Encoding ASCII

# Create PowerShell launcher too
$launcherPs1 = @'
$env:ROS_DOMAIN_ID = "0"
Write-Host "Starting Windows Camera Publisher..." -ForegroundColor Green
python camera_publisher.py
'@

$launcherPs1 | Out-File -FilePath "start_camera.ps1" -Encoding UTF8

# Final instructions
Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "Setup Complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "Camera setup files created in: C:\zkml-camera\" -ForegroundColor Cyan
Write-Host ""
Write-Host "To start your camera:" -ForegroundColor Yellow
Write-Host "  Option 1: Double-click C:\zkml-camera\start_camera.bat" -ForegroundColor White
Write-Host "  Option 2: Run: cd C:\zkml-camera; python camera_publisher.py" -ForegroundColor White
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "  1. Run start_camera.bat (your webcam light should turn on)" -ForegroundColor White
Write-Host "  2. Open http://localhost:9200 in WSL2" -ForegroundColor White
Write-Host "  3. Uncheck 'Burger Mode'" -ForegroundColor White
Write-Host "  4. Click 'Start Full Demo'" -ForegroundColor White
Write-Host ""

# Create desktop shortcut
$WshShell = New-Object -ComObject WScript.Shell
$Shortcut = $WshShell.CreateShortcut("$env:USERPROFILE\Desktop\Start zkML Camera.lnk")
$Shortcut.TargetPath = "C:\zkml-camera\start_camera.bat"
$Shortcut.WorkingDirectory = "C:\zkml-camera"
$Shortcut.Description = "Start camera for zkML robotics demo"
$Shortcut.Save()

Write-Host "Desktop shortcut created: 'Start zkML Camera'" -ForegroundColor Green
Write-Host ""
Write-Host "Press any key to test the camera now..." -ForegroundColor Yellow
pause

# Test camera
Write-Host ""
Write-Host "Testing camera..." -ForegroundColor Yellow
cd C:\zkml-camera
python camera_publisher.py
