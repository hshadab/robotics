#!/bin/bash

echo "========================================"
echo "Linux Camera Publisher for zkML Demo"
echo "========================================"
echo ""

# Source ROS2 environment
echo "Setting up ROS2 environment..."
source /opt/ros/jazzy/setup.bash
source ~/robotics/install/setup.bash

# Set ROS domain
export ROS_DOMAIN_ID=0

echo ""
echo "Starting camera publisher..."
echo "This will attempt to use /dev/video0"
echo ""
echo "If you see an error about camera not found:"
echo "  - This is expected in WSL2 (no USB passthrough)"
echo "  - Use Windows camera setup instead"
echo "  - OR continue with burger mode (test patterns)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Make script executable
chmod +x ~/robotics/linux_camera_publisher.py

# Run camera publisher
python3 ~/robotics/linux_camera_publisher.py
