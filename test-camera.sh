#!/bin/bash

echo "=== Camera Detection Test ==="
echo ""

echo "1. Checking for camera devices..."
if ls /dev/video* &> /dev/null; then
    echo "✓ Found camera devices:"
    ls -la /dev/video*
else
    echo "✗ No camera devices found in /dev/video*"
    echo ""
    echo "This is expected in WSL2. Solutions:"
    echo "  - Run camera on Windows with ROS2"
    echo "  - Use USB/IP passthrough"
    echo "  - Continue with burger mode (test patterns)"
fi

echo ""
echo "2. Testing camera access (5 seconds)..."
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source ~/robotics/install/setup.bash 2>/dev/null || true

timeout 5 ros2 run image_tools cam2image &> /tmp/camera-test.log &
PID=$!
sleep 6

if kill -0 $PID 2>/dev/null; then
    echo "✓ Camera node is running"
    kill $PID
else
    echo "✗ Camera node stopped (likely no camera access)"
fi

echo ""
echo "3. Camera test log:"
cat /tmp/camera-test.log | head -20

echo ""
echo "=== Recommendations ==="
if ls /dev/video* &> /dev/null; then
    echo "✓ Camera detected - just uncheck 'Burger Mode' in UI"
else
    echo "⚠ No camera in WSL2 - recommended solutions:"
    echo ""
    echo "Option A (Easiest): Run camera on Windows"
    echo "  1. Install ROS2 on Windows"
    echo "  2. Run: ros2 run image_tools cam2image"
    echo "  3. WSL2 will receive images automatically"
    echo ""
    echo "Option B: Use test mode (current setup)"
    echo "  - Keep 'Burger Mode' checked"
    echo "  - Uses animated test patterns"
    echo "  - Fully functional for testing proofs"
fi
