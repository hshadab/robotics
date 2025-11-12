# Camera Scripts Guide

This repository includes multiple camera scripts for different platforms and use cases. Use the appropriate script based on your operating system and setup.

## Available Scripts

### 1. `linux_camera_publisher.py`
**Recommended for Linux systems**

- **Platform**: Linux (native)
- **Purpose**: Direct camera access using OpenCV on Linux
- **Requirements**: opencv-python package
- **Usage**:
  ```bash
  python3 linux_camera_publisher.py
  ```
- **When to use**:
  - Running on native Linux
  - Direct access to /dev/video0 or similar
  - Best performance on Linux systems

### 2. `http_camera_bridge.py`
**For Windows Subsystem for Linux (WSL)**

- **Platform**: WSL with Windows host camera
- **Purpose**: Bridge between Windows HTTP camera server and ROS 2
- **Requirements**: opencv-python, requests
- **Usage**:
  ```bash
  # On Windows host, first run:
  python simple_windows_camera.py

  # Then in WSL:
  python3 http_camera_bridge.py
  ```
- **When to use**:
  - Running ROS 2 in WSL but camera is on Windows host
  - Cannot access camera device directly from WSL
  - Need to bridge HTTP stream to ROS topics

### 3. `simple_windows_camera.py`
**Windows-only HTTP camera server**

- **Platform**: Windows (native Python)
- **Purpose**: Expose Windows camera as HTTP MJPEG stream
- **Requirements**: opencv-python, Windows system
- **Usage**:
  ```bash
  python simple_windows_camera.py
  ```
- **When to use**:
  - Running on Windows host
  - Need to serve camera to WSL or network clients
  - Works in conjunction with http_camera_bridge.py

### 4. `test_all_cameras.py`
**Utility for testing camera devices**

- **Platform**: Any (Linux, Windows, macOS)
- **Purpose**: Enumerate and test available camera devices
- **Usage**:
  ```bash
  python3 test_all_cameras.py
  ```
- **When to use**:
  - Troubleshooting camera access
  - Finding correct camera device index
  - Verifying OpenCV can access cameras

## Quick Selection Guide

| Your Setup | Use This Script | Additional Steps |
|------------|-----------------|------------------|
| Native Linux | `linux_camera_publisher.py` | None |
| WSL 2 | `http_camera_bridge.py` | Run `simple_windows_camera.py` on Windows host first |
| Windows | `simple_windows_camera.py` | For serving camera only |
| Debugging | `test_all_cameras.py` | None |

## Troubleshooting

### Camera not found
1. Run `test_all_cameras.py` to list available devices
2. Check device permissions: `ls -l /dev/video*`
3. Ensure opencv-python is installed: `pip install opencv-python>=4.8.0`

### WSL camera access fails
WSL 2 cannot directly access USB devices. Use the HTTP bridge approach:
1. Run `simple_windows_camera.py` on Windows (check Windows Python has opencv)
2. Run `http_camera_bridge.py` in WSL (connects to http://localhost:5000)
3. Verify Windows firewall allows localhost connections

### Low framerate
- Adjust camera resolution in script (default: 640x480)
- Check CPU usage (ONNX inference may compete for resources)
- Consider reducing inference frequency in zkml_guard config

## Integration with Demo

The demo launcher (`start_demo.sh`) automatically:
1. Detects if running in WSL or native Linux
2. Starts appropriate camera script
3. Publishes frames to `/image` ROS topic
4. Handled by frame_proxy.py (part of robotics-ui)

Manual camera startup is usually not needed when using `start_demo.sh`.
