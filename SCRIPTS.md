# Root Directory Scripts

Scripts in the repository root for quick access and compatibility with existing workflows.

## Demo Control Scripts

### start_demo.sh
**Primary demo launcher**

Comprehensive startup script that orchestrates the entire demo:
- Checks dependencies (ROS 2, Node.js, Python packages)
- Builds ROS workspace if needed
- Starts onnx-verifier service
- Starts camera publisher (auto-detects Linux/WSL)
- Launches zkml_guard node with twist_mux
- Starts web UI server
- Opens browser to demo interface

**Usage**:
```bash
./start_demo.sh
```

**Status**: Actively maintained - primary entry point for demos

---

### stop_demo.sh
**Demo shutdown script**

Cleanly stops all demo services:
- Stops web UI server
- Stops zkml_guard ROS node
- Stops camera publisher
- Stops onnx-verifier service
- Cleans up PID files

**Usage**:
```bash
./stop_demo.sh
```

**Status**: Actively maintained

---

### start_linux_camera.sh
**Camera launcher helper**

Starts appropriate camera script based on platform:
- Detects WSL vs native Linux
- Launches linux_camera_publisher.py or http_camera_bridge.py
- Used by start_demo.sh internally

**Usage**:
```bash
./start_linux_camera.sh
```

**Status**: Actively maintained (called by start_demo.sh)

---

## Camera Scripts

See `CAMERA_SCRIPTS.md` for detailed documentation.

### linux_camera_publisher.py
Native Linux camera ROS publisher

**Usage**: `python3 linux_camera_publisher.py`

---

### http_camera_bridge.py
WSL camera bridge (Windows host → ROS 2)

**Usage**: `python3 http_camera_bridge.py`

**Prerequisites**: Run `simple_windows_camera.py` on Windows host first

---

### simple_windows_camera.py
Windows HTTP camera server

**Usage** (on Windows): `python simple_windows_camera.py`

**Serves**: HTTP MJPEG stream at http://localhost:5000

---

### test_all_cameras.py
Camera testing utility

**Usage**: `python3 test_all_cameras.py`

**Purpose**: Enumerate and test available camera devices

---

## Testing Scripts

### test-camera.sh
Quick camera test script

**Usage**: `./test-camera.sh`

**Purpose**: Verify camera is accessible and working

---

## Why Scripts Are in Root

These scripts are kept in the repository root for several reasons:

1. **Quick Access**: Demo users can run `./start_demo.sh` immediately after cloning
2. **Historical Paths**: Existing documentation and workflows reference these locations
3. **Cross-References**: Scripts call each other using relative paths (moving would break)
4. **Demo Convention**: Many robotics demos follow this pattern (start script in root)

## Organization Notes

Additional utility scripts are organized in subdirectories:
- `scripts/` - ROS environment setup, legacy launchers
- `tools/robotics-ui/` - Web UI server and proxy scripts
- `tools/onnx-verifier/` - Verifier service scripts

## Usage Patterns

### Standard Demo Workflow
```bash
# Start everything
./start_demo.sh

# Stop everything
./stop_demo.sh
```

### Manual Control
```bash
# Start components individually
./start_linux_camera.sh              # Camera only
cd tools/onnx-verifier && npm start  # Verifier only
cd tools/robotics-ui && npm start    # UI only

# Or use web UI controls at http://localhost:9200
```

### Testing
```bash
# Test camera access
python3 test_all_cameras.py

# Test camera ROS integration
./test-camera.sh
```

## Maintenance Notes

### Modifying Scripts

When editing these scripts:
1. Maintain backward compatibility (paths, arguments)
2. Test both WSL and native Linux if applicable
3. Update this documentation
4. Check for cross-references in other scripts

### Adding New Scripts

For new functionality:
- Demo control scripts → Repository root
- Utility scripts → `scripts/` directory
- Service-specific → Appropriate `tools/` subdirectory
- Camera-related → Document in `CAMERA_SCRIPTS.md`

### Script Dependencies

Visual dependency graph:
```
start_demo.sh
├── start_linux_camera.sh
│   ├── linux_camera_publisher.py (native Linux)
│   └── http_camera_bridge.py (WSL)
│       └── simple_windows_camera.py (Windows host)
├── tools/onnx-verifier/server.js
├── tools/robotics-ui/server.js
└── ROS 2 launch files
```

## Troubleshooting

### Script won't execute
```bash
chmod +x *.sh
```

### ROS environment not found
```bash
source scripts/setup_ros.sh
./start_demo.sh
```

### Camera script fails
```bash
# Test camera first
python3 test_all_cameras.py

# Check detailed docs
cat CAMERA_SCRIPTS.md
```

### Dependencies missing
start_demo.sh checks and reports missing dependencies. Install as directed.

## Future Improvements

Potential enhancements (non-breaking):
- Add `--dry-run` flag to start_demo.sh
- Add `--camera-only` mode
- Add status checking script (show running services)
- Add log collection script for debugging
- Add dependency installation script
