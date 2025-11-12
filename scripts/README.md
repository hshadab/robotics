# Scripts Directory

Utility scripts for setting up and running the zkML robotics demo.

## Available Scripts

### setup_ros.sh
**Purpose**: ROS 2 environment setup

Sets up ROS 2 environment variables and sources workspace.

**Usage**:
```bash
source scripts/setup_ros.sh
```

**What it does**:
- Sources ROS 2 installation (`/opt/ros/jazzy/setup.bash`)
- Sources workspace overlay (`install/setup.bash`)
- Sets necessary environment variables

**When to use**:
- Before running any ROS 2 commands
- In new terminal sessions
- After building workspace with `colcon build`

### run_demo_ui.sh
**Purpose**: Legacy UI launcher (DEPRECATED)

Launches the old Tkinter-based demo UI.

**Status**: Deprecated - replaced by web UI

**Migration**: Use `start_demo.sh` in root directory instead:
```bash
cd ~/robotics
./start_demo.sh
```

The new web UI provides better functionality and runs at http://localhost:9200.

## Root Directory Scripts

Additional scripts in the repository root:

### start_demo.sh
**Primary demo launcher** (use this one!)

Comprehensive startup script that:
1. Checks dependencies (ROS 2, Node.js, Python packages)
2. Builds workspace if needed
3. Starts onnx-verifier service
4. Starts camera publisher
5. Launches zkml_guard node with twist_mux
6. Opens web UI in browser

**Usage**:
```bash
cd ~/robotics
./start_demo.sh
```

### Camera Scripts (in root)
See `CAMERA_SCRIPTS.md` for details on:
- `linux_camera_publisher.py` - Native Linux camera
- `http_camera_bridge.py` - WSL camera bridge
- `simple_windows_camera.py` - Windows camera server
- `test_all_cameras.py` - Camera testing utility

## Recommended Workflow

### First Time Setup
```bash
cd ~/robotics
source scripts/setup_ros.sh
colcon build
cd tools/onnx-verifier && npm install
cd ../robotics-ui && npm install
```

### Running the Demo
```bash
cd ~/robotics
./start_demo.sh  # This handles everything automatically
```

### Manual Service Control
If you need to start services individually:

```bash
# Terminal 1: ROS environment
source scripts/setup_ros.sh

# Terminal 2: Verifier service
cd tools/onnx-verifier && npm start

# Terminal 3: Camera
python3 linux_camera_publisher.py

# Terminal 4: Guard node
ros2 launch zkml_guard zkml_guard_proof.launch.py

# Terminal 5: Web UI
cd tools/robotics-ui && npm start
```

## Script Maintenance

### Adding New Scripts
When adding scripts to this directory:
1. Make executable: `chmod +x script_name.sh`
2. Add shebang: `#!/bin/bash`
3. Document in this README
4. Follow existing naming convention

### Deprecating Scripts
When deprecating:
1. Update this README with deprecation notice
2. Add migration instructions
3. Consider moving to `archive/` directory
4. Update any documentation that references it

## Notes

- All scripts assume repository root is `~/robotics`
- Scripts require bash (not sh)
- Set execute permissions: `chmod +x script_name.sh`
- Source scripts (with `source` command) for environment changes
- Execute scripts (with `./` or `bash`) for actions

## Troubleshooting

### "command not found" errors
Scripts may need execution permissions:
```bash
chmod +x scripts/*.sh
```

### ROS environment not set
Always source setup_ros.sh before ROS commands:
```bash
source scripts/setup_ros.sh
```

### Script paths not working
Ensure you're in the repository root:
```bash
cd ~/robotics
./scripts/script_name.sh
```
