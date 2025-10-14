# JOLT Atlas zkML Guard — ROS 2 Robotics Demo

> Zero-Knowledge Machine Learning proofs gating robot motion commands

## Overview

This demo integrates JOLT Atlas zero-knowledge proofs with ROS 2 to create a trustworthy robotics control system:

- **Camera feed** (`/image`) → **zkML Guard** runs ONNX inference (MobileNetV2)
- **ZK Proof Generation**: Proves argmax classification on preprocessed tensor
- **Motion Gating**: Only releases `/zkml/stop` lock after verified proof
- **twist_mux** enforces high-priority lock on `/cmd_vel` commands
- **Web UI**: Real-time monitoring and control at http://localhost:9200

## Quick Start (Web UI)

**Prerequisites:**
- ROS 2 (Jazzy/Humble/Rolling)
- Node.js 18+ and npm
- Python 3.9+ with ROS 2 bindings
- Docker (optional - for building JOLT prover binary)
- Camera device (or use burger_mode for test pattern)

```bash
# 1. Build ROS workspace
cd ~/robotics
colcon build
source install/setup.bash

# 2. Install Python dependencies
pip install -r src/zkml_guard/requirements.txt

# 3. Install ONNX verifier dependencies
cd tools/onnx-verifier
npm install
cd ../..

# 4. Install UI dependencies
cd tools/robotics-ui
npm install
cd ../..

# 5. Start the Web UI (this will auto-start the verifier and proxies)
cd tools/robotics-ui
npm start
```

Then open http://localhost:9200/demo.html and click **"Start Full Demo"**

The UI automatically:
- Starts HTTP verifier (port 9100)
- Launches camera node
- Starts zkML guard with proof verification
- Starts demo teleop publisher
- Manages ROS proxy bridges for UI updates

## What's Included

### Core Components

1. **zkml_guard** (`src/zkml_guard/`): ROS 2 Python package
   - Subscribes to `/image`, runs ONNX inference
   - Generates/verifies ZK proofs via CLI or HTTP
   - Publishes `/zkml/stop` (Bool) and `/zkml/event` (String/JSON)

2. **Web UI** (`tools/robotics-ui/`): Express.js server + static frontend
   - Control panel for starting/stopping all components
   - Live event stream from `/zkml/event`
   - Camera frame preview
   - Proof status and lock state indicators
   - Debug endpoints: `/api/proxy_status`, `/debug/logs`

3. **ONNX Verifier** (`tools/onnx-verifier/`): HTTP zkML verification service
   - Accepts model + input, returns cryptographic proof
   - Used by default (HTTP mode)

4. **ROS Proxies** (`tools/robotics-ui/*_proxy.py`): Bridge ROS → Filesystem
   - `event_proxy.py`: `/zkml/event` → `/tmp/rdemo-last-event.json`
   - `frame_proxy.py`: `/image` → `/tmp/rdemo-frame.png`
   - Auto-started by UI server with retry logic

### File Structure

```
robotics/
├── src/zkml_guard/          # ROS 2 package
│   ├── zkml_guard/
│   │   ├── zkml_guard_node.py   # Main guard node
│   │   └── jolt.py              # Proof helper
│   ├── launch/                  # Launch files
│   ├── config/                  # YAML configs
│   └── requirements.txt
├── tools/
│   ├── robotics-ui/         # Web control panel
│   │   ├── server.js            # Express API server
│   │   ├── event_proxy.py       # ROS→File bridge
│   │   ├── frame_proxy.py       # Camera→File bridge
│   │   └── public/index.html    # Frontend UI
│   ├── onnx-verifier/       # HTTP verifier service
│   └── ui/demo_ui.py        # Legacy Tkinter UI
├── scripts/
│   ├── setup_ros.sh         # Install ROS + deps
│   └── run_demo_ui.sh       # Launch with venv
└── README.md
```

## Manual Setup (Without Web UI)

### 1. Install ROS 2 Packages

```bash
sudo apt install \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-twist-mux \
  ros-$ROS_DISTRO-image-tools \
  ros-$ROS_DISTRO-rosbag2-storage-mcap
```

### 2. Build Workspace

```bash
cd ~/robotics
colcon build
source install/setup.bash
```

### 3. Install Python Dependencies

```bash
# In a venv (recommended)
python3 -m venv venv
source venv/bin/activate
pip install -r src/zkml_guard/requirements.txt

# Or system-wide
pip install onnxruntime numpy pillow requests rclpy
```

### 4. Run Components (3 terminals)

**Terminal 1: Camera**
```bash
source ~/robotics/install/setup.bash
ros2 run image_tools cam2image
```

**Terminal 2: zkML Guard + twist_mux**
```bash
source ~/robotics/install/setup.bash
# HTTP mode (default - requires verifier running)
ros2 launch zkml_guard zkml_guard_proof.launch.py

# Or CLI mode (requires atlas_argmax_prover in PATH)
ros2 launch zkml_guard zkml_guard_proof.launch.py --ros-args -p verifier_mode:=cli
```

**Terminal 3: Teleop (interactive keyboard control)**
```bash
source ~/robotics/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 5. Optional: Start HTTP Verifier

```bash
cd ~/robotics/tools/onnx-verifier
npm install  # first time only
node server.js
# Listens on http://localhost:9100
```

## Configuration

### Key Parameters (`src/zkml_guard/config/zkml_guard.params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `verifier_mode` | `cli` | `cli` or `http` |
| `verifier_url` | - | HTTP endpoint (e.g., `http://localhost:9100/verify`) |
| `gating_mode` | `argmax` | Proof mode: `argmax` or `threshold` |
| `threshold` | `0.6` | Min confidence score |
| `prove_on` | `rising_edge` | `rising_edge` or `every_pass` |
| `unlock_hold_ms` | `1200` | Keep unlocked for N ms after proof |
| `require_proof` | `true` | Gate motion on proof verification |
| `model_path` | (auto) | ONNX model path (auto-downloads MobileNetV2) |
| `prove_cmd_template` | (see yaml) | CLI command template with placeholders |

Override at launch:
```bash
ros2 launch zkml_guard zkml_guard_proof.launch.py \
  --ros-args -p verifier_mode:=http \
             -p verifier_url:=http://localhost:9100/verify \
             -p threshold:=0.7
```

## Web UI Endpoints

Server runs on http://localhost:9200

- **GET** `/` or `/demo.html` → Enhanced demo UI
- **GET** `/status` → Component status (camera, guard, proxies, etc.)
- **GET** `/api/proxy_status` → Detailed proxy health + logs
- **GET** `/api/last_event` → Latest `/zkml/event` JSON
- **GET** `/api/stop_state` → Lock state (`/zkml/stop`)
- **GET** `/api/frame.png` → Latest camera frame
- **GET** `/api/events` → Server-Sent Events stream
- **POST** `/start/{service}` → Start verifier/camera/guard/teleop/bag
- **POST** `/stop/{service}` → Stop service
- **GET** `/start/full?mode=http&burger=1&record=0` → One-shot demo start
- **GET** `/stop/all` → Stop all services
- **GET** `/start/proxies` → Restart ROS proxies
- **GET** `/debug/logs` → Proxy log tails
- **GET** `/debug/files` → Temp file status

## Troubleshooting

### UI Not Showing ROS Data

1. **Check proxy status:**
   ```bash
   curl http://localhost:9200/api/proxy_status
   ```

2. **Manually restart proxies:**
   ```bash
   cd ~/robotics/tools/robotics-ui
   source ~/robotics/install/setup.bash
   source /opt/ros/jazzy/setup.bash
   python3 event_proxy.py &
   python3 frame_proxy.py &
   ```

3. **Check logs:**
   ```bash
   tail -f /tmp/event_proxy.py.log
   tail -f /tmp/frame_proxy.py.log
   ```

4. **Verify ROS topics are publishing:**
   ```bash
   ros2 topic list
   ros2 topic echo /zkml/event --once
   ros2 topic hz /image
   ```

### Proof Failures

- **CLI mode**: Ensure `atlas_argmax_prover` is in PATH and executable
- **HTTP mode**: Verify verifier is running on port 9100
- **Check logs**: `ros2 topic echo /zkml/event` shows `proof_verified: false` with error details

### Camera Not Working

```bash
# Check if camera device exists
ls -l /dev/video*

# Test camera directly
ros2 run image_tools cam2image --ros-args -p burger_mode:=true

# Check topic
ros2 topic hz /image
```

## Recording & Visualization

### Record to MCAP

```bash
ros2 bag record -a -s mcap
# Or via UI: enable "Record MCAP" checkbox before starting
```

### View in Foxglove

1. Open Foxglove Studio
2. Connect to ROS 2 or open `.mcap` file
3. Add panels for:
   - `/image` (Image)
   - `/zkml/stop` (Boolean indicator)
   - `/zkml/event` (Raw Messages)
   - `/cmd_vel` and `/cmd_vel_out` (Twist)

## Architecture Notes

### Proof Flow

1. Camera publishes `/image` at ~30 Hz
2. `zkml_guard_node` samples at 500ms (configurable)
3. Runs ONNX inference (MobileNetV2) on preprocessed frame
4. If predicate met (threshold + argmax):
   - CLI mode: Spawns `atlas_argmax_prover` subprocess
   - HTTP mode: POST to verifier with model + input tensor
5. Parses proof result from JSON stdout/response
6. Updates `/zkml/stop`:
   - `true` (locked) if no proof or proof failed
   - `false` (unlocked) if proof verified
7. Publishes full event metadata to `/zkml/event`

### Proxy Bridge Pattern

The UI server cannot directly access ROS topics (different process context). Solution:

1. **Proxy processes** (`event_proxy.py`, `frame_proxy.py`) run as ROS nodes
2. Subscribe to topics and write to temp files in `/tmp/`
   - `/tmp/rdemo-last-event.json` - Latest zkML event
   - `/tmp/rdemo-stop.txt` - Lock state (true/false)
   - `/tmp/rdemo-frame.png` - Latest camera frame
3. UI server polls/serves these files via HTTP
4. Server auto-starts proxies with retry logic and logging to `/tmp/*_proxy.py.log`

## Building JOLT Prover (CLI Mode)

```bash
cd ~/robotics/tools
./build_helper.sh
# Builds Docker image, compiles Rust prover, extracts binary to bin/

# Add to PATH
export PATH="$HOME/robotics/tools/bin:$PATH"

# Verify
atlas_argmax_prover --help
```

Target specific jolt-atlas branch:
```bash
./build_helper.sh atlas-helper:latest <git-ref>
```

## Safety & Security

- **Fail-safe**: Lock engages if proof generation fails or times out
- **No blind trust**: Motion only allowed after cryptographic verification
- **Reproducible**: All inputs (model hash, tensor hash) logged in `/zkml/event`
- **Auditable**: MCAP recordings capture full provenance chain

## Legacy UI (Tkinter)

Alternative GUI launcher:
```bash
sudo apt install python3-tk
python3 tools/ui/demo_ui.py
# Or: scripts/run_demo_ui.sh
```

## License & Credits

- JOLT Atlas: [ICME-Lab/jolt-atlas](https://github.com/ICME-Lab/jolt-atlas)
- ROS 2: Apache 2.0
- This demo: MIT
