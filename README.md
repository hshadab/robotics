# JOLT Atlas zkML Guard — ROS 2 Robotics Demo

> Zero-Knowledge Machine Learning proofs gating robot motion commands

## ⚠️ IMPORTANT: PROOF OF CONCEPT LIMITATION

**THIS IS A DEMONSTRATION SYSTEM - NOT PRODUCTION READY**

**Critical Limitation**: This demo uses a **sentinel model** for proof generation, NOT the full MobileNetV2 model used for inference decisions. Here's what this means:

- ✅ **What IS proven**: A small sentinel ONNX model (11-step execution trace) was executed correctly with cryptographic guarantees
- ❌ **What is NOT proven**: The MobileNetV2 inference that determines motion gating
- 🔗 **Binding mechanism**: SHA256 hash metadata only - NOT cryptographically binding the two computations

**Why this limitation exists**:
- Full MobileNetV2 JOLT proof would take **30-120 minutes** and require 10-50GB RAM
- Current JOLT technology cannot produce real-time proofs for production ML models
- Demo proves the proof system works, but NOT the actual inference used for robot control

**Security implications**:
- Hash binding (model_sha256, input_sha256) is metadata that can be forged
- An attacker could run malicious MobileNetV2 inference while proving a benign sentinel model
- The cryptographic proof and the safety-critical inference are **decoupled**

**For production use, you would need**:
- Hardware-accelerated proof generation (GPU provers)
- Incremental Verifiable Computation (IVC) / recursive SNARKs
- Proof aggregation to bind sentinel proof to full model output hash
- Or accept multi-minute latency for full model proofs

**This demo is valuable for**:
- Understanding zkML concepts and architecture
- Testing proof system integration with ROS 2
- Demonstrating cryptographic verification workflows
- Exploring zkML UX and developer experience

**This demo should NOT be used for**:
- Safety-critical robot control
- Production autonomous systems
- Any scenario where you need to cryptographically verify the actual ML inference

See [Smart Proof Design](#smart-proof-design) section for technical details.

---

## Overview

This project demonstrates **cryptographically verifiable robotics** by integrating JOLT-Atlas zero-knowledge proofs with ROS 2. It creates a trustworthy control system where robot motion requires proof that specific ML computations were executed correctly.

### System Flow

```
Camera → ML Inference → ZK Proof → Verified → Motion Unlocked
  ↓           ↓            ↓          ↓            ↓
/image   MobileNetV2   JOLT-Atlas  Crypto      /cmd_vel
         (10ms)       (3-4s)      (128-bit)   (gated)
```

**Key Components:**
- **Camera feed** (`/image`) → **zkML Guard** runs ONNX inference (MobileNetV2)
- **ZK Proof Generation**: JOLT-Atlas proves computation integrity with cryptographic guarantees
- **Motion Gating**: Only releases `/zkml/stop` lock after verified proof
- **twist_mux** enforces high-priority lock on `/cmd_vel` commands
- **Web UI**: Real-time monitoring and control at http://localhost:9200

### What zkML Provides

**Computational Integrity Guarantees:**
- Proves the **exact** model (by SHA256 hash) was used for inference
- Proves the **exact** input (by SHA256 hash) was processed
- Prevents model substitution, result forgery, or replay attacks
- Creates auditable proof chain for regulatory compliance

**Security Benefits:**
- **Trustless Operation**: Verify robot decisions without trusting the operator
- **Multi-Party Scenarios**: Multiple organizations can verify same robot used approved model
- **Tamper Detection**: Any modification to model weights or inference results is cryptographically detectable
- **Audit Trail**: Every decision includes cryptographic proof of what computation happened

This is essential for high-stakes robotics (autonomous vehicles, medical robots, defense systems) and multi-party deployments where trust cannot be assumed.

## Quick Start (Automated)

**One-command demo launch:**

```bash
cd ~/robotics
./start_demo.sh
```

This automatically:
- Checks and installs all dependencies
- Builds ROS workspace if needed
- Starts ONNX verifier (port 9100)
- Starts UI server with auto-proxy launch (port 9200)
- Launches camera, zkML guard, and teleop
- Opens browser to demo UI

**Options:**
```bash
./start_demo.sh --help          # Show all options
./start_demo.sh --cli           # Use CLI verifier instead of HTTP
./start_demo.sh --burger        # Use test pattern instead of camera
./start_demo.sh --record        # Record to MCAP file
./start_demo.sh --no-browser    # Don't auto-open browser
```

**Stop everything:**
```bash
./stop_demo.sh
```

## Manual Quick Start

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

## 📚 Documentation

Comprehensive documentation is available in the [`docs/`](docs/) directory:

- **[Quick Start Guide](docs/QUICKSTART.md)** - Fast setup instructions
- **[Architecture](docs/ARCHITECTURE.md)** - System design and components
- **[Camera Setup](docs/CAMERA_SCRIPTS.md)** - Multi-platform camera configuration
- **[Security Fixes](docs/SECURITY_FIXES.md)** - Security improvements and best practices
- **[Testing Guide](docs/TESTING.md)** - Testing procedures and verification
- **[Scripts Reference](docs/SCRIPTS.md)** - Root directory scripts documentation
- **[Development Log](docs/CLAUDE.md)** - Recent improvements and changes

See [`docs/README.md`](docs/README.md) for a complete documentation index.

## What's Included

### Core Components

1. **zkml_guard** (`src/zkml_guard/`): ROS 2 Python package
   - Subscribes to `/image`, runs ONNX inference
   - Generates/verifies ZK proofs via CLI or HTTP
   - Publishes `/zkml/stop` (Bool) and `/zkml/event` (String/JSON)

2. **Web UI** (`tools/robotics-ui/`): Express.js server + static frontend
   - Control panel for starting/stopping all components
   - Live event stream from `/zkml/event`
   - Camera frame preview with real-time detection display
   - Proof status and lock state indicators
   - Visual pipeline display showing inference → proof → verification states
   - Verified proofs history with snapshot thumbnails
   - Motion gating countdown timer
   - Debug endpoints: `/api/proxy_status`, `/debug/logs`

3. **ONNX Verifier** (`tools/onnx-verifier/`): HTTP zkML verification service
   - Accepts model + input, returns cryptographic proof
   - Used by default (HTTP mode)

4. **ROS Proxies** (`tools/robotics-ui/*_proxy.py`): Bridge ROS → Filesystem
   - `event_proxy.py`: `/zkml/event` → `/tmp/rdemo-last-event.json`, manages verified proofs history
   - `frame_proxy.py`: `/image` → `/tmp/rdemo-frame.png`
   - Snapshot saving: Captures frame images for verified proofs with retry logic
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

## Web UI Features

### Real-Time Detection Display

The UI shows live ML inference results with intelligent state management:

**Detection Behavior:**
- **Guard Running**: Shows continuous detection results (label + confidence %)
- **During Proof Generation**: Clears detection display to indicate proof processing
- **After Proof**: Resumes showing detections after inference pause (~2 seconds)
- **Guard Stopped**: Immediately clears all detection data for clean restart

**State Machine:**
1. **Stopped** → Clear display, clear internal state (`lastEvent = null`)
2. **Running → No threshold met** → Show all detections continuously
3. **Running → Threshold met** → Clear display (proof generating)
4. **Running → Proof complete** → Resume showing detections after inference pause

This ensures users see fresh, relevant inference data without confusing stale results.

### Verified Proofs History

- Displays last 50 verified proofs with snapshot thumbnails
- Each proof shows: timestamp, detected label, confidence score, proof ID
- Snapshots automatically captured at moment of proof verification
- Reliable snapshot saving with 500ms retry mechanism
- Updates immediately when new proofs are verified

### Motion Gating Timer

- Large, prominent countdown display showing time until motion lock re-engages
- Shows remaining seconds from `unlock_hold_ms` parameter
- Pauses during proof generation
- Resets when new proof verified

### Pipeline Visualization

Real-time visual feedback showing system state:
- **Inference** → Running/idle indicator
- **Proof Generation** → Active/inactive with timing
- **Verification** → Success/failure status

Color-coded states make it easy to understand what the system is doing at any moment.

## Web UI Endpoints

Server runs on http://localhost:9200

- **GET** `/` or `/demo.html` → Enhanced demo UI
- **GET** `/status` → Component status (camera, guard, proxies, etc.)
- **GET** `/api/proxy_status` → Detailed proxy health + logs
- **GET** `/api/last_event` → Latest `/zkml/event` JSON
- **GET** `/api/stop_state` → Lock state (`/zkml/stop`)
- **GET** `/api/frame.png` → Latest camera frame
- **GET** `/api/verified_proofs` → History of last 50 verified proofs
- **GET** `/api/snapshot/:id.png` → Snapshot image for specific proof ID
- **GET** `/api/events` → Server-Sent Events stream
- **POST** `/start/{service}` → Start verifier/camera/guard/teleop/bag
- **POST** `/stop/{service}` → Stop service
- **GET** `/start/full?mode=http&burger=1&record=0` → One-shot demo start
- **GET** `/stop/all` → Stop all services
- **GET** `/start/proxies` → Restart ROS proxies
- **GET** `/debug/logs` → Proxy log tails
- **GET** `/debug/files` → Temp file status
- **POST** `/api/flip_model` → Demo: Swap model to tampered version (for testing detection)
- **POST** `/api/restore_model` → Demo: Restore original model from backup

## Recent Security & UI Improvements

### Model Tampering Detection (2025-10-16)

**Problem**: The guard node uploaded the tampered model to the verifier, which then computed the hash of that tampered model. Since both used the tampered model, hash verification always passed - allowing proofs to succeed even with a substituted model.

**Root Cause**:
- Guard loaded model into memory at startup and computed hash once
- When model file was swapped on disk (via `/api/flip_model`), guard restarted with tampered model
- Guard uploaded tampered model to verifier → verifier computed tampered hash → everything matched
- No stored "expected" hash to compare against

**Solution**: Added runtime tampering detection in `zkml_guard_node.py:190-193, 308-317`:
1. **Store original hash** at startup: `self.original_model_sha256 = sha256_file(self.model_path)`
2. **Check before every proof**: Compare current file hash against stored original
3. **Block proof generation** if mismatch detected:
   ```python
   current_hash = sha256_file(self.model_path)
   if current_hash != self.original_model_sha256:
       self.get_logger().error('MODEL TAMPERING DETECTED!')
       proof_verified = False
       proof_id = 'BLOCKED_TAMPERED_MODEL'
   ```

**Result**:
- ✅ Proofs **immediately blocked** when model file is modified
- ✅ Motion stays locked (red) - no unsafe operation
- ✅ Clear error logs: `MODEL TAMPERING DETECTED! Expected: c0c3f76d... | Actual: b9cacfef...`
- ✅ UI shows red banner with hash mismatch details
- ✅ Demonstrates zkML's tamper detection capabilities in live demo

**Testing the Feature**:
```bash
# 1. Start demo normally - model hash registered at startup
curl http://localhost:9200/start/full

# 2. Flip model to tampered version (changes 3 bytes in middle of file)
curl -X POST http://localhost:9200/api/flip_model

# 3. Try to generate proof - BLOCKED with error:
# [zkml_guard] MODEL TAMPERING DETECTED! Expected: c0c3f76d... | Actual: b9cacfef...
# [zkml_guard] Proof generation BLOCKED - model hash mismatch

# 4. Restore original model
curl -X POST http://localhost:9200/api/restore_model

# 5. Proofs work normally again
```

**API Endpoints**:
- **POST** `/api/flip_model` → Swaps model with tampered version, returns hash comparison
- **POST** `/api/restore_model` → Restores original model from backup

**Files Modified**:
- `/home/hshadab/robotics/src/zkml_guard/zkml_guard/zkml_guard_node.py` - Added tampering detection
- `/home/hshadab/robotics/tools/robotics-ui/server.js` - Model swap API endpoints
- `/home/hshadab/robotics/tools/robotics-ui/public/demo.html` - UI controls and banner
- `/home/hshadab/robotics/tools/robotics-ui/public/index.html` - Synced with demo.html

### Detection Display State Management (2025-10-16)

**Problem**: Detection display showed stale inference results, causing confusion:
- Old detections remained visible when demo was stopped
- Detection data persisted briefly when restarting demo
- Labels shown even when below threshold during proof generation

**Solution**: Implemented intelligent state machine in `demo.html:896-922`:
- Detection clears immediately when guard stops
- `lastEvent` set to `null` on stop for clean restart (`demo.html:1131-1136`)
- Detection clears during proof generation (when `predicate_met && !proof_ms`)
- Detection shows continuously when guard running (regardless of threshold)
- Detection resumes after inference pause completes

**Files Modified**: `/home/hshadab/robotics/tools/robotics-ui/public/demo.html`

### Reliable Snapshot Capture (2025-10-16)

**Problem**: Some verified proofs missing snapshot thumbnails (~33% failure rate):
- `frame_proxy.py` throttles writes (every 5th frame)
- `event_proxy.py` tried to copy frame immediately
- Race condition when frame file didn't exist yet

**Solution**: Added retry mechanism in `event_proxy.py:62-68`:
- Wait up to 500ms for frame file (5 attempts × 100ms)
- Log warning if frame still not available after retries
- Import `time` module for sleep functionality

**Result**: All verified proofs now consistently include snapshots

**Files Modified**: `/home/hshadab/robotics/tools/robotics-ui/event_proxy.py`

### Configuration Updates

**Threshold**: Changed from 25% to 40% in `zkml_guard.params.yaml:12`
**Inference Rate**: Changed to 2000ms (2 seconds) in `zkml_guard.params.yaml:13`

**Note**: These config changes require restarting the guard service to take effect:
```bash
# Via UI: Click "Stop All" then "Start Full Demo"
# Or manually restart guard node
```

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

## Architecture & Technology Stack

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Web Browser                             │
│                  http://localhost:9200/demo.html                │
└────────────────────────────┬────────────────────────────────────┘
                             │ HTTP/SSE
┌────────────────────────────▼────────────────────────────────────┐
│                    UI Server (Node.js/Express)                  │
│  - Control panel API                                            │
│  - File serving (/tmp/rdemo-*.{json,png,txt})                   │
│  - Service orchestration                                        │
└──────┬──────────────────────────────────────────┬───────────────┘
       │ HTTP                                      │ spawn/monitor
       │                                           │
┌──────▼──────────────────┐              ┌────────▼────────────────┐
│ ONNX Verifier (Node.js) │              │  ROS2 Proxy Scripts     │
│  - HTTP proof API       │              │  - event_proxy.py       │
│  - Spawns JOLT binary   │              │  - frame_proxy.py       │
│  - Returns proof JSON   │              │  - Write to /tmp/       │
└─────────────────────────┘              └────────┬────────────────┘
                                                   │ ROS2 pub/sub
                                         ┌─────────▼─────────────────┐
                                         │    ROS 2 Middleware       │
                                         │  (DDS - Data Distribution)│
                                         └─┬─────────┬───────────┬───┘
                                           │         │           │
                      ┌────────────────────▼─┐  ┌────▼──────┐ ┌─▼──────────┐
                      │ zkml_guard (Python)  │  │ twist_mux │ │ cam2image  │
                      │ - Subscribes: /image │  │ (C++)     │ │ (C++)      │
                      │ - ONNX inference     │  │           │ │            │
                      │ - Triggers proofs    │  │           │ │            │
                      │ - Publishes:         │  │           │ │            │
                      │   /zkml/event        │  │           │ │            │
                      │   /zkml/stop         │  │           │ │            │
                      └──────────────────────┘  └───────────┘ └────────────┘
```

### Technology Roles

| Technology | Role | Why This Choice |
|-----------|------|-----------------|
| **ROS 2** | Communication middleware | Industry-standard for robotics, pub/sub topics, QoS guarantees |
| **Node.js** | Web servers (UI + ONNX verifier) | Fast async I/O, easy HTTP APIs, child process management |
| **Python** | ROS2 nodes, proxies | `rclpy` bindings, `onnxruntime` support, rapid prototyping |
| **Rust** | JOLT-Atlas prover binary | Memory safety, cryptographic performance, JOLT implementation |
| **ONNX Runtime** | ML inference engine | Cross-platform, optimized for MobileNetV2, 10ms inference |
| **HTML/JS** | Frontend dashboard | Real-time updates via SSE, responsive UI, minimal dependencies |

### ROS 2 Components

**Nodes:**
- `zkml_guard` - Main guard node (Python)
  - Subscribes to `/image` (sensor_msgs/Image)
  - Publishes to `/zkml/event` (std_msgs/String - JSON)
  - Publishes to `/zkml/stop` (std_msgs/Bool)
  - Triggers HTTP proof generation

- `twist_mux` - Safety multiplexer (C++)
  - Subscribes to multiple `/cmd_vel_*` topics
  - Publishes to `/cmd_vel_out`
  - Locks on `/zkml/stop` signal

- `cam2image` - Camera publisher (C++)
  - Publishes to `/image` at ~30Hz
  - Optional burger_mode for test patterns

**Topics:**
- `/image` - Camera frames (sensor_msgs/Image)
- `/zkml/event` - Proof metadata (std_msgs/String - JSON)
- `/zkml/stop` - Motion lock (std_msgs/Bool)
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/cmd_vel_out` - Gated velocity output (geometry_msgs/Twist)

### Proof Flow (Detailed)

**Phase 1: Inference (10-50ms)**
1. Camera publishes `/image` at ~30 Hz
2. `zkml_guard_node` samples at configurable rate (default: 500ms)
3. Preprocesses frame: resize 224×224, normalize, convert to tensor
4. Runs ONNX inference (MobileNetV2) on CPU/GPU
5. Computes argmax and confidence score

**Phase 2: Proof Triggering (Conditional)**
6. If predicate met (threshold + gating_mode):
   - **HTTP mode**: POST to `http://localhost:9100/verify`
     - Sends model path, input tensor, metadata
     - Server spawns JOLT binary: `simple_jolt_proof`
   - **CLI mode**: Spawns `atlas_argmax_prover` subprocess directly

**Phase 3: Proof Generation (3-4 seconds)**
7. JOLT-Atlas prover:
   - Loads sentinel model (NOT full MobileNetV2 - that would take 30+ minutes)
   - Generates execution trace (11 steps for sentinel model)
   - Creates cryptographic proof using Dory polynomial commitments
   - **Proves**: "I correctly executed computation X on input Y"

**Phase 4: Verification (<1 second)**
8. JOLT binary verifies proof internally (6s)
   - Checks polynomial commitments
   - Validates execution trace
   - Returns verification result

**Phase 5: Motion Gating**
9. `zkml_guard_node` parses proof result
10. Updates `/zkml/stop`:
    - `true` (locked) if no proof or proof failed
    - `false` (unlocked) if proof verified
11. Publishes full event metadata to `/zkml/event`:
    ```json
    {
      "ts": 1760501733.95,
      "model_sha256": "c0c3f76d...",  // Binds to specific model
      "input_sha256": "bb92837d...",  // Binds to specific input
      "top1_label": "matchstick",
      "top1_score": 0.0256,
      "proof_verified": true,
      "proof_ms": 2847,
      "proof_id": "0x1a2b3c4d..."
    }
    ```
12. Motion unlocked for `unlock_hold_ms` duration (default: 3000ms)

### Proxy Bridge Pattern

The UI server cannot directly access ROS topics (different process context). Solution:

**Architecture:**
1. **Proxy processes** (`event_proxy.py`, `frame_proxy.py`) run as ROS nodes
2. Subscribe to topics and write to temp files in `/tmp/`
   - `/tmp/rdemo-last-event.json` - Latest zkML event (JSON)
   - `/tmp/rdemo-stop.txt` - Lock state (true/false)
   - `/tmp/rdemo-frame.png` - Latest camera frame (PNG)
   - `/tmp/rdemo-verified-proofs.json` - History of last 50 verified proofs
   - `/tmp/rdemo-snapshots/*.png` - Snapshot images for verified proofs
3. UI server polls/serves these files via HTTP
4. Server auto-starts proxies with retry logic and logging to `/tmp/*_proxy.py.log`

**Why This Design:**
- Node.js server runs outside ROS environment (no `rclpy` bindings)
- File-based IPC is simple, debuggable, and cross-process
- Proxies can crash/restart independently without affecting UI server
- Easy to inspect state: `cat /tmp/rdemo-last-event.json`

**Snapshot Reliability:**
- `event_proxy.py` waits up to 500ms for frame file when proof verified (5 retries × 100ms)
- Handles timing issue where `frame_proxy.py` throttles frame writes (every 5th frame)
- Ensures all verified proofs have associated snapshot images

### Smart Proof Design

**Current Implementation:**
- **Fast path**: ONNX inference on full MobileNetV2 (10ms) → immediate feedback
- **Slow path**: JOLT proof on sentinel model (3-4s) → cryptographic guarantee
- **Binding**: Both use same `input_sha256` and `model_sha256` in metadata

**Why Not Prove Full MobileNetV2?**
- Full model proof would take **30 minutes to 2+ hours**
- Requires 10-50GB RAM
- Trace length: 100M+ steps vs current 11
- Would completely break real-time demo

**Trade-off:**
- ✅ Real-time performance for normal operation
- ✅ Cryptographic audit trail with hash binding
- ✅ Safety gating that requires proof verification
- ⚠️ Proof is for sentinel computation, not full inference
- ⚠️ Full MobileNetV2 proof would require hardware acceleration (GPU prover)

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

## Why zkML Matters: Real-World ML Failures

### Common ML Failure Modes

**1. Adversarial Attacks**
- Adding imperceptible noise to images causes misclassification
- Example: Stop sign with stickers classified as speed limit sign
- Impact: Autonomous vehicle runs stop sign, robot misidentifies hazards

**2. Model Poisoning/Substitution**
- Attacker replaces model weights with backdoored version
- Malicious behavior triggered by specific patterns
- Impact: Robot behaves normally until trigger detected, then acts maliciously

**3. Distribution Shift**
- Model trained in lab fails in production environment
- Lighting changes, sensor degradation, unseen objects
- Impact: Confidence scores become unreliable, wrong decisions made

**4. Silent Hardware/Software Faults**
- Bit flips in GPU memory during inference
- Corrupted model files on disk
- Firmware vulnerabilities in inference engine
- Impact: Incorrect results with no error indication

### What zkML Solves (and Doesn't)

**zkML Provides:**
✅ **Computational Integrity** - Proves the exact computation happened
✅ **Model Authenticity** - Cryptographically binds to model SHA256
✅ **Input Binding** - Proves computation used specific input tensor
✅ **Tamper Detection** - Any modification breaks proof verification
✅ **Non-Repudiation** - Audit trail of what model was used when
✅ **Trustless Verification** - Anyone can verify without trusting operator

**zkML Does NOT Provide:**
❌ Makes model more accurate
❌ Prevents adversarial examples from fooling model
❌ Detects if model was trained on poisoned data
❌ Guarantees model is "correct" for the task

**Key Insight:** zkML proves "this computation happened correctly" not "this computation gave the right answer"

### Use Cases Where zkML is Essential

**High-Stakes Robotics:**
- **Autonomous vehicles** - Prove certified model used for safety decisions
- **Medical robots** - Regulatory compliance, audit trail for liability
- **Defense systems** - Verify authorized software, prevent backdoors

**Multi-Party Scenarios:**
- **Shared robot fleets** - Multiple organizations verify same robot used approved model
- **Third-party operations** - Customer verifies contractor used certified software
- **Supply chain** - Warehouse owner verifies robot operator didn't tamper with safety systems

**Regulatory Compliance:**
- **Aviation** - FAA-certified models with cryptographic proof of use
- **Healthcare** - FDA-approved diagnostic algorithms with audit trail
- **Insurance** - Prove robot used approved software at time of incident

**Adversarial Environments:**
- **Untrusted networks** - Robot in hostile environment can't fake safety checks
- **Public spaces** - Third parties can verify robot behavior without access
- **Critical infrastructure** - Prevent remote model substitution attacks

### Demo Scenario

In this demo:
- **Threat Model**: Attacker could modify `mobilenetv2-12.onnx` to always return "safe" classification
- **Without zkML**: No detection, motion always unlocked
- **With zkML**: `model_sha256` changes, proof verification fails, motion stays locked

The demo shows:
1. Camera detects object → MobileNetV2 inference (10ms)
2. JOLT proof generated (3-4s) binding to exact model+input hashes
3. Motion only unlocked if proof verifies
4. Event log creates audit trail: "At timestamp T, robot used model M on input I"

This enables **trustless robotics** where decisions are cryptographically verifiable without trusting the robot operator.

## Safety & Security

- **Fail-safe**: Lock engages if proof generation fails or times out
- **No blind trust**: Motion only allowed after cryptographic verification
- **Reproducible**: All inputs (model hash, tensor hash) logged in `/zkml/event`
- **Auditable**: MCAP recordings capture full provenance chain
- **Cryptographic binding**: SHA256 hashes prevent model/input substitution
- **128-bit security**: JOLT-Atlas uses Dory polynomial commitments

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
