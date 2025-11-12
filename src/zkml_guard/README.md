# zkml_guard - ROS 2 Package

Proof-gated motion control system using JOLT Atlas zkML for robotics applications.

## Overview

This ROS 2 package provides a node that:
1. Subscribes to camera images (`/image`)
2. Runs ONNX model inference (MobileNetV2)
3. Generates cryptographic proofs using JOLT Atlas
4. Controls robot motion based on verified proofs
5. Publishes motion gate status to twist_mux

## Package Structure

```
zkml_guard/
├── config/               # Configuration files
│   ├── twist_mux.yaml   # Motion multiplexer config
│   └── zkml_guard.params.yaml  # Guard node parameters
├── launch/              # Launch files
│   ├── zkml_guard_demo.launch.py   # Basic demo
│   └── zkml_guard_proof.launch.py  # Full proof verification
├── zkml_guard/          # Python package
│   ├── zkml_guard_node.py  # Main ROS node (599 lines)
│   └── jolt.py             # JOLT proof utilities (223 lines)
├── requirements.txt     # Python dependencies
├── package.xml          # ROS package manifest
└── setup.py            # Python package setup
```

## Dependencies

### Python Packages
Install from requirements.txt:
```bash
pip install -r requirements.txt
```

Includes:
- onnxruntime>=1.17 (ONNX model inference)
- numpy>=1.23 (numerical operations)
- pillow>=10.0 (image processing)
- requests>=2.31 (HTTP client for verifier)
- opencv-python>=4.8.0 (camera capture)

### ROS 2 Packages
- rclpy (ROS 2 Python client)
- sensor_msgs (image messages)
- std_msgs (basic message types)
- twist_mux (motion multiplexer)

### External Services
- **onnx-verifier** service at http://localhost:9100
  - Generates JOLT proofs
  - See `tools/onnx-verifier/` for setup

## Configuration

### Quick Config: `config/zkml_guard.params.yaml`

```yaml
zkml_guard:
  ros__parameters:
    gating_mode: argmax              # Gate based on highest confidence class
    threshold: 0.40                  # 40% confidence minimum
    require_proof: true              # Require cryptographic proof
    unlock_hold_ms: 10000            # Keep unlocked for 10 seconds
    inference_period_ms: 2000        # Run inference every 2 seconds
    prove_on: rising_edge            # Generate proof on first detection
    reprove_on_label_change: false   # Don't cancel proof if label changes
    verifier_mode: http              # Use HTTP verifier service
    verifier_url: "http://localhost:9100/verify"
```

### Launch Arguments

```bash
ros2 launch zkml_guard zkml_guard_proof.launch.py \
  camera_topic:=/image \
  gating_mode:=argmax \
  threshold:=0.40 \
  require_proof:=true \
  unlock_hold_ms:=10000 \
  proof_timeout_ms:=15000
```

## Usage

### 1. Build the package
```bash
cd ~/robotics
colcon build --packages-select zkml_guard
source install/setup.bash
```

### 2. Start onnx-verifier service
```bash
cd tools/onnx-verifier
npm install
npm start  # Runs on port 9100
```

### 3. Launch the guard node
```bash
ros2 launch zkml_guard zkml_guard_proof.launch.py
```

### 4. Publish camera images
The node expects images on `/image` topic (sensor_msgs/Image).

Use provided camera scripts:
```bash
python3 linux_camera_publisher.py
```

## Topics

### Subscribed
- `/image` (sensor_msgs/Image) - Camera frames for inference

### Published
- `/zkml/event` (std_msgs/String) - JSON events with inference results
- `/zkml/stop` (std_msgs/Bool) - Motion gate status (true=locked, false=unlocked)

## How It Works

1. **Inference**: Runs MobileNetV2 on camera frames every 2 seconds
2. **Gate Logic**: Checks if detection meets threshold (e.g., 40% confidence)
3. **Proof Generation**: On successful detection, sends request to onnx-verifier
4. **Verification**: Waits for JOLT proof (typically 3-4 seconds)
5. **Motion Unlock**: On verified proof, unlocks motion for 10 seconds
6. **Event Publishing**: Broadcasts status to UI and other nodes

## Development

### Running Tests
```bash
# No unit tests yet - manual testing via demo
ros2 launch zkml_guard zkml_guard_proof.launch.py
```

### Debugging
- Check node logs: `~/.ros/log/`
- Monitor topics: `ros2 topic echo /zkml/event`
- Verify proof requests: Check onnx-verifier logs

### Key Files
- `zkml_guard_node.py:344-353` - Model tampering detection
- `zkml_guard_node.py:482-586` - HTTP proof verification
- `jolt.py:103-177` - Local proof generation (CLI mode)

## Security Features

- **Model Integrity**: SHA-256 verification before each inference
- **Proof Binding**: Cryptographic commitment links input→output→proof
- **Tamper Detection**: Automatic verification failure on model modification

See `SECURITY_FIXES.md` for security improvements made to the codebase.

## Troubleshooting

### Model fails to download
- Check internet connection
- Fallback model: `tools/onnx-verifier/models/mobilenetv2-12.onnx`
- Set `model_path` in YAML config to use local model

### Proof generation fails
- Verify onnx-verifier is running: `curl http://localhost:9100/health`
- Check JOLT binary exists: `tools/onnx-verifier/bin/simple_jolt_proof`
- Review proof timeout setting (default: 15 seconds)

### Motion not unlocking
- Check `/zkml/stop` topic: `ros2 topic echo /zkml/stop`
- Verify twist_mux configuration
- Ensure proof verification succeeded (check logs)

## License

MIT
