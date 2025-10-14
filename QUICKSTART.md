# Quick Start Guide

## Web UI (Recommended)

```bash
# 1. Build ROS workspace (first time only)
cd ~/robotics
colcon build
source install/setup.bash

# 2. Install dependencies (first time only)
pip install -r src/zkml_guard/requirements.txt
cd tools/onnx-verifier && npm install && cd ../..
cd tools/robotics-ui && npm install && cd ../..

# 3. Start UI server (auto-starts verifier and proxies)
cd ~/robotics/tools/robotics-ui
npm start

# 4. Open http://localhost:9200/demo.html
# Click "Start Full Demo"
```

That's it! The UI handles everything else.

## Manual Mode (3 terminals)

```bash
# Terminal 1: Camera
source ~/robotics/install/setup.bash
ros2 run image_tools cam2image

# Terminal 2: Guard
source ~/robotics/install/setup.bash
ros2 launch zkml_guard zkml_guard_proof.launch.py

# Terminal 3: Teleop
source ~/robotics/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Troubleshooting

**UI not updating?**
```bash
curl http://localhost:9200/api/proxy_status
# Should show both proxies running

# If not, restart them:
curl http://localhost:9200/start/proxies
```

**Check ROS topics:**
```bash
ros2 topic list
ros2 topic echo /zkml/event --once
ros2 topic hz /image
```

**View logs:**
```bash
tail -f /tmp/event_proxy.py.log
tail -f /tmp/frame_proxy.py.log
```

## Modes

- **HTTP mode** (default): Uses HTTP verifier on port 9100
- **CLI mode**: Uses local `atlas_argmax_prover` binary

Switch modes:
```bash
# In UI: change dropdown to "cli" before starting
# Or manually:
ros2 launch zkml_guard zkml_guard_proof.launch.py --ros-args -p verifier_mode:=cli
```

## Key Endpoints

- http://localhost:9200/demo.html - Enhanced demo UI
- http://localhost:9200/status - Component status
- http://localhost:9200/api/proxy_status - Proxy health check
- http://localhost:9200/api/last_event - Latest zkML event JSON
- http://localhost:9200/api/frame.png - Live camera frame
- http://localhost:9200/debug/logs - View proxy logs
- http://localhost:9100/verify - HTTP verifier endpoint
- http://localhost:9100/health - Verifier health check
