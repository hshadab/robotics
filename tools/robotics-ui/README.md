# Robotics UI - zkML Demo Interface

Web-based demonstration interface for the JOLT Atlas zkML robotics system.

## Overview

Modern web UI that provides:
- Real-time camera feed display
- Inference results visualization
- Proof verification status
- Motion gating countdown
- Verified proofs history
- Service lifecycle control (start/stop)

## Quick Start

```bash
cd tools/robotics-ui
npm install
npm start  # Runs on http://localhost:9200
```

Then open http://localhost:9200 in your browser.

## Architecture

### Components

1. **server.js** (Node.js backend)
   - Express HTTP server
   - ROS 2 process management
   - File-based IPC with ROS nodes
   - REST API endpoints
   - Rate limiting (100 req/min)

2. **public/demo.html** (Frontend)
   - Single-page web interface
   - Real-time polling (1 Hz for events, 10 Hz for camera)
   - Service control buttons
   - Proof verification display
   - Motion gating countdown

3. **event_proxy.py** (ROS bridge)
   - Subscribes to `/zkml/event` topic
   - Writes events to `/tmp/rdemo-last-event.json`
   - Manages verified proofs history
   - Serves as ROS → HTTP bridge

4. **frame_proxy.py** (Camera bridge)
   - Subscribes to `/image` topic
   - Writes frames to `/tmp/rdemo-frame.png`
   - Throttles updates (every 5th frame)

## API Endpoints

### Service Control
- `POST /start/all` - Start all services (verifier, camera, guard, teleop)
- `POST /stop/all` - Stop all services
- `POST /start/verifier` - Start onnx-verifier service
- `POST /start/camera` - Start camera publisher
- `POST /start/guard` - Start zkml_guard node
- `POST /start/teleop` - Start teleop keyboard control

### Data Endpoints
- `GET /api/last_event` - Latest inference event (JSON)
- `GET /api/verified_proofs` - Proof verification history (JSON array)
- `GET /api/camera` - Current camera frame (PNG image)
- `GET /api/stop_status` - Motion gate status

### Admin Endpoints
- `POST /api/flip_model` - Tamper with model (demo feature)
- `POST /api/reset_model` - Restore original model
- `GET /api/list_models` - List available models
- `POST /stop/process` - Stop specific process by name

## Configuration

### Environment Variables

Create `.env` file (see `.env.example`):
```bash
PORT=9200           # Server port (default: 9200)
ROS_DISTRO=jazzy    # ROS 2 distribution (default: jazzy)
```

### Rate Limiting

Server includes basic rate limiting:
- 100 requests per minute per IP
- Protects against accidental DOS
- Configurable in server.js:11-18

## Data Flow

```
Camera → /image topic → frame_proxy.py → /tmp/rdemo-frame.png → HTTP /api/camera → Browser

Guard → /zkml/event topic → event_proxy.py → /tmp/rdemo-last-event.json → HTTP /api/last_event → Browser
                                          → /tmp/rdemo-verified-proofs.json → HTTP /api/verified_proofs → Browser
```

## File-Based IPC

Temporary files used for inter-process communication:

| File | Purpose | Updated By |
|------|---------|------------|
| `/tmp/rdemo-last-event.json` | Latest inference event | event_proxy.py |
| `/tmp/rdemo-verified-proofs.json` | Proof history | event_proxy.py |
| `/tmp/rdemo-frame.png` | Current camera frame | frame_proxy.py |
| `/tmp/rdemo-stop.txt` | Motion gate status | zkml_guard node |
| `/tmp/rdemo-*.pid` | Process IDs | server.js |
| `/tmp/*.log` | Service logs | server.js |

## UI Features

### Pipeline Visualization
Shows 3-step verification process:
1. **Inference** - ONNX model runs on image
2. **Proof** - JOLT generates cryptographic proof
3. **Verification** - Proof validated, countdown starts

### Motion Gating Display
- Shows lock status (LOCKED / UNLOCKED)
- Countdown timer when motion allowed (10s → 9s → ...)
- Visual feedback with color coding

### Verified Proofs List
- Displays recent successful verifications
- Shows snapshot images
- Includes proof metadata (time, label, confidence)
- Expandable proof details

### Service Status
- Real-time status indicators for each service
- Start/Stop controls
- Log viewing (planned feature)

## Development

### Running in Development
```bash
# Start server with auto-reload
npm install -g nodemon
nodemon server.js
```

### Testing API Endpoints
```bash
# Test last event endpoint
curl http://localhost:9200/api/last_event

# Test camera endpoint
curl http://localhost:9200/api/camera -o frame.png

# Start all services
curl -X POST http://localhost:9200/start/all
```

### Debugging
- Server logs to console
- Check `/tmp/*.log` for service logs
- Use browser DevTools Network tab for API calls
- Monitor `/tmp/rdemo-*.json` files directly

## Known Limitations (Prototype)

1. **Polling instead of WebSocket** - 1 second delay for updates
2. **File-based IPC** - Not suitable for production, but simple for demo
3. **No authentication** - Localhost only, no access control
4. **No persistent storage** - Events lost on restart
5. **Single client** - No multi-user support

These are acceptable for a localhost demo but should be addressed for production deployment.

## Recent Fixes (See CLAUDE.md)

- Fixed verification display (Step 3 now updates correctly)
- Fixed motion countdown (shows 10s → 0s properly)
- Added snapshot pictures to verified proofs
- Auto-stop services on page load for consistent state
- Removed distracting FPS counter

## Future Enhancements

- WebSocket for real-time updates (no polling)
- Persistent database (SQLite) for proof history
- Multi-client support
- Authentication/authorization
- Service log streaming to UI
- Configurable update intervals
- Proof export/download feature

## Troubleshooting

### Services won't start
- Check ROS 2 environment: `echo $ROS_DISTRO`
- Verify workspace built: `ls ~/robotics/install/`
- Check onnx-verifier: `cd ../onnx-verifier && npm start`

### Camera not updating
- Verify camera script running: `ps aux | grep camera`
- Check frame file: `ls -lh /tmp/rdemo-frame.png`
- Test camera endpoint: `curl -I http://localhost:9200/api/camera`

### Events not appearing
- Check event proxy: `ps aux | grep event_proxy`
- Verify ROS topic: `ros2 topic echo /zkml/event`
- Check event file: `cat /tmp/rdemo-last-event.json`

### "Too many requests" error
Rate limiting triggered. Wait 1 minute or restart server.

## Dependencies

- Node.js >= 14
- express 4.19.2
- cors 2.8.5
- express-rate-limit 7.1.5
- ROS 2 (jazzy recommended)
- Python 3.7+ (for proxy scripts)

## License

MIT
