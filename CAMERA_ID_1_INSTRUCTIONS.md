# Windows Camera Setup - Camera ID 1

## Quick Fix for Black Frames

Your Windows `http_camera_bridge` is currently publishing black frames because it's likely using the wrong camera ID (default is 0, but you need camera 1).

## Solution: Use Camera ID 1

### Option 1: Batch File (Easiest)

**On Windows**, navigate to your robotics directory and **double-click**:
```
start-camera-1.bat
```

This will:
1. Stop any existing camera processes
2. Start camera server with camera ID **1**
3. Your webcam light should turn ON
4. Frames will be served at `http://localhost:8080/frame.jpg`

### Option 2: PowerShell Script

**In Windows PowerShell**, run:
```powershell
cd \\wsl$\Ubuntu\home\hshadab\robotics
.\restart-windows-camera.ps1
```

### Option 3: Manual Command

**In Windows Command Prompt or PowerShell**:
```cmd
cd path\to\robotics
python simple_windows_camera.py 1
```

## Verify It's Working

### On Windows
You should see:
```
==================================================
✓ Camera 1 opened successfully!
✓ Resolution: 640x480
✓ Serving frames at http://localhost:8080/frame.jpg
==================================================

Your webcam light should be ON now.
```

### In WSL2 (Linux)
Check that frames are coming through:
```bash
curl -s http://localhost:8080/frame.jpg -o /tmp/test_frame.jpg
file /tmp/test_frame.jpg  # Should show: JPEG image data
```

## Then Test the Demo

1. **Stop current services** (if running):
   ```bash
   curl http://localhost:9200/stop/all
   ```

2. **Restart with real camera**:
   ```bash
   curl "http://localhost:9200/start/full?mode=http&burger=0&record=0"
   ```

3. **Open the UI**:
   ```
   http://localhost:9200/demo.html
   ```

4. **You should now see**:
   - ✅ Real camera feed (not black!)
   - ✅ ML detection overlays
   - ✅ Live events showing predictions
   - ✅ Motion gating based on detections

## Troubleshooting

### Camera Still Shows Black
- Make sure no other Windows applications are using the camera (Teams, Zoom, etc.)
- Try different camera IDs: `python simple_windows_camera.py 0`, `1`, or `2`
- Check Windows Settings → Privacy → Camera permissions

### Can't Find Camera Script
The script is located at:
```
\\wsl$\Ubuntu\home\hshadab\robotics\simple_windows_camera.py
```

You can also access it via File Explorer: `\\wsl$\Ubuntu\home\hshadab\robotics\`

### Camera Opens But Still Black in Demo
1. Check HTTP server is working:
   ```powershell
   curl http://localhost:8080/status
   ```
   Should return: `Camera running`

2. Check frame endpoint:
   ```powershell
   curl http://localhost:8080/frame.jpg -OutFile test.jpg
   ```
   Open `test.jpg` - should show your camera view

3. Restart the WSL2 bridge:
   ```bash
   # In WSL2
   ./stop_demo.sh
   ./start_demo.sh --no-browser
   ```

## Camera ID Reference

| Camera ID | Typical Device |
|-----------|----------------|
| 0 | Built-in laptop webcam (primary) |
| 1 | Built-in laptop IR camera / Secondary camera |
| 2 | External USB webcam |

**Your system needs Camera ID 1** (likely an IR camera or secondary camera device)
