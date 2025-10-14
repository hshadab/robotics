# Windows Camera Setup Guide

## Automated Installation Script for Windows

This guide will help you install ROS2 on Windows to use your laptop camera with the robotics demo.

## Prerequisites

- Windows 10/11 (64-bit)
- Administrator access
- ~2GB disk space
- Working webcam

## Option 1: Quick Install (Recommended)

### Step 1: Download ROS2 for Windows

Open **PowerShell as Administrator** and run:

```powershell
# Create installation directory
New-Item -Path "C:\ros2" -ItemType Directory -Force
cd C:\ros2

# Download ROS2 Jazzy (latest)
$url = "https://github.com/ros2/ros2/releases/download/release-jazzy-20241010/ros2-jazzy-20241010-windows-release-amd64.zip"
Write-Host "Downloading ROS2 Jazzy for Windows..."
Invoke-WebRequest -Uri $url -OutFile "ros2-jazzy.zip"

# Extract
Write-Host "Extracting..."
Expand-Archive -Path "ros2-jazzy.zip" -DestinationPath "C:\ros2" -Force

Write-Host "✓ ROS2 downloaded and extracted to C:\ros2"
```

### Step 2: Install Dependencies

```powershell
# Install Visual C++ Redistributables (required)
$vcUrl = "https://aka.ms/vs/17/release/vc_redist.x64.exe"
Write-Host "Downloading Visual C++ Redistributables..."
Invoke-WebRequest -Uri $vcUrl -OutFile "vc_redist.x64.exe"
Start-Process -FilePath ".\vc_redist.x64.exe" -ArgumentList "/quiet /norestart" -Wait

Write-Host "✓ Dependencies installed"
```

### Step 3: Create Camera Launcher Script

Create file: **`C:\ros2\launch_camera.bat`**

```batch
@echo off
echo ========================================
echo ROS2 Camera for zkML Robotics Demo
echo ========================================
echo.

REM Source ROS2 environment
call C:\ros2\ros2-windows\local_setup.bat

REM Set ROS_DOMAIN_ID to match WSL2 (optional, usually auto-detects)
set ROS_DOMAIN_ID=0

REM Launch camera
echo Starting camera node...
echo Your webcam should turn on now.
echo Press Ctrl+C to stop.
echo.

ros2 run image_tools cam2image

pause
```

### Step 4: Run the Camera

1. **Double-click** `C:\ros2\launch_camera.bat`
2. Your **webcam light** should turn on
3. Leave this window **open**

### Step 5: In WSL2 (Your Current Setup)

The camera feed will automatically appear on ROS2 network!

```bash
# Verify you're receiving camera feed
source ~/robotics/install/setup.bash
ros2 topic list | grep image

# Should show: /image
```

### Step 6: Use in Demo

1. Open UI: http://localhost:9200
2. **Uncheck "Burger Mode"**
3. Click "Stop All" then "Start Full Demo"
4. You should now see **real camera feed** with ML detection!

---

## Option 2: Even Faster (Chocolatey)

If you have Chocolatey package manager:

```powershell
# Install Chocolatey first (if not installed)
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

# Install ROS2
choco install ros-jazzy-desktop -y

# Run camera
ros2 run image_tools cam2image
```

---

## Option 3: Manual Download (If Scripts Fail)

1. **Download**: https://github.com/ros2/ros2/releases/latest
   - Look for: `ros2-jazzy-*-windows-release-amd64.zip`

2. **Extract** to `C:\ros2`

3. **Install Visual C++ Redistributables**: https://aka.ms/vs/17/release/vc_redist.x64.exe

4. **Open CMD/PowerShell**:
   ```cmd
   cd C:\ros2\ros2-windows
   call local_setup.bat
   ros2 run image_tools cam2image
   ```

---

## Troubleshooting

### Camera Not Found

```powershell
# Check camera is working in Windows
Get-PnpDevice | Where-Object {$_.Class -eq "Camera"}
```

If no camera listed:
- Check Device Manager
- Enable camera in Windows Settings → Privacy → Camera
- Restart computer

### WSL2 Not Receiving Images

```bash
# In WSL2, check firewall isn't blocking
ros2 daemon stop
ros2 daemon start

# Set explicit domain
export ROS_DOMAIN_ID=0

# Check topic
ros2 topic hz /image
```

### Performance Issues

If camera is laggy:
- Close other applications using camera (Teams, Zoom, etc.)
- Reduce resolution: `ros2 run image_tools cam2image --ros-args -p width:=320 -p height:=240`

---

## Network Configuration (Advanced)

If Windows and WSL2 aren't auto-discovering:

### Windows Side:
```powershell
# Check IP
ipconfig

# Allow ROS2 through firewall
New-NetFirewallRule -DisplayName "ROS2 DDS" -Direction Inbound -Protocol UDP -LocalPort 7400-7500 -Action Allow
```

### WSL2 Side:
```bash
# Get WSL2 IP
hostname -I

# Verify ROS2 daemon
ros2 daemon status
```

---

## Quick Test

### Windows:
```cmd
cd C:\ros2\ros2-windows
call local_setup.bat
ros2 run image_tools cam2image
```

### WSL2 (separate terminal):
```bash
source ~/robotics/install/setup.bash
ros2 topic echo /image --once
```

You should see image data!

---

## Alternative: Use Webcam via HTTP Stream

If ROS2 installation is too complex, you can stream camera via HTTP:

### Windows:
1. Install OBS Studio: https://obsproject.com/
2. Add "Video Capture Device" (your camera)
3. Start Virtual Camera
4. Use browser extension to stream to WSL2

This is less ideal but works without ROS2 installation.

---

## Summary

**Easiest Path**:
1. Open PowerShell as Admin
2. Paste the quick install script above
3. Run `C:\ros2\launch_camera.bat`
4. Uncheck "Burger Mode" in UI
5. Restart demo

**Result**: Real laptop camera → ROS2 → ML Detection → ZK Proofs!

---

## Need Help?

If installation fails:
1. Check error messages
2. Ensure you have admin rights
3. Verify Windows version (needs Windows 10 1809+)
4. Check antivirus isn't blocking downloads

Contact me with specific error messages if you get stuck!
