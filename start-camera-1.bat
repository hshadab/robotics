@echo off
REM Quick launcher for Windows camera ID 1
REM Double-click this file to start the camera

echo ========================================
echo Windows Camera Server (ID 1)
echo ========================================
echo.

REM Kill any existing camera processes
echo Stopping existing camera processes...
taskkill /F /FI "WINDOWTITLE eq *simple_windows_camera*" >nul 2>&1
timeout /t 1 /nobreak >nul

REM Navigate to robotics directory
cd /d %~dp0

REM Start camera
echo Starting camera ID 1...
echo Your webcam light should turn ON now.
echo Keep this window open.
echo Press Ctrl+C to stop.
echo.

python simple_windows_camera.py 1

pause
