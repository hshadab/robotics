#!/usr/bin/env python3
"""
Simple Windows Camera HTTP Server
No ROS2 dependencies needed on Windows!

This script:
1. Opens your webcam using OpenCV
2. Serves frames via HTTP on port 8080
3. WSL2 will fetch frames and publish to ROS2

Usage:
    python simple_windows_camera.py
"""

import cv2
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import time

class CameraStreamHandler(BaseHTTPRequestHandler):
    camera = None
    latest_frame = None
    frame_lock = threading.Lock()

    def do_GET(self):
        if self.path == '/frame.jpg':
            with self.frame_lock:
                if self.latest_frame is not None:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(self.latest_frame)
                else:
                    self.send_error(503, 'Camera not ready')
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Camera running')
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        # Suppress HTTP logs
        pass


def camera_thread(camera_id=0):
    """Continuously capture frames from camera"""
    print(f"Opening camera {camera_id}...")
    cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print(f"ERROR: Failed to open camera {camera_id}")
        print("Available cameras to try: 0, 1, 2")
        sys.exit(1)

    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print("="*50)
    print(f"✓ Camera {camera_id} opened successfully!")
    print(f"✓ Resolution: {width}x{height}")
    print(f"✓ Serving frames at http://localhost:8080/frame.jpg")
    print("="*50)
    print()
    print("Your webcam light should be ON now.")
    print("Leave this window open.")
    print("Press Ctrl+C to stop.")
    print()

    CameraStreamHandler.camera = cap
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Failed to read frame")
            time.sleep(0.1)
            continue

        # Encode as JPEG
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Update latest frame
        with CameraStreamHandler.frame_lock:
            CameraStreamHandler.latest_frame = jpeg.tobytes()

        frame_count += 1
        if frame_count % 300 == 0:
            print(f"Captured {frame_count} frames...")

        time.sleep(1/30.0)  # 30 FPS


def main():
    print()
    print("="*50)
    print("Simple Windows Camera Server")
    print("="*50)
    print()

    # Get camera ID from command line
    camera_id = 0
    if len(sys.argv) > 1:
        try:
            camera_id = int(sys.argv[1])
        except ValueError:
            print(f"ERROR: Invalid camera ID '{sys.argv[1]}'")
            print("Usage: python simple_windows_camera.py [camera_id]")
            sys.exit(1)

    # Start camera thread
    cam_thread = threading.Thread(target=camera_thread, args=(camera_id,), daemon=True)
    cam_thread.start()

    # Give camera time to initialize
    time.sleep(2)

    # Start HTTP server
    port = 8080
    server = HTTPServer(('0.0.0.0', port), CameraStreamHandler)

    print(f"HTTP server running on port {port}")
    print()
    print("Next step: In WSL2, run the ROS2 bridge node")
    print()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nStopping camera server...")
        if CameraStreamHandler.camera:
            CameraStreamHandler.camera.release()
        print("Camera stopped.")


if __name__ == '__main__':
    main()
