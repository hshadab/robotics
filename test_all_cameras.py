#!/usr/bin/env python3
"""
Test all camera IDs to find which one is available
Run this in PowerShell to find your USB camera ID
"""

import cv2

print()
print("="*60)
print("Testing all camera IDs (0-5)...")
print("="*60)
print()

for camera_id in range(6):
    print(f"Testing camera {camera_id}... ", end='', flush=True)

    try:
        cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)

        if cap.isOpened():
            # Try to read a frame
            ret, frame = cap.read()

            if ret:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"✓ FOUND! ({width}x{height})")

                # Get camera name if possible
                backend = cap.getBackendName()
                print(f"   Backend: {backend}")
            else:
                print("✗ opened but can't read frames")

            cap.release()
        else:
            print("✗ not available")

    except Exception as e:
        print(f"✗ error: {e}")

print()
print("="*60)
print("Summary:")
print("  - Your LG laptop camera is ID 1 (already working)")
print("  - Look for another ✓ FOUND above - that's your USB camera!")
print("  - Use that ID with: python simple_windows_camera.py <ID>")
print("="*60)
print()
