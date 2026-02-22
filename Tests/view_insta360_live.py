#!/usr/bin/env python3
"""
Simple real-time viewer for Insta360 X3 camera
Shows the dual fisheye view (front and back lenses side-by-side)

Controls:
    Q or ESC - Quit
    S - Save snapshot
    F - Toggle fullscreen
    1 - Show full dual view
    2 - Show left lens only (front)
    3 - Show right lens only (back)
"""

import cv2
import sys
import time
from datetime import datetime

# Configuration
CAMERA_DEVICE = "/dev/video0"  # Change if needed
WINDOW_NAME = "Insta360 X3 Live View"
SAVE_DIR = "./insta360_captures/"

def main():
    # Open camera
    print(f"Opening camera: {CAMERA_DEVICE}")
    cap = cv2.VideoCapture(CAMERA_DEVICE)

    if not cap.isOpened():
        print(f"Error: Could not open camera at {CAMERA_DEVICE}")
        print("Try:")
        print("  - Check if camera is connected: lsusb | grep Insta")
        print("  - Try different device: CAMERA_DEVICE=/dev/video1 python3 view_insta360_live.py")
        return 1

    # Set resolution (Insta360 X3 supports 1920x1080 @ 30fps)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Get actual resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    print(f"Camera opened: {width}x{height} @ {fps}fps")
    print(f"\nControls:")
    print(f"  Q or ESC - Quit")
    print(f"  S - Save snapshot")
    print(f"  F - Toggle fullscreen")
    print(f"  1 - Show full dual view (both lenses)")
    print(f"  2 - Show left lens only (front)")
    print(f"  3 - Show right lens only (back)")
    print(f"\nPress any key in the video window to start...")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    view_mode = 1  # 1=both, 2=left only, 3=right only
    fullscreen = False
    frame_count = 0
    start_time = time.time()

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to read frame")
            break

        frame_count += 1

        # Calculate FPS
        elapsed = time.time() - start_time
        if elapsed > 1.0:
            current_fps = frame_count / elapsed
            frame_count = 0
            start_time = time.time()
        else:
            current_fps = fps

        # Split into left and right fisheye views
        h, w = frame.shape[:2]
        left_lens = frame[:, :w//2]   # Front lens
        right_lens = frame[:, w//2:]  # Back lens

        # Choose display based on view mode
        if view_mode == 1:
            # Show both lenses side-by-side
            display_frame = frame.copy()
            cv2.putText(display_frame, "FRONT", (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_frame, "BACK", (w//2 + 50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.line(display_frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)

        elif view_mode == 2:
            # Show left lens only (front)
            display_frame = cv2.resize(left_lens, (width, height))
            cv2.putText(display_frame, "FRONT LENS", (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        elif view_mode == 3:
            # Show right lens only (back)
            display_frame = cv2.resize(right_lens, (width, height))
            cv2.putText(display_frame, "BACK LENS", (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Add FPS counter
        cv2.putText(display_frame, f"FPS: {current_fps:.1f}",
                   (display_frame.shape[1] - 150, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display frame
        cv2.imshow(WINDOW_NAME, display_frame)

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # Q or ESC
            print("Quitting...")
            break

        elif key == ord('s'):  # Save snapshot
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{SAVE_DIR}snapshot_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Snapshot saved: {filename}")

        elif key == ord('f'):  # Toggle fullscreen
            fullscreen = not fullscreen
            if fullscreen:
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                     cv2.WINDOW_FULLSCREEN)
            else:
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                     cv2.WINDOW_NORMAL)
            print(f"Fullscreen: {'ON' if fullscreen else 'OFF'}")

        elif key == ord('1'):
            view_mode = 1
            print("View mode: Both lenses (dual fisheye)")

        elif key == ord('2'):
            view_mode = 2
            print("View mode: Front lens only")

        elif key == ord('3'):
            view_mode = 3
            print("View mode: Back lens only")

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("Camera closed")
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(0)
