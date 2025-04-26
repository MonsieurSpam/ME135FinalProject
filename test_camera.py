#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple script to test camera access on macOS
"""

import cv2
import os

# Set environment variable to enable camera authorization
os.environ['OPENCV_AVFOUNDATION_SKIP_AUTH'] = '0'

def test_camera():
    print("Testing camera access...")
    
    # List available cameras with debug info
    for i in range(10):  # Check first 10 potential camera indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera index {i} is available")
            ret, frame = cap.read()
            if ret:
                print(f"  - Resolution: {frame.shape[1]}x{frame.shape[0]}")
            else:
                print(f"  - Cannot read frame from camera {i}")
            cap.release()
        else:
            print(f"Camera index {i} is not available")
    
    # Try to open the default camera
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    print("Camera opened successfully!")
    print("Press 'q' to quit")
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to grab frame")
            break
            
        # Display the frame
        cv2.imshow('Camera Test', frame)
        
        # Exit on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the camera
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera() 