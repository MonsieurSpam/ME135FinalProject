#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Hand Tracking 3D Control for macOS
----------------------------------
This script uses OpenCV and Mediapipe to track hand movements and control a 3D point.
- Right hand pinch: Move point in X-Y plane
- Left hand pinch: Move point in Z plane (up/down)

This version is optimized for macOS compatibility.
"""

import cv2
import mediapipe as mp
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
import os

# Set environment variable to enable camera authorization on Mac
os.environ['OPENCV_AVFOUNDATION_SKIP_AUTH'] = '0'

# Mediapipe Hand settings
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Global variables
point_position = np.array([0.0, 0.0, 0.0])  # Initial 3D point position
is_calibrated = False
left_pinch_range = [0.3, 0.7]  # Default [min, max] values for Z control
right_pinch_range = [0.3, 0.7, 0.3, 0.7]  # Default [min_x, max_x, min_y, max_y] for XY control
update_visualization = False


def is_pinch(hand_landmarks):
    """Detect if the hand is making a pinch gesture (thumb and index finger touching)."""
    if not hand_landmarks:
        return False
    
    # Get the landmarks for thumb tip and index finger tip
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    
    # Calculate distance between thumb and index finger tips
    distance = math.sqrt(
        (thumb_tip.x - index_tip.x) ** 2 +
        (thumb_tip.y - index_tip.y) ** 2 +
        (thumb_tip.z - index_tip.z) ** 2
    )
    
    # Determine if distance is small enough to be considered a pinch
    return distance < 0.08  # Slightly higher threshold for better detection


def get_pinch_position(hand_landmarks):
    """Get the position of the pinch (average of thumb and index finger tips)."""
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    
    # Calculate the midpoint between thumb and index finger
    x = (thumb_tip.x + index_tip.x) / 2
    y = (thumb_tip.y + index_tip.y) / 2
    z = (thumb_tip.z + index_tip.z) / 2
    
    return (x, y, z)


def identify_hands(results):
    """Identify which hand is left and which is right."""
    if not results.multi_hand_landmarks:
        return None, None
    
    left_hand = None
    right_hand = None
    
    for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
        if idx < len(results.multi_handedness):
            handedness = results.multi_handedness[idx]
            hand_label = handedness.classification[0].label
            
            if hand_label == 'Left':
                right_hand = hand_landmarks  # Camera mirrors image
            elif hand_label == 'Right':
                left_hand = hand_landmarks   # Camera mirrors image
    
    return left_hand, right_hand


def run_calibration(frame, hands, calibration_stage, calibration_timer, calibration_positions):
    """Run the calibration process."""
    global is_calibrated, left_pinch_range, right_pinch_range
    
    # Process the frame to find hands
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)
    
    # Draw hand landmarks
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(
                frame, 
                hand_landmarks, 
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style()
            )
    
    left_hand, right_hand = identify_hands(results)
    frame_height, frame_width, _ = frame.shape
    
    # Display calibration instructions
    if calibration_stage == 0:
        # Starting calibration - instructions
        cv2.putText(frame, "Calibration: We will calibrate hand movement ranges", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, "Have both hands visible in the frame", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, "Press 'C' to continue", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Wait for 'C' key to continue
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            calibration_stage = 1
            calibration_timer = time.time()
            calibration_positions = []
    
    elif calibration_stage == 1:
        # XY Calibration with right hand
        cv2.putText(frame, "Right Hand Calibration (X-Y plane)", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, "Pinch your right hand and move it around the screen", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Time left: {max(0, int(5 - (time.time() - calibration_timer)))} seconds", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if right_hand and is_pinch(right_hand):
            x, y, _ = get_pinch_position(right_hand)
            
            # Record the pinch positions for calibration
            calibration_positions.append((x, y))
            
            # Draw the pinch point
            cx, cy = int(x * frame_width), int(y * frame_height)
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
        
        # Move to next stage after 5 seconds
        if time.time() - calibration_timer > 5:
            # Calculate the right hand pinch range
            if calibration_positions:
                x_values = [p[0] for p in calibration_positions]
                y_values = [p[1] for p in calibration_positions]
                
                right_pinch_range = [
                    min(x_values), max(x_values),
                    min(y_values), max(y_values)
                ]
            
            calibration_stage = 2
            calibration_timer = time.time()
            calibration_positions = []
    
    elif calibration_stage == 2:
        # Z Calibration with left hand
        cv2.putText(frame, "Left Hand Calibration (Z axis)", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(frame, "Pinch your left hand and move it up and down", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(frame, f"Time left: {max(0, int(5 - (time.time() - calibration_timer)))} seconds", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        if left_hand and is_pinch(left_hand):
            _, y, _ = get_pinch_position(left_hand)
            
            # Record the pinch Y positions for Z calibration
            calibration_positions.append(y)
            
            # Draw the pinch point
            cx, cy = int(0.2 * frame_width), int(y * frame_height)
            cv2.circle(frame, (cx, cy), 10, (255, 0, 0), -1)
        
        # Move to next stage after 5 seconds
        if time.time() - calibration_timer > 5:
            # Calculate the left hand pinch range for Z axis
            if calibration_positions:
                left_pinch_range = [min(calibration_positions), max(calibration_positions)]
            
            calibration_stage = 3
            calibration_timer = time.time()
    
    elif calibration_stage == 3:
        # Calibration complete
        cv2.putText(frame, "Calibration Complete!", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, "Right hand pinch: control X-Y position", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, "Left hand pinch: control Z position", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        if time.time() - calibration_timer > 2:
            is_calibrated = True
    
    return frame, calibration_stage, calibration_timer, calibration_positions, results


def process_hands(frame, results):
    """Process the detected hands and update the 3D point position."""
    global point_position, left_pinch_range, right_pinch_range, update_visualization
    
    frame_height, frame_width, _ = frame.shape
    
    # Process the hands to update the 3D point
    left_hand, right_hand = identify_hands(results)
    
    # Update X-Y position with right hand
    if right_hand and is_pinch(right_hand):
        x, y, _ = get_pinch_position(right_hand)
        
        # Normalize based on calibrated range
        min_x, max_x, min_y, max_y = right_pinch_range
        if max_x > min_x and max_y > min_y:
            norm_x = (x - min_x) / (max_x - min_x)
            norm_y = (y - min_y) / (max_y - min_y)
            
            # Clamp values to [0, 1]
            norm_x = max(0, min(1, norm_x))
            norm_y = max(0, min(1, norm_y))
            
            # Scale to [-1, 1] range and invert Y axis (video has Y increasing downward)
            scaled_x = norm_x * 2 - 1
            scaled_y = 1 - (norm_y * 2)
            
            # Update point position for X-Y
            point_position[0] = scaled_x
            point_position[1] = scaled_y
            update_visualization = True
        
        # Draw visual feedback
        cx, cy = int(x * frame_width), int(y * frame_height)
        cv2.circle(frame, (cx, cy), 15, (0, 255, 0), -1)
        cv2.putText(frame, f"X: {point_position[0]:.2f}, Y: {point_position[1]:.2f}", 
                    (cx + 20, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Update Z position with left hand
    if left_hand and is_pinch(left_hand):
        _, y, _ = get_pinch_position(left_hand)
        
        # Normalize based on calibrated range
        min_y, max_y = left_pinch_range
        if max_y > min_y:
            norm_y = (y - min_y) / (max_y - min_y)
            
            # Clamp values to [0, 1]
            norm_y = max(0, min(1, norm_y))
            
            # Scale to [-1, 1] range and invert (video has Y increasing downward)
            scaled_z = 1 - (norm_y * 2)
            
            # Update point position for Z
            point_position[2] = scaled_z
            update_visualization = True
        
        # Draw visual feedback
        cx, cy = int(0.2 * frame_width), int(y * frame_height)
        cv2.circle(frame, (cx, cy), 15, (255, 0, 0), -1)
        cv2.putText(frame, f"Z: {point_position[2]:.2f}", 
                    (cx + 20, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    # Draw the current 3D coordinates
    cv2.putText(frame, f"Point: ({point_position[0]:.2f}, {point_position[1]:.2f}, {point_position[2]:.2f})", 
                (20, frame_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    return frame


def setup_3d_plot():
    """Set up the 3D plot for visualization."""
    # Create a figure and a 3D axis
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set the axis limits
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_zlim(-1.5, 1.5)
    
    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'3D Point Control')
    
    # Create a scatter plot for the 3D point
    scatter = ax.scatter([point_position[0]], [point_position[1]], [point_position[2]], 
                         c='red', s=100, marker='o')
    
    # Add reference grid and origin
    ax.plot([-1, 1], [0, 0], [0, 0], 'k--', alpha=0.5)  # X axis
    ax.plot([0, 0], [-1, 1], [0, 0], 'k--', alpha=0.5)  # Y axis
    ax.plot([0, 0], [0, 0], [-1, 1], 'k--', alpha=0.5)  # Z axis
    
    # Add reference planes with light color
    xx, yy = np.meshgrid(np.linspace(-1, 1, 2), np.linspace(-1, 1, 2))
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.1, color='gray')  # XY plane
    ax.plot_surface(xx, np.zeros_like(xx), yy, alpha=0.1, color='gray')  # XZ plane
    ax.plot_surface(np.zeros_like(xx), xx, yy, alpha=0.1, color='gray')  # YZ plane
    
    plt.ion()  # Turn on interactive mode
    plt.draw()
    plt.pause(0.001)
    
    return fig, ax, scatter


def update_3d_plot(fig, ax, scatter):
    """Update the 3D plot with the current point position."""
    global update_visualization
    
    if update_visualization:
        # Update the 3D point position
        scatter._offsets3d = ([point_position[0]], [point_position[1]], [point_position[2]])
        
        # Update the title with current coordinates
        ax.set_title(f'3D Point: ({point_position[0]:.2f}, {point_position[1]:.2f}, {point_position[2]:.2f})')
        
        # Redraw the plot
        fig.canvas.draw_idle()
        plt.pause(0.001)
        
        update_visualization = False


def main():
    """Main function to run the hand tracking and 3D visualization."""
    global is_calibrated, update_visualization
    
    print("Starting Hand Tracking 3D Control")
    print("Press 'q' or ESC to exit")
    
    # Initialize webcam with retry logic
    max_retries = 3
    for attempt in range(max_retries):
        print(f"Attempting to connect to camera (attempt {attempt+1}/{max_retries})...")
        cap = cv2.VideoCapture(1)  # Use camera index 1
        
        if not cap.isOpened():
            print(f"Failed to open camera on attempt {attempt+1}")
            if attempt < max_retries - 1:
                print("Retrying in 2 seconds...")
                time.sleep(2)
            continue
        
        # Camera warm-up time
        print("Camera connected! Warming up...")
        for i in range(10):  # Discard the first few frames
            ret, _ = cap.read()
            time.sleep(0.1)
        
        print("Camera ready!")
        break
    
    if not cap.isOpened():
        print("Error: Could not open webcam after multiple attempts")
        return

    # Set a smaller resolution for better performance
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Variables for calibration
    calibration_stage = 0
    calibration_timer = None
    calibration_positions = []
    
    # Initialize the 3D plot
    fig, ax, scatter = setup_3d_plot()
    
    # Initialize MediaPipe Hands
    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7
    ) as hands:
        while True:
            success, frame = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                time.sleep(0.1)
                continue
            
            # Flip the frame horizontally for a more intuitive mirror view
            frame = cv2.flip(frame, 1)
            
            if not is_calibrated:
                # Run calibration process
                frame, calibration_stage, calibration_timer, calibration_positions, results = run_calibration(
                    frame, hands, calibration_stage, calibration_timer, calibration_positions
                )
            else:
                # Process the frame to find hands
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = hands.process(frame_rgb)
                
                # Draw hand landmarks
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            frame, 
                            hand_landmarks, 
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style()
                        )
                
                # Process hands and update 3D point position
                frame = process_hands(frame, results)
                
                # Update the 3D visualization
                update_3d_plot(fig, ax, scatter)
            
            # Display the frame
            cv2.imshow('Hand Tracking 3D Control', frame)
            
            # Exit on 'q' press or ESC
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 27 is ESC key
                break
    
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    plt.close(fig)


if __name__ == "__main__":
    main() 