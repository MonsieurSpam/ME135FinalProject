#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Hand Tracking to Robot Arm Control
----------------------------------
This script combines hand tracking with robot arm control.
- Uses hand pinch gestures to control the 3D position
- Sends the position to the robot arm using inverse kinematics
"""

import cv2
import mediapipe as mp
import numpy as np
import time
import math
import os
import argparse
import warnings
from move_to_position_fixed import send_servo_command

# Suppress ikpy warnings
warnings.filterwarnings("ignore", category=UserWarning, module="ikpy.chain")

# Set environment variable for camera authorization
os.environ['OPENCV_AVFOUNDATION_SKIP_AUTH'] = '0'

# Mediapipe Hand settings
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Global variables
point_position = np.array([0.15, 0.0, 0.2])  # Initial robot position (x, y, z)
is_calibrated = False
left_pinch_range = [0.3, 0.7]  # Default [min, max] values for Z control
right_pinch_range = [0.3, 0.7, 0.3, 0.7]  # Default [min_x, max_x, min_y, max_y] for XY control
last_movement_time = 0
movement_cooldown = 0.3  # Reduced cooldown between movements (was 0.5)
position_threshold = 0.01  # Increased threshold to reduce small movements (was 0.005)
process_every_n_frames = 2  # Only process every nth frame
robot_model = None  # Will hold the robot kinematics model
ik_cache = {}  # Cache for IK solutions to avoid repeated calculations
MAX_CACHE_SIZE = 100

# Robot arm workspace limits
ROBOT_LIMITS = {
    'x_min': 0.08, 'x_max': 0.25,  # Forward/backward
    'y_min': -0.1, 'y_max': 0.1,   # Left/right
    'z_min': 0.1, 'z_max': 0.25    # Up/down
}

def map_value(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another."""
    # Ensure the input is within the input range
    value = max(in_min, min(in_max, value))
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def is_pinch(hand_landmarks):
    """Detect if the hand is making a pinch gesture."""
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
    return distance < 0.08

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

def process_hands_for_robot(frame, results, port, invert_x, invert_y, invert_z, invert_servo3, control_mode='continuous', control_interval=1.0):
    """Process the detected hands and update the robot arm position."""
    global point_position, left_pinch_range, right_pinch_range, last_movement_time
    
    frame_height, frame_width, _ = frame.shape
    
    # Process the hands to update the 3D point
    left_hand, right_hand = identify_hands(results)
    position_changed = False
    current_time = time.time()
    time_since_last_movement = current_time - last_movement_time
    
    # Update X-Y position with right hand
    if right_hand and is_pinch(right_hand):
        x, y, _ = get_pinch_position(right_hand)
        
        # Normalize based on calibrated range
        min_x, max_x, min_y, max_y = right_pinch_range
        if max_x > min_x and max_y > min_y:
            # Map hand position to robot workspace
            robot_x = map_value(x, min_x, max_x, ROBOT_LIMITS['x_min'], ROBOT_LIMITS['x_max'])
            robot_y = map_value(y, min_y, max_y, ROBOT_LIMITS['y_min'], ROBOT_LIMITS['y_max'])
            
            # Check if position has changed enough to update
            if abs(robot_x - point_position[0]) > position_threshold or abs(robot_y - point_position[1]) > position_threshold:
                point_position[0] = robot_x
                point_position[1] = robot_y
                position_changed = True
        
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
            # Map hand position to robot workspace for Z
            robot_z = map_value(y, min_y, max_y, ROBOT_LIMITS['z_max'], ROBOT_LIMITS['z_min'])
            
            # Check if position has changed enough to update
            if abs(robot_z - point_position[2]) > position_threshold:
                point_position[2] = robot_z
                position_changed = True
        
        # Draw visual feedback
        cx, cy = int(0.2 * frame_width), int(y * frame_height)
        cv2.circle(frame, (cx, cy), 15, (255, 0, 0), -1)
        cv2.putText(frame, f"Z: {point_position[2]:.2f}", 
                    (cx + 20, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    # Draw the current robot position
    cv2.putText(frame, f"Robot Target: ({point_position[0]:.2f}, {point_position[1]:.2f}, {point_position[2]:.2f})", 
                (20, frame_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Add time indicator for next movement
    time_to_next = max(0, control_interval - time_since_last_movement)
    if time_to_next > 0:
        cv2.putText(frame, f"Next update in: {time_to_next:.1f}s", 
                    (20, frame_height - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    else:
        cv2.putText(frame, "Ready to move", 
                    (20, frame_height - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # Move the robot if needed and enough time has passed
    if position_changed and time_since_last_movement >= control_interval:
        if control_mode == 'continuous':
            # In continuous mode, send commands constantly as hand moves
            move_robot(port, invert_x, invert_y, invert_z, invert_servo3)
            last_movement_time = current_time
        elif control_mode == 'pinch':
            # In pinch mode, only move when both hands are pinching
            if left_hand and right_hand and is_pinch(left_hand) and is_pinch(right_hand):
                move_robot(port, invert_x, invert_y, invert_z, invert_servo3)
                last_movement_time = current_time
    
    return frame

def move_robot(port, invert_x, invert_y, invert_z, invert_servo3):
    """Send the movement command to the robot."""
    global robot_model
    
    # Round position to reduce unique positions (improves caching)
    x = round(point_position[0], 3)
    y = round(point_position[1], 3)
    z = round(point_position[2], 3)
    
    print(f"Moving robot to: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
    
    # Create cache key
    cache_key = (x, y, z, invert_x, invert_y, invert_z, invert_servo3)
    
    # Check if we have this solution cached
    if cache_key in ik_cache:
        command = ik_cache[cache_key]
        return send_servo_command(port, command)
    
    try:
        # Load the robot model if needed
        if robot_model is None:
            from robot_ikpy_kinematics import RobotArmIKPy
            # No global declaration inside a function!
            robot_model = RobotArmIKPy()
        
        # Apply direction inversion if needed
        target_x, target_y, target_z = x, y, z
        if invert_x:
            target_x = -target_x
        if invert_y:
            target_y = -target_y
        if invert_z:
            target_z = -target_z
        
        # Calculate inverse kinematics
        joint_angles = robot_model.inverse_kinematics([target_x, target_y, target_z])
        
        # Convert to Dynamixel positions
        dynamixel_positions = robot_model.angles_to_dynamixel(joint_angles)
        
        # Apply inversion for Servo 3 if requested
        if invert_servo3:
            # Calculate the center position
            center = (robot_model.joint_dxl_limits[2][0] + robot_model.joint_dxl_limits[2][1]) / 2
            # Invert the position relative to center
            dynamixel_positions[2] = int(center * 2 - dynamixel_positions[2])
        
        # Add default values for wrist and gripper
        dynamixel_positions[3] += 0  # Wrist pitch
        dynamixel_positions[4] += 0  # Wrist rotation
        dynamixel_positions[5] += 0  # Gripper
        
        # Ensure positions are within valid ranges
        for i in range(len(dynamixel_positions)):
            # Default limits for Dynamixel servos
            min_pos = 1024
            max_pos = 3072
            
            # Apply specific limits from robot model if available
            if hasattr(robot_model, 'joint_dxl_limits') and i < len(robot_model.joint_dxl_limits):
                min_pos = robot_model.joint_dxl_limits[i][0]
                max_pos = robot_model.joint_dxl_limits[i][1]
            
            dynamixel_positions[i] = max(min_pos, min(max_pos, dynamixel_positions[i]))
            
        # Create the command
        command = f"M{dynamixel_positions[0]},{dynamixel_positions[1]},{dynamixel_positions[2]},{dynamixel_positions[3]},{dynamixel_positions[4]},{dynamixel_positions[5]}"
        
        # Cache the result
        if len(ik_cache) >= MAX_CACHE_SIZE:
            # Remove a random item if cache is full
            ik_cache.pop(next(iter(ik_cache)))
        ik_cache[cache_key] = command
        
        # Send the command
        result = send_servo_command(port, command)
        return result
    except Exception as e:
        print(f"Error moving robot: {e}")
        import traceback
        traceback.print_exc()
        return False

def initialize_robot_model():
    """Initialize the robot model."""
    global robot_model
    try:
        from robot_ikpy_kinematics import RobotArmIKPy
        print("Initializing robot model...")
        robot_model = RobotArmIKPy()
        print("Robot model initialized.")
        return True
    except Exception as e:
        print(f"Warning: Could not initialize robot model: {e}")
        return False

def main():
    """Main function to run the hand tracking and robot arm control."""
    global is_calibrated, point_position
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Control robot arm with hand tracking')
    parser.add_argument('port', type=str, help='Serial port for ESP32')
    
    # Add calibration options
    parser.add_argument('--invert-x', action='store_true', help='Invert the X direction')
    parser.add_argument('--invert-y', action='store_true', help='Invert the Y direction')
    parser.add_argument('--invert-z', action='store_true', help='Invert the Z direction')
    parser.add_argument('--invert-servo3', action='store_true', help='Invert the elbow servo')
    parser.add_argument('--mode', choices=['continuous', 'pinch'], default='continuous',
                        help='Control mode: continuous (default) or pinch (both hands must pinch)')
    parser.add_argument('--resolution', type=str, choices=['low', 'medium', 'high'], default='low',
                        help='Camera resolution: low (320x240), medium (640x480), high (1280x720)')
    parser.add_argument('--camera-index', type=int, default=1, 
                        help='Camera index to use (default: 1)')
    parser.add_argument('--no-flip', action='store_true',
                        help='Disable horizontal flipping of the camera image')
    parser.add_argument('--control-freq', type=float, default=1.0,
                        help='Control frequency in Hz - how many times per second to update the robot position (default: 1.0)')
    
    args = parser.parse_args()
    
    # Calculate control interval based on frequency
    control_interval = 1.0 / args.control_freq
    
    print("Starting Hand Tracking Robot Control")
    print(f"Control mode: {args.mode}")
    print(f"Camera horizontal flip: {'disabled' if args.no_flip else 'enabled'}")
    print(f"Robot control frequency: {args.control_freq} Hz (every {control_interval:.2f} seconds)")
    print("Press 'q' or ESC to exit, 'r' to reset position, 'f' to toggle flipping")
    
    # Initialize robot model
    initialize_robot_model()
    
    # Initialize webcam with retry logic
    max_retries = 3
    camera_opened = False
    
    for attempt in range(max_retries):
        print(f"Attempting to connect to camera (attempt {attempt+1}/{max_retries})...")
        cap = cv2.VideoCapture(args.camera_index)  # Use camera index from arguments
        
        if not cap.isOpened():
            print(f"Failed to open camera on attempt {attempt+1}")
            if attempt < max_retries - 1:
                print("Retrying in 2 seconds...")
                time.sleep(2)
            continue
        
        # Set camera buffer properties to minimize latency
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer size
        
        # Try to read a test frame to verify camera is working
        print("Testing camera...")
        valid_frames = 0
        test_attempts = 20  # Try more times to get valid frames
        
        for _ in range(test_attempts):
            ret, test_frame = cap.read()
            if ret and test_frame is not None and test_frame.size > 0:
                valid_frames += 1
                if valid_frames >= 3:  # Require at least 3 valid frames
                    camera_opened = True
                    break
            time.sleep(0.1)
        
        if camera_opened:
            print(f"Camera ready! Received {valid_frames} valid frames.")
            break
        else:
            print(f"Camera opened but couldn't get stable video feed. Only got {valid_frames} valid frames.")
            cap.release()
            if attempt < max_retries - 1:
                print(f"Trying again with different settings...")
                time.sleep(1)
    
    if not camera_opened:
        print("Error: Could not get a stable video feed after multiple attempts")
        print("Try with a different camera index using --camera-index option")
        return

    # Set resolution based on argument
    if args.resolution == 'low':
        width, height = 320, 240
    elif args.resolution == 'medium':
        width, height = 640, 480
    else:  # high
        width, height = 1280, 720
        
    print(f"Setting camera resolution to {width}x{height}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    # First center all servos
    print("\nCentering all servos...")
    if not send_servo_command(args.port, "C"):
        print("Failed to center servos. Exiting.")
        return
    
    # Wait for centering to complete
    time.sleep(1)
    
    # Variables for calibration
    calibration_stage = 0
    calibration_timer = None
    calibration_positions = []
    frame_count = 0
    flip_horizontally = not args.no_flip  # Flip by default unless --no-flip is specified
    
    # Variables for freeze detection
    last_frame_time = time.time()
    freeze_threshold = 1.0  # Consider camera frozen if no frame for 1 second
    
    # Initialize MediaPipe Hands with lower detection confidence for better performance
    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.6,  # Lowered from 0.7
        min_tracking_confidence=0.6,   # Lowered from 0.7
        model_complexity=0             # Use simplest model (0, 1, or 2)
    ) as hands:
        empty_frame_counter = 0
        max_empty_frames = 30  # Maximum consecutive empty frames before giving up
        
        while True:
            # Check for potential camera freeze
            current_time = time.time()
            if current_time - last_frame_time > freeze_threshold:
                print(f"Potential camera freeze detected ({current_time - last_frame_time:.1f}s without frames)")
                # Try to recover by grabbing a new frame explicitly
                cap.grab()
            
            success, frame = cap.read()
            
            # Update last frame time on successful reads
            if success and frame is not None and frame.size > 0:
                last_frame_time = current_time
            
            if not success or frame is None or frame.size == 0:
                empty_frame_counter += 1
                print(f"Ignoring empty camera frame ({empty_frame_counter}/{max_empty_frames}).")
                
                # If too many consecutive empty frames, try to reset the camera
                if empty_frame_counter >= max_empty_frames:
                    print("Too many empty frames. Attempting to reset camera...")
                    cap.release()
                    time.sleep(1)
                    cap = cv2.VideoCapture(args.camera_index)
                    
                    if not cap.isOpened():
                        print("Failed to reopen camera. Exiting.")
                        break
                    
                    # Set resolution again
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer size
                    empty_frame_counter = 0
                    last_frame_time = time.time()
                
                time.sleep(0.1)
                continue
            
            # Reset counter when we get a valid frame
            empty_frame_counter = 0
            
            # Increment frame counter
            frame_count += 1
            
            # Skip frames for better performance
            if frame_count % process_every_n_frames != 0 and is_calibrated:
                # Just show the frame without processing
                if flip_horizontally:
                    frame = cv2.flip(frame, 1)
                
                cv2.imshow('Hand Tracking Robot Control', frame)
                
                # Check for key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 27 is ESC key
                    break
                elif key == ord('r'):  # Reset to home position
                    point_position = np.array([0.15, 0.0, 0.2])
                    send_servo_command(args.port, "C")
                    print("Reset to home position")
                elif key == ord('f'):  # Toggle horizontal flipping
                    flip_horizontally = not flip_horizontally
                    print(f"Horizontal flipping: {'enabled' if flip_horizontally else 'disabled'}")
                    
                continue
            
            # Flip the frame horizontally if needed
            if flip_horizontally:
                frame = cv2.flip(frame, 1)
            
            if not is_calibrated:
                # Run calibration process
                frame, calibration_stage, calibration_timer, calibration_positions, results = run_calibration(
                    frame, hands, calibration_stage, calibration_timer, calibration_positions
                )
            else:
                # Process the frame to find hands - convert to RGB only once
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Set image as not writeable to improve performance
                frame_rgb.flags.writeable = False
                results = hands.process(frame_rgb)
                frame_rgb.flags.writeable = True
                
                # Draw hand landmarks if they exist
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            frame, 
                            hand_landmarks, 
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style()
                        )
                
                # Process hands for robot control
                frame = process_hands_for_robot(
                    frame, results, args.port, 
                    args.invert_x, args.invert_y, args.invert_z, args.invert_servo3,
                    args.mode, control_interval
                )
            
            # Add performance and status info
            fps_text = f"FPS: {int(cap.get(cv2.CAP_PROP_FPS))}"
            cv2.putText(frame, fps_text, (width - 100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            flip_status = "Flipped" if flip_horizontally else "Normal"
            cv2.putText(frame, flip_status, (width - 100, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # Display the frame
            cv2.imshow('Hand Tracking Robot Control', frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 27 is ESC key
                break
            elif key == ord('r'):  # Reset to home position
                point_position = np.array([0.15, 0.0, 0.2])
                send_servo_command(args.port, "C")
                print("Reset to home position")
            elif key == ord('f'):  # Toggle horizontal flipping
                flip_horizontally = not flip_horizontally
                print(f"Horizontal flipping: {'enabled' if flip_horizontally else 'disabled'}")
    
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    
    # Return to center position
    print("\nReturning to center position...")
    send_servo_command(args.port, "C")

if __name__ == "__main__":
    main() 