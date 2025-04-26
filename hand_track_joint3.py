#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Hand Tracking to Control Servo 3 (Elbow Joint)
----------------------------------------------
This script uses hand tracking to control only the elbow joint (Servo 3).
- Uses vertical hand position to control the servo angle
- Keeps other servos in a fixed position
- Uses caching to improve performance
- Uses multithreading to separate CV from servo control
"""

import cv2
import mediapipe as mp
import numpy as np
import time
import math
import os
import argparse
import warnings
import threading
import queue
from move_to_position_fixed import send_servo_command

# Suppress warnings
warnings.filterwarnings("ignore", category=UserWarning)

# Set environment variable for camera authorization
os.environ['OPENCV_AVFOUNDATION_SKIP_AUTH'] = '0'

# Mediapipe Hand settings
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Global variables
is_calibrated = False
hand_y_range = [0.3, 0.7]  # Default [min, max] values for vertical hand position
last_movement_time = 0
movement_cooldown = 0.2  # Short cooldown between movements
process_every_n_frames = 5  # Increased frame skipping for better performance

# Servo 3 limits from robot_ikpy_kinematics.py
SERVO_3_MIN = 1241  # Fully extended position
SERVO_3_MAX = 3000  # Fully bent position
SERVO_3_CENTER = 2048  # Center position

# Position cache to avoid recalculating similar positions
# Map from y position (rounded to 2 decimal places) to servo position
position_cache = {}
# Threshold for position change to be considered significant
POSITION_THRESHOLD = 0.01

# Threading variables
position_queue = queue.Queue(maxsize=10)  # Queue for passing hand positions between threads
exit_event = threading.Event()  # Event to signal threads to exit
servo_position_lock = threading.Lock()  # Lock for updating the servo position
current_servo_position = SERVO_3_CENTER  # Shared servo position between threads

# Optimization: Pre-calculated positions for Servo 3
# This completely bypasses IK computation for direct hand-to-servo mapping
def direct_map_to_servo3(y_position, min_y, max_y):
    """
    Directly map hand Y position to Servo 3 position without IK calculations.
    This is much faster than using the full IK solver.
    """
    # Make sure y is within range
    y = max(min_y, min(max_y, y_position))
    
    # Linear interpolation from hand position to servo position
    # Higher hand = more extended (lower position value)
    mapped_value = SERVO_3_MIN + (y - min_y) * (SERVO_3_MAX - SERVO_3_MIN) / (max_y - min_y)
    return int(mapped_value)

def map_value(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another."""
    # Ensure the input is within the input range
    value = max(in_min, min(in_max, value))
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def get_hand_position(hand_landmarks):
    """Get the position of the hand (using palm center)."""
    if not hand_landmarks:
        return None
    
    # Use wrist position only for faster calculation
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    return (wrist.x, wrist.y, wrist.z)

def identify_hands(results):
    """Identify hand in the frame."""
    if not results.multi_hand_landmarks:
        return None
    
    # Just return the first detected hand
    if results.multi_hand_landmarks:
        return results.multi_hand_landmarks[0]
    
    return None

def run_calibration(frame, hands, calibration_stage, calibration_timer, calibration_positions):
    """Run the calibration process."""
    global is_calibrated, hand_y_range, position_cache
    
    # Process the frame to find hands
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_rgb.flags.writeable = False  # Performance optimization
    results = hands.process(frame_rgb)
    frame_rgb.flags.writeable = True
    
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
    
    hand = identify_hands(results)
    frame_height, frame_width, _ = frame.shape
    
    # Display calibration instructions
    if calibration_stage == 0:
        # Starting calibration - instructions
        cv2.putText(frame, "Calibration: We will calibrate hand movement range", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, "Move your hand up and down", 
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
        # Y Calibration
        cv2.putText(frame, "Hand Calibration (vertical movement)", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, "Move your hand up and down", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Time left: {max(0, int(5 - (time.time() - calibration_timer)))} seconds", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if hand:
            hand_pos = get_hand_position(hand)
            if hand_pos:
                _, y, _ = hand_pos
                
                # Record the hand positions for calibration
                calibration_positions.append(y)
                
                # Draw the hand position point
                cx, cy = int(0.5 * frame_width), int(y * frame_height)
                cv2.circle(frame, (cx, cy), 15, (0, 255, 0), -1)
        
        # Move to next stage after 5 seconds
        if time.time() - calibration_timer > 5:
            # Calculate the hand range
            if calibration_positions:
                hand_y_range = [min(calibration_positions), max(calibration_positions)]
                # Expand slightly to give some margin
                range_size = hand_y_range[1] - hand_y_range[0]
                hand_y_range[0] -= range_size * 0.1
                hand_y_range[1] += range_size * 0.1
            
            # Clear cache when calibration changes
            position_cache.clear()
            
            calibration_stage = 2
            calibration_timer = time.time()
    
    elif calibration_stage == 2:
        # Calibration complete
        cv2.putText(frame, "Calibration Complete!", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, "Move hand up/down to control elbow joint", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"Y-range: {hand_y_range[0]:.2f} - {hand_y_range[1]:.2f}", 
                    (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        if time.time() - calibration_timer > 2:
            is_calibrated = True
    
    return frame, calibration_stage, calibration_timer, calibration_positions, results

def process_hand_for_servo3(frame, results):
    """Process the detected hand and update servo 3 position."""
    global hand_y_range, position_cache, current_servo_position
    
    frame_height, frame_width, _ = frame.shape
    
    # Process the hand to update servo position
    hand = identify_hands(results)
    position_changed = False
    servo_position = SERVO_3_CENTER  # Default to center position
    
    # Update servo position with hand
    if hand:
        hand_pos = get_hand_position(hand)
        if hand_pos:
            _, y, _ = hand_pos
            
            # Round y to reduce number of unique positions
            y_rounded = round(y, 2)
            
            # Check if this position is in cache
            if y_rounded in position_cache:
                servo_position = position_cache[y_rounded]
                position_changed = True
            else:
                # Normalize based on calibrated range
                min_y, max_y = hand_y_range
                if max_y > min_y:
                    # Fast direct mapping without IK
                    new_servo_position = direct_map_to_servo3(y, min_y, max_y)
                    
                    # Check if position has changed significantly
                    with servo_position_lock:
                        if abs(new_servo_position - current_servo_position) > (SERVO_3_MAX - SERVO_3_MIN) * POSITION_THRESHOLD:
                            servo_position = new_servo_position
                            position_changed = True
                            
                            # Cache this position
                            position_cache[y_rounded] = servo_position
            
            # Draw visual feedback
            cx, cy = int(0.5 * frame_width), int(y * frame_height)
            cv2.circle(frame, (cx, cy), 15, (0, 255, 0), -1)
            
            # Map current position to angle for display
            angle = map_value(servo_position, SERVO_3_MIN, SERVO_3_MAX, 0, 180)
            cv2.putText(frame, f"Servo 3: {servo_position} ({angle:.0f}°)", 
                        (cx - 100, cy + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # If position changed, put it in the queue for the servo control thread
    if position_changed:
        try:
            # Non-blocking put to avoid slowing down the CV thread
            position_queue.put_nowait((servo_position, time.time()))
        except queue.Full:
            # Queue is full, just continue
            pass
    
    return frame, servo_position if position_changed else None

def servo_control_thread(port, invert_servo3, control_interval):
    """Thread function to handle servo control."""
    global exit_event, current_servo_position
    
    print("Servo control thread started")
    last_movement_time = 0
    
    # Reconnect optimization: Only reconnect to serial port when needed
    reconnect_interval = 10.0  # Seconds between reconnection attempts
    last_reconnect = 0
    send_attempts = 0
    max_fail_attempts = 3
    
    while not exit_event.is_set():
        current_time = time.time()
        
        try:
            # Get the latest position from the queue with a timeout
            servo_position, timestamp = position_queue.get(timeout=0.1)
            
            # Check if enough time has passed since last movement
            time_since_last_movement = current_time - last_movement_time
            
            if time_since_last_movement >= control_interval:
                # Invert servo position if requested
                final_position = servo_position
                if invert_servo3:
                    # Center is 2048, calculate the inverted position
                    final_position = int(SERVO_3_CENTER * 2 - servo_position)
                    # Ensure within limits
                    final_position = max(SERVO_3_MIN, min(SERVO_3_MAX, final_position))
                
                # Optimized command formatting - direct string concatenation
                command = "S3P" + str(final_position)
                
                # Send command
                success = send_servo_command(port, command)
                
                if success:
                    # Update the current position
                    with servo_position_lock:
                        current_servo_position = servo_position
                    
                    last_movement_time = current_time
                    send_attempts = 0  # Reset failure counter
                else:
                    send_attempts += 1
                    if send_attempts >= max_fail_attempts:
                        # Too many failures, try reconnecting if enough time passed
                        if current_time - last_reconnect > reconnect_interval:
                            print("Multiple command failures. Will try to reconnect next time.")
                            last_reconnect = current_time
                            send_attempts = 0
                
                # Mark the task as done
                position_queue.task_done()
            else:
                # Not enough time has passed, put it back in the queue
                try:
                    position_queue.put_nowait((servo_position, timestamp))
                except queue.Full:
                    pass
                position_queue.task_done()
                time.sleep(0.05)  # Short sleep to avoid CPU spinning
        
        except queue.Empty:
            # No position in queue, just continue
            pass
        
        except Exception as e:
            print(f"Error in servo control thread: {e}")
            time.sleep(0.1)  # Sleep to avoid spinning on errors
    
    print("Servo control thread exiting")

def main():
    """Main function to run the hand tracking and servo control."""
    global is_calibrated, exit_event
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Control Servo 3 (elbow) with hand tracking')
    parser.add_argument('port', type=str, help='Serial port for ESP32')
    parser.add_argument('--invert-servo3', action='store_true', help='Invert the elbow servo')
    parser.add_argument('--resolution', type=str, choices=['low', 'medium', 'high'], default='low',
                        help='Camera resolution: low (320x240), medium (640x480), high (1280x720)')
    parser.add_argument('--camera-index', type=int, default=0, 
                        help='Camera index to use (default: 0)')
    parser.add_argument('--no-flip', action='store_true',
                        help='Disable horizontal flipping of the camera image')
    parser.add_argument('--control-freq', type=float, default=0.25,
                        help='Control frequency in Hz - how many times per second to update servo position (default: 0.25)')
    parser.add_argument('--skip-frames', type=int, default=5,
                        help='Number of frames to skip between processing (default: 5)')
    
    args = parser.parse_args()
    
    # Apply custom frame skipping if provided
    global process_every_n_frames
    process_every_n_frames = args.skip_frames
    
    # Calculate control interval based on frequency
    control_interval = 1.0 / args.control_freq
    
    print("Starting Hand Tracking for Servo 3 Control")
    print(f"Camera horizontal flip: {'disabled' if args.no_flip else 'enabled'}")
    print(f"Control frequency: {args.control_freq} Hz (every {control_interval:.2f} seconds)")
    print(f"Frame skipping: {process_every_n_frames} (processing 1 in every {process_every_n_frames} frames)")
    print(f"Direct Mapping: Enabled (bypassing IK completely)")
    print("Press 'q' or ESC to exit, 'r' to reset position, 'c' to recalibrate")
    
    # Initialize webcam with retry logic
    max_retries = 3
    camera_opened = False
    
    for attempt in range(max_retries):
        print(f"Attempting to connect to camera (attempt {attempt+1}/{max_retries})...")
        cap = cv2.VideoCapture(args.camera_index)
        
        if not cap.isOpened():
            print(f"Failed to open camera on attempt {attempt+1}")
            if attempt < max_retries - 1:
                print("Retrying in 2 seconds...")
                time.sleep(2)
            continue
        
        # Set camera buffer properties to minimize latency
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Try to read a test frame to verify camera is working
        print("Testing camera...")
        valid_frames = 0
        test_attempts = 20
        
        for _ in range(test_attempts):
            ret, test_frame = cap.read()
            if ret and test_frame is not None and test_frame.size > 0:
                valid_frames += 1
                if valid_frames >= 3:
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
    
    # Center all servos first
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
    flip_horizontally = not args.no_flip
    
    # Variables for freeze detection
    last_frame_time = time.time()
    freeze_threshold = 1.0  # Consider camera frozen if no frame for 1 second
    
    # Start the servo control thread
    servo_thread = threading.Thread(
        target=servo_control_thread,
        args=(args.port, args.invert_servo3, control_interval),
        daemon=True
    )
    servo_thread.start()
    
    # Initialize MediaPipe Hands with lower detection confidence for better performance
    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,  # Only track one hand for simplicity
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=0  # Use simplest model for better performance
    ) as hands:
        empty_frame_counter = 0
        max_empty_frames = 30
        last_servo_position = SERVO_3_CENTER  # Remember last position
        
        # Pre-create window for faster display
        cv2.namedWindow('Hand Tracking Servo 3 Control', cv2.WINDOW_NORMAL)
        
        try:
            while not exit_event.is_set():
                # Check for potential camera freeze
                current_time = time.time()
                if current_time - last_frame_time > freeze_threshold:
                    print(f"Potential camera freeze detected ({current_time - last_frame_time:.1f}s without frames)")
                    cap.grab()  # Try to reset the frame buffer
                
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
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        empty_frame_counter = 0
                        last_frame_time = time.time()
                    
                    time.sleep(0.1)
                    continue
                
                # Reset counter when we get a valid frame
                empty_frame_counter = 0
                
                # Increment frame counter
                frame_count += 1
                
                # Skip frames for better performance - more aggressive skipping
                if frame_count % process_every_n_frames != 0 and is_calibrated:
                    # Just show the frame without processing
                    if flip_horizontally:
                        frame = cv2.flip(frame, 1)
                    
                    # Add servo info to unprocessed frames
                    with servo_position_lock:
                        angle = map_value(current_servo_position, SERVO_3_MIN, SERVO_3_MAX, 0, 180)
                        cv2.putText(frame, f"Current servo: {current_servo_position} ({angle:.0f}°)", 
                                    (20, height - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    cv2.imshow('Hand Tracking Servo 3 Control', frame)
                    
                    # Check for key presses
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 27 is ESC key
                        break
                    elif key == ord('r'):  # Reset to center position
                        # Put center position in queue
                        try:
                            position_queue.put_nowait((SERVO_3_CENTER, time.time()))
                        except queue.Full:
                            pass
                        print("Reset servo 3 to center position")
                    elif key == ord('c'):  # Recalibrate
                        is_calibrated = False
                        calibration_stage = 0
                        print("Recalibrating...")
                    elif key == ord('f'):  # Toggle horizontal flipping
                        flip_horizontally = not flip_horizontally
                        print(f"Horizontal flipping: {'enabled' if flip_horizontally else 'disabled'}")
                        
                    continue
                
                # Flip the frame horizontally if needed
                if flip_horizontally:
                    frame = cv2.flip(frame, 1)
                
                start_process_time = time.time()
                
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
                    
                    # Process hand for servo 3 control
                    frame, new_position = process_hand_for_servo3(frame, results)
                    
                    # Update last position if changed
                    if new_position is not None:
                        last_servo_position = new_position
                
                # Calculate processing time
                process_time = time.time() - start_process_time
                
                # Add performance and status info
                fps_text = f"FPS: {int(cap.get(cv2.CAP_PROP_FPS))}"
                cv2.putText(frame, fps_text, (width - 100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                proc_text = f"Process: {process_time*1000:.1f}ms" 
                cv2.putText(frame, proc_text, (width - 150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                queue_text = f"Queue: {position_queue.qsize()}"
                cv2.putText(frame, queue_text, (width - 150, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                flip_status = "Flipped" if flip_horizontally else "Normal"
                cv2.putText(frame, flip_status, (width - 100, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                # Show servo range visualization
                bar_width = int(width * 0.8)
                bar_height = 20
                bar_x = int(width * 0.1)
                bar_y = height - 100
                
                # Draw background bar
                cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (50, 50, 50), -1)
                
                # Draw current position if calibrated
                if is_calibrated and results.multi_hand_landmarks:
                    hand = identify_hands(results)
                    if hand:
                        hand_pos = get_hand_position(hand)
                        if hand_pos:
                            _, y, _ = hand_pos
                            # Map hand position to bar position
                            position_ratio = (y - hand_y_range[0]) / (hand_y_range[1] - hand_y_range[0])
                            position_ratio = max(0, min(1, position_ratio))
                            position_x = bar_x + int(position_ratio * bar_width)
                            
                            # Draw position marker
                            cv2.circle(frame, (position_x, bar_y + bar_height//2), 10, (0, 255, 0), -1)
                
                # Label the bar
                cv2.putText(frame, "Hand Range", (bar_x, bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, "Extended", (bar_x, bar_y + bar_height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, "Bent", (bar_x + bar_width - 30, bar_y + bar_height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Display cache size
                cv2.putText(frame, f"Cache: {len(position_cache)}", 
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # Display threading status
                cv2.putText(frame, "Threaded: Yes", 
                        (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # Display direct mapping status
                cv2.putText(frame, "Direct Mapping: Yes", 
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # Display the frame
                cv2.imshow('Hand Tracking Servo 3 Control', frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 27 is ESC key
                    break
                elif key == ord('r'):  # Reset to center position
                    # Put center position in queue
                    try:
                        position_queue.put_nowait((SERVO_3_CENTER, time.time()))
                    except queue.Full:
                        pass
                    print("Reset servo 3 to center position")
                elif key == ord('c'):  # Recalibrate
                    is_calibrated = False
                    calibration_stage = 0
                    position_cache.clear()  # Clear cache on recalibration
                    print("Recalibrating...")
                elif key == ord('f'):  # Toggle horizontal flipping
                    flip_horizontally = not flip_horizontally
                    print(f"Horizontal flipping: {'enabled' if flip_horizontally else 'disabled'}")
        
        finally:
            # Clean up
            exit_event.set()  # Signal all threads to exit
            
            # Wait for servo thread to finish
            print("Waiting for servo thread to finish...")
            if servo_thread.is_alive():
                servo_thread.join(timeout=2.0)
            
            # Release camera
            cap.release()
            cv2.destroyAllWindows()
            
            # Return to center position
            print("\nReturning all servos to center position...")
            send_servo_command(args.port, "C")
            print("Done.")

if __name__ == "__main__":
    main() 