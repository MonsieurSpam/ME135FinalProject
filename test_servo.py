#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for servo control using direct serial communication
"""

import serial
import time
import sys

def test_servo(port, servo_id, position):
    """Test moving a servo to a specific position"""
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=2)
        print(f"Connected to {port}")

        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Format and send command
        command = f"S{servo_id}P{position}\n"
        print(f"Sending command: {command.strip()}")
        ser.write(command.encode('utf-8'))
        ser.flush()

        # Read response - read until a timeout or no more data
        print("Response:")
        time.sleep(0.2)  # Short delay to allow response to arrive
        responses = []
        
        # Read all available data
        start_time = time.time()
        while (time.time() - start_time) < 3:  # Read for up to 3 seconds
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                responses.append(line)
                print(f"  {line}")
            else:
                time.sleep(0.1)  # Small delay if no data available
                if len(responses) > 0 and time.time() - start_time > 0.5:
                    # If we've received some data and there's been a pause, assume we're done
                    break

        # Check for OK in the response
        success = any("OK" in resp for resp in responses)
        if success:
            print("\n✅ Command executed successfully")
        else:
            print("\n❌ Command did not complete with OK response")

        # Close connection
        ser.close()
        print("Connection closed")
        
        return success
        
    except Exception as e:
        print(f"Error: {e}")
        return False

def test_center_all(port):
    """Test centering all servos"""
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=2)
        print(f"Connected to {port}")

        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Send center command
        command = "C\n"
        print(f"Sending command: {command.strip()}")
        ser.write(command.encode('utf-8'))
        ser.flush()

        # Read response
        print("Response:")
        time.sleep(0.2)  # Short delay to allow response to arrive
        responses = []
        
        # Read all available data
        start_time = time.time()
        while (time.time() - start_time) < 3:  # Read for up to 3 seconds
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                responses.append(line)
                print(f"  {line}")
            else:
                time.sleep(0.1)  # Small delay if no data available
                if len(responses) > 0 and time.time() - start_time > 0.5:
                    # If we've received some data and there's been a pause, assume we're done
                    break

        # Close connection
        ser.close()
        print("Connection closed")
        
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        return False

def test_servo_limits(port):
    """Test servo positions using calibrated limits"""
    # Joint limits as provided by the user
    joint_limits = {
        1: {"min": 1027, "max": 2975, "min_deg": 0, "max_deg": 180},  # Clockwise to counterclockwise
        2: {"min": 1582, "max": 2806, "min_deg": 0, "max_deg": 180},  # 0° to 180°
        3: {"min": 1241, "max": 3000, "min_deg": 0, "max_deg": 180},  # Straight to bent
        4: {"min": 1523, "max": 2476, "min_deg": 0, "max_deg": 90},   # Backward to forward
        5: {"min": 1830, "max": 2200, "min_deg": 0, "max_deg": 45},   # Counterclockwise to clockwise
        6: {"min": 2000, "max": 2634, "min_deg": 90, "max_deg": 0}    # Closed to open
    }
    
    # First center all servos
    print("\n===== CENTERING ALL SERVOS =====")
    test_center_all(port)
    time.sleep(2)  # Give time for servos to move
    
    # Test each servo at its minimum position
    for servo_id, limits in joint_limits.items():
        min_pos = limits["min"]
        min_deg = limits["min_deg"]
        
        print(f"\n===== TESTING SERVO {servo_id} MINIMUM POSITION =====")
        print(f"Moving to position: {min_pos} ({min_deg}°)")
        test_servo(port, servo_id, min_pos)
        time.sleep(2)  # Give time for servo to move
    
    # Center all servos again
    print("\n===== CENTERING ALL SERVOS AGAIN =====")
    test_center_all(port)
    time.sleep(2)  # Give time for servos to move
    
    # Test each servo at its maximum position
    for servo_id, limits in joint_limits.items():
        max_pos = limits["max"]
        max_deg = limits["max_deg"]
        
        print(f"\n===== TESTING SERVO {servo_id} MAXIMUM POSITION =====")
        print(f"Moving to position: {max_pos} ({max_deg}°)")
        test_servo(port, servo_id, max_pos)
        time.sleep(2)  # Give time for servo to move
    
    # Center all servos one last time
    print("\n===== CENTERING ALL SERVOS ONE LAST TIME =====")
    test_center_all(port)
    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_servo.py <port> [servo_id] [position]")
        print("  OR   python test_servo.py <port> test_limits")
        print("Example: python test_servo.py /dev/cu.usbserial-59100209741 1 2048")
        print("Example: python test_servo.py /dev/cu.usbserial-59100209741 test_limits")
        sys.exit(1)
        
    port = sys.argv[1]
    
    if len(sys.argv) == 3 and sys.argv[2] == "test_limits":
        test_servo_limits(port)
    elif len(sys.argv) >= 4:
        servo_id = int(sys.argv[2])
        position = int(sys.argv[3])
        test_servo(port, servo_id, position)
    else:
        print("Invalid arguments")
        print("Usage: python test_servo.py <port> [servo_id] [position]")
        print("  OR   python test_servo.py <port> test_limits")
        sys.exit(1) 