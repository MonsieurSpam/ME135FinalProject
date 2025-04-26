#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script to control all servos using their calibrated limits
"""

import serial
import time
import sys

# Calibrated joint limits
JOINT_LIMITS = {
    1: {"min": 1027, "max": 2975, "min_deg": 0, "max_deg": 180},  # Clockwise to counterclockwise
    2: {"min": 1582, "max": 2806, "min_deg": 0, "max_deg": 180},  # 0° to 180°
    3: {"min": 1241, "max": 3000, "min_deg": 0, "max_deg": 180},  # Straight to bent
    4: {"min": 1523, "max": 2476, "min_deg": 0, "max_deg": 90},   # Backward to forward
    5: {"min": 1830, "max": 2200, "min_deg": 0, "max_deg": 45},   # Counterclockwise to clockwise
    6: {"min": 2000, "max": 2634, "min_deg": 90, "max_deg": 0}    # Closed to open
}

def center_all_servos(ser):
    """Center all servos"""
    print("\nCentering all servos...")
    ser.write(b'C\n')
    ser.flush()
    time.sleep(2)  # Give time for the servos to center

def move_servo_to_position(ser, servo_id, position):
    """Move a servo to a specific position"""
    command = f"S{servo_id}P{position}\n"
    print(f"Sending command: {command.strip()}")
    ser.write(command.encode('utf-8'))
    ser.flush()
    time.sleep(1)  # Allow time for the servo to move

def test_servo_range(port, servo_id):
    """Test a specific servo through its full range"""
    if servo_id not in JOINT_LIMITS:
        print(f"Error: Invalid servo ID {servo_id}. Must be 1-6.")
        return
    
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=2)
        print(f"Connected to {port}")
        
        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Center all servos first
        center_all_servos(ser)
        
        # Get servo limits
        limits = JOINT_LIMITS[servo_id]
        min_pos = limits["min"]
        max_pos = limits["max"]
        center_pos = min_pos + (max_pos - min_pos) // 2
        
        print(f"\n===== TESTING SERVO {servo_id} =====")
        print(f"Min: {min_pos} ({limits['min_deg']}°)")
        print(f"Max: {max_pos} ({limits['max_deg']}°)")
        print(f"Center: ~{center_pos}")
        
        # Move to minimum position
        print(f"\nMoving to minimum position: {min_pos}")
        move_servo_to_position(ser, servo_id, min_pos)
        input("Press Enter to continue...")
        
        # Move to center position
        print(f"\nMoving to center position: {center_pos}")
        move_servo_to_position(ser, servo_id, center_pos)
        input("Press Enter to continue...")
        
        # Move to maximum position
        print(f"\nMoving to maximum position: {max_pos}")
        move_servo_to_position(ser, servo_id, max_pos)
        input("Press Enter to continue...")
        
        # Sweep from min to max in steps
        print("\nSweeping through range...")
        steps = 5
        step_size = (max_pos - min_pos) // steps
        
        for i in range(steps + 1):
            position = min_pos + (i * step_size)
            print(f"  Position: {position}")
            move_servo_to_position(ser, servo_id, position)
            time.sleep(0.5)
        
        # Return to center
        print("\nReturning to center position")
        move_servo_to_position(ser, servo_id, center_pos)
        
        # Center all servos before exiting
        center_all_servos(ser)
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

def test_all_servos(port):
    """Test all servos sequentially"""
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=2)
        print(f"Connected to {port}")
        
        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Center all servos first
        center_all_servos(ser)
        
        # Test each servo
        for servo_id in JOINT_LIMITS.keys():
            limits = JOINT_LIMITS[servo_id]
            min_pos = limits["min"]
            max_pos = limits["max"]
            
            print(f"\n===== TESTING SERVO {servo_id} =====")
            print(f"Min: {min_pos} ({limits['min_deg']}°)")
            print(f"Max: {max_pos} ({limits['max_deg']}°)")
            
            # Move to minimum position
            print(f"\nMoving to minimum position: {min_pos}")
            move_servo_to_position(ser, servo_id, min_pos)
            time.sleep(2)
            
            # Move to maximum position
            print(f"\nMoving to maximum position: {max_pos}")
            move_servo_to_position(ser, servo_id, max_pos)
            time.sleep(2)
            
            # Center all servos before moving to the next servo
            center_all_servos(ser)
            
            # Ask to continue
            if servo_id < 6:
                response = input(f"Continue to next servo? (y/n): ")
                if response.lower() != 'y':
                    break
        
        # Center all servos before exiting
        center_all_servos(ser)
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_all_servos.py <port> [servo_id]")
        print("Example: python test_all_servos.py /dev/cu.usbserial-59100209741 1")
        print("Example: python test_all_servos.py /dev/cu.usbserial-59100209741 all")
        sys.exit(1)
        
    port = sys.argv[1]
    
    if len(sys.argv) >= 3:
        if sys.argv[2].lower() == 'all':
            test_all_servos(port)
        else:
            try:
                servo_id = int(sys.argv[2])
                if 1 <= servo_id <= 6:
                    test_servo_range(port, servo_id)
                else:
                    print("Error: Servo ID must be between 1 and 6")
            except ValueError:
                print("Error: Servo ID must be an integer")
    else:
        test_all_servos(port) 