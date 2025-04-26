#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for basic arm movements along each axis to aid in calibration
"""

import argparse
import time
from move_to_position_fixed import move_to_position, send_servo_command

# Define a series of test movements
# Format: [name, x, y, z, wrist_pitch, wrist_rotation, gripper, invert_servo3]
TEST_MOVEMENTS = [
    ["Home Position", 0.15, 0.0, 0.2, 0, 0, 0, False],
    ["Forward", 0.2, 0.0, 0.2, 0, 0, 0, False],
    ["Backward", 0.1, 0.0, 0.2, 0, 0, 0, False],
    ["Left", 0.15, 0.05, 0.2, 0, 0, 0, False],
    ["Right", 0.15, -0.05, 0.2, 0, 0, 0, False],
    ["Up", 0.15, 0.0, 0.25, 0, 0, 0, False],
    ["Down", 0.15, 0.0, 0.15, 0, 0, 0, False],
    ["Forward with Inverted Servo 3", 0.2, 0.0, 0.2, 0, 0, 0, True],
    ["Home with Inverted Servo 3", 0.15, 0.0, 0.2, 0, 0, 0, True]
]

def run_test(port, invert_x=False, invert_y=False, invert_z=False):
    """Run the test movements with the specified port"""
    print("\nRunning test movements...")
    
    try:
        # First center all servos
        print("\nCentering all servos...")
        if not send_servo_command(port, "C"):
            print("Failed to center servos. Exiting.")
            return False
        
        # Wait for centering to complete
        time.sleep(1)
        
        # Run each test movement
        for i, test in enumerate(TEST_MOVEMENTS):
            name, x, y, z, wrist_pitch, wrist_rot, gripper, invert_servo3 = test
            
            print(f"\n===== Test {i+1}: {name} =====")
            print(f"Position: X={x}, Y={y}, Z={z}")
            print(f"Wrist Pitch: {wrist_pitch}, Rotation: {wrist_rot}, Gripper: {gripper}")
            if invert_servo3:
                print("Servo 3 will be inverted for this movement")
            
            # Ask for confirmation before each test
            response = input("Run this test? (y/n/q to quit): ")
            if response.lower() == 'q':
                print("Tests aborted.")
                break
            elif response.lower() != 'y':
                print("Skipping this test.")
                continue
            
            # Execute the movement
            move_to_position(
                port, x, y, z, False, invert_x, invert_y, invert_z,
                wrist_pitch, wrist_rot, gripper, invert_servo3
            )
            
            # Wait a moment after the movement
            time.sleep(1)
        
        # Return to center position
        print("\nReturning to center position...")
        send_servo_command(port, "C")
        
        print("\nAll tests completed.")
        return True
        
    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    parser = argparse.ArgumentParser(description='Run test movements for the robot arm')
    parser.add_argument('port', type=str, help='Serial port for ESP32')
    
    # Add calibration options
    parser.add_argument('--invert-x', action='store_true', help='Invert the X direction')
    parser.add_argument('--invert-y', action='store_true', help='Invert the Y direction')
    parser.add_argument('--invert-z', action='store_true', help='Invert the Z direction')
    
    args = parser.parse_args()
    
    run_test(args.port, args.invert_x, args.invert_y, args.invert_z)

if __name__ == "__main__":
    main() 