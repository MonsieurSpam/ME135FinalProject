#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for finding the optimal gripper position (servo 6)
The gripper opens more as values increase above center (2048)
"""

import time
import argparse
from move_to_position_fixed import send_servo_command

def test_gripper_positions(port, start_value=0, end_value=700, step=100, delay=1.0):
    """Test different gripper positions"""
    print("\n---- Gripper Position Test ----\n")
    print("Higher values = more open, Lower values = more closed")
    print("NOTE: Values below center (2048) may cause errors")
    
    # First center all servos to ensure a safe starting position
    print("\nCentering all servos...")
    if not send_servo_command(port, "C"):
        print("Failed to center servos. Exiting.")
        return
    
    # Wait for servos to reach center position
    time.sleep(1)
    
    # Test different positions for servo 6 (gripper)
    current_value = start_value
    while current_value <= end_value:
        pos = 2048 + current_value  # 2048 is center position
        print(f"\nTesting gripper position: {pos} (offset: {current_value})")
        
        # Ensure the position is within valid range
        if pos < 1024 or pos > 3072:
            print(f"Position {pos} is out of valid range (1024-3072). Skipping.")
            current_value += step
            continue
        
        # Set the gripper position
        command = f"S6P{pos}"
        success = send_servo_command(port, command)
        
        if not success:
            print(f"Failed to set gripper position to {pos}. Stopping test.")
            break
            
        # Ask for feedback
        print(f"Gripper position set to {pos}")
        response = input("Is this a good opening position? (y/n/q to quit): ").lower()
        
        if response == 'q':
            print("Test stopped by user.")
            break
        elif response == 'y':
            print(f"\n--- RESULT: Optimal gripper position found ---")
            print(f"Position: {pos}")
            print(f"Offset from center: {current_value}")
            print(f"To use this in your code: --gripper-open {current_value}")
            break
            
        # Move to next position
        current_value += step
    
    # Return to center position
    print("\nReturning to center position...")
    send_servo_command(port, "C")
    print("Test completed.")

def main():
    parser = argparse.ArgumentParser(description='Test different gripper positions')
    parser.add_argument('port', type=str, help='Serial port for ESP32 (e.g., /dev/cu.usbserial-59100209741)')
    parser.add_argument('--start', type=int, default=0, help='Starting offset from center (default: 0)')
    parser.add_argument('--end', type=int, default=700, help='Ending offset from center (default: 700)')
    parser.add_argument('--step', type=int, default=100, help='Step size for testing (default: 100)')
    parser.add_argument('--delay', type=float, default=1.0, help='Delay between movements (default: 1.0)')
    
    args = parser.parse_args()
    
    # Run the gripper test
    test_gripper_positions(
        args.port,
        start_value=args.start,
        end_value=args.end,
        step=args.step,
        delay=args.delay
    )

if __name__ == "__main__":
    main() 