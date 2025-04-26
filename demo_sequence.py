#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Demo sequence for robot arm using inverse kinematics
"""

import time
import argparse
from move_to_position_fixed import send_servo_command, move_to_position

# Waypoints for the demo sequence
WAYPOINTS = [
    # X, Y, Z, wrist_pitch, gripper_close, description
    (0.15, 0.0, 0.15, 0, 0, "Home position - gripper at center"),
    (0.15, 0.0, 0.15, 300, 500, "Open gripper fully and point wrist down"),
    (0.18, 0.0, 0.15, 300, 500, "Extend forward"),
    (0.15, 0.0, 0.15, 300, 500, "Retract"),
    (0.15, 0.08, 0.15, 300, 500, "Move right"),
    (0.15, -0.08, 0.15, 300, 500, "Move left"),
    (0.15, 0.0, 0.15, 300, 500, "Center horizontally"),
    (0.15, 0.0, 0.22, 300, 500, "Move up high"),
    (0.15, 0.0, 0.15, 300, 500, "Move down"),
    (0.15, 0.0, 0.15, 0, 500, "Level wrist, keep gripper open"),
    (0.15, 0.0, 0.15, 0, 0, "Return gripper to center position"),
    # Pick and place demo - using center position as the closed position
    (0.18, 0.08, 0.15, 300, 500, "Position to pick - right side, gripper open"),
    (0.18, 0.08, 0.12, 300, 500, "Lower to object with gripper open"),
    (0.18, 0.08, 0.12, 300, 0, "Close gripper to grab object (center position)"),
    (0.18, 0.08, 0.18, 300, 0, "Lift object with gripper closed"),
    (0.18, -0.08, 0.18, 300, 0, "Move object to left side"),
    (0.18, -0.08, 0.12, 300, 0, "Lower object"),
    (0.18, -0.08, 0.12, 300, 500, "Open gripper to release object"),
    (0.18, -0.08, 0.18, 300, 500, "Raise after placing with gripper open"),
    (0.15, 0.0, 0.15, 0, 0, "Return to home position with gripper center")
]

def run_demo_sequence(port, invert_x=True, invert_y=False, delay=2.0):
    """Run a demo sequence of waypoints"""
    print("\n---- Robot Arm Demo Sequence ----\n")
    
    # First center all servos to ensure a safe starting position
    print("Centering all servos...")
    if not send_servo_command(port, "C"):
        print("Failed to center servos. Exiting.")
        return
    
    # Wait for servos to reach center position
    time.sleep(1)
    
    # Go through each waypoint
    for i, (x, y, z, wrist_pitch, gripper_open, description) in enumerate(WAYPOINTS):
        print(f"\nWaypoint {i+1}/{len(WAYPOINTS)}: {description}")
        print(f"Position: X={x}, Y={y}, Z={z}")
        print(f"Wrist pitch: {wrist_pitch}, Gripper (open amount): {gripper_open}")
        
        # Move to the waypoint
        success = move_to_position(
            port, x, y, z, False,
            invert_x, invert_y, False,
            wrist_pitch, 0, gripper_open
        )
        
        if not success:
            print(f"Failed to reach waypoint {i+1}. Stopping sequence.")
            break
        
        # Delay before next movement
        print(f"Waiting {delay} seconds before next movement...")
        time.sleep(delay)
    
    print("\n---- Demo Sequence Completed ----")

def main():
    parser = argparse.ArgumentParser(description='Run a demo sequence for the robot arm')
    parser.add_argument('port', type=str, help='Serial port for ESP32 (e.g., /dev/cu.usbserial-59100209741)')
    parser.add_argument('--no-invert-x', action='store_true', help='Do not invert the X direction')
    parser.add_argument('--invert-y', action='store_true', help='Invert the Y direction')
    parser.add_argument('--delay', type=float, default=2.0, help='Delay between movements in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    # Run the demo sequence
    run_demo_sequence(
        args.port,
        invert_x=not args.no_invert_x,
        invert_y=args.invert_y,
        delay=args.delay
    )

if __name__ == "__main__":
    main() 