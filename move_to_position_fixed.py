#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to move the robot arm to a specific position using inverse kinematics
with corrected servo mappings
"""

import argparse
import numpy as np
import time
import subprocess
import sys
from math import pi, degrees
import serial

# Import the robot kinematics class
from robot_ikpy_kinematics import RobotArmIKPy

# Define baud rate for serial communication
BAUD_RATE = 115200

def send_servo_command(port, command):
    """Send a command to the ESP32 using servo_command.py"""
    try:
        print(f"Sending command: {command}")
        result = subprocess.run(['python', 'servo_command.py', port, command], 
                               capture_output=True, text=True)
        print(result.stdout)
        if result.stderr:
            print(f"Error: {result.stderr}")
            return False
        return True
    except Exception as e:
        print(f"Error sending command: {e}")
        return False

def move_to_position(serial_port, x, y, z, pitch, roll, use_visualizer=False, invert_servo3=False):
    """
    Move the robot arm to the specified position with specified orientation.
    
    Args:
        serial_port: Serial port object for communication with ESP32
        x, y, z: Target position coordinates
        pitch, roll: Target orientation angles
        use_visualizer: Whether to visualize the robot configuration
        invert_servo3: Whether to invert servo 3's movement
    
    Returns:
        A tuple containing the joint positions and dynamixel positions
    """
    # Initialize the robot kinematics handler
    kinematics = RobotArmIKPy()
    
    # Print the target position
    print(f"Target position: x={x}, y={y}, z={z}, pitch={pitch}, roll={roll}")
    
    # Get the joint angles using inverse kinematics
    # Create target orientation matrix from pitch and roll
    target_orientation = None
    
    # Get the joint angles using inverse kinematics
    joint_angles = kinematics.inverse_kinematics([x, y, z], target_orientation)
    
    # Print the joint angles
    print(f"Joint angles (degrees): {np.degrees(joint_angles)}")
    
    # Convert joint angles to dynamixel positions
    dxl_positions = kinematics.angles_to_dynamixel(joint_angles)
    
    # Print the dynamixel positions
    print(f"Dynamixel positions: {dxl_positions}")
    
    # Invert servo 3 if requested
    if invert_servo3 and len(dxl_positions) > 2:
        dxl_positions[2] = kinematics.invert_servo3_position(dxl_positions[2])
        print(f"Inverted Servo 3 position: {dxl_positions[2]}")
    
    # Send commands to move the servos
    for i, pos in enumerate(dxl_positions):
        servo_id = i + 1  # Servo IDs start from 1
        command = f"S{servo_id}P{int(pos)}"
        send_servo_command(serial_port, command)
        time.sleep(0.1)  # Small delay between commands
    
    # Visualize the robot configuration if requested
    if use_visualizer:
        kinematics.plot_arm(joint_angles, [x, y, z])
    
    return joint_angles, dxl_positions

def main():
    parser = argparse.ArgumentParser(description='Move robot arm to a position using inverse kinematics')
    parser.add_argument('port', help='Serial port')
    parser.add_argument('--x', type=float, default=0.2, help='X coordinate')
    parser.add_argument('--y', type=float, default=0.0, help='Y coordinate')
    parser.add_argument('--z', type=float, default=0.2, help='Z coordinate')
    parser.add_argument('--pitch', type=float, default=0.0, help='Pitch angle in degrees')
    parser.add_argument('--roll', type=float, default=0.0, help='Roll angle in degrees')
    parser.add_argument('--visualize', action='store_true', help='Visualize robot configuration')
    parser.add_argument('--invert-servo3', action='store_true', help='Invert servo 3 position relative to center')
    
    args = parser.parse_args()
    
    # Use the port directly for the send_servo_command function instead of creating a serial connection
    port = args.port
    
    try:
        print(f"Using port {port}")
        
        # Move to the specified position
        move_to_position(port, args.x, args.y, args.z, np.radians(args.pitch), np.radians(args.roll), 
                         args.visualize, args.invert_servo3)
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 