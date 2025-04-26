#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to move the robot arm to a specific position using inverse kinematics
"""

import argparse
import numpy as np
import time
import subprocess
import sys
from math import pi, degrees

# Import the robot kinematics class
from robot_ikpy_kinematics import RobotArmIKPy

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

def move_to_position(port, x, y, z, visualize=False, invert_x=False, invert_y=False, invert_z=False,
                     offset_j1=0, offset_j2=0, offset_j3=0, offset_j4=0, offset_j5=0, offset_j6=0):
    """Calculate inverse kinematics and move the robot to the specified position"""
    # Create the robot arm model
    robot = RobotArmIKPy()
    
    # Apply direction inversion if needed
    if invert_x:
        x = -x
    if invert_y:
        y = -y
    if invert_z:
        z = -z
    
    # Calculate inverse kinematics for the target position
    target_position = [x, y, z]
    print(f"Target position: X={x}, Y={y}, Z={z}")
    
    try:
        # Calculate joint angles
        joint_angles = robot.inverse_kinematics(target_position)
        
        # Convert to Dynamixel positions
        dynamixel_positions = robot.angles_to_dynamixel(joint_angles)
        
        # Apply offsets to calibrate servos
        dynamixel_positions[0] += offset_j1
        dynamixel_positions[1] += offset_j2
        dynamixel_positions[2] += offset_j3
        dynamixel_positions[3] += offset_j4
        dynamixel_positions[4] += offset_j5
        dynamixel_positions[5] += offset_j6
        
        # Ensure positions are within valid ranges
        for i in range(len(dynamixel_positions)):
            # Default limits for Dynamixel servos
            min_pos = 1024
            max_pos = 3072
            
            # Apply specific limits from robot model if available
            if hasattr(robot, 'joint_dxl_limits') and i < len(robot.joint_dxl_limits):
                min_pos = robot.joint_dxl_limits[i][0]
                max_pos = robot.joint_dxl_limits[i][1]
            
            dynamixel_positions[i] = max(min_pos, min(max_pos, dynamixel_positions[i]))
        
        # Print joint angles and positions
        print("\nCalculated joint angles:")
        for i, angle in enumerate(joint_angles):
            print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f}°)")
        
        print("\nDynamixel position values with offsets:")
        for i, pos in enumerate(dynamixel_positions):
            print(f"Joint {i+1}: {pos}")
        
        # Verify the solution with forward kinematics
        actual_position, _, _ = robot.forward_kinematics(joint_angles)
        print("\nActual position that will be reached (before calibration):")
        print(f"X: {actual_position[0]:.4f} m")
        print(f"Y: {actual_position[1]:.4f} m")
        print(f"Z: {actual_position[2]:.4f} m")
        
        # Calculate error
        error = np.linalg.norm(np.array(target_position) - actual_position)
        print(f"Position error: {error:.4f} m")
        
        # If visualization is requested, show the arm configuration
        if visualize:
            robot.plot_arm(joint_angles, target_position)
            import matplotlib.pyplot as plt
            plt.show()
        
        # Check with user before moving
        response = input("\nMove robot to this position? (y/n): ")
        if response.lower() != 'y':
            print("Operation cancelled")
            return False
        
        # Send the move command to the ESP32
        command = f"M{dynamixel_positions[0]},{dynamixel_positions[1]},{dynamixel_positions[2]},{dynamixel_positions[3]},{dynamixel_positions[4]},{dynamixel_positions[5]}"
        result = send_servo_command(port, command)
        
        if result:
            print("Robot moved to position successfully.")
            return True
        else:
            print("Failed to move robot to position.")
            return False
        
    except Exception as e:
        print(f"Error in inverse kinematics calculation: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    parser = argparse.ArgumentParser(description='Move robot arm to a specific position')
    parser.add_argument('port', type=str, help='Serial port for ESP32 (e.g., /dev/cu.usbserial-59100209741)')
    parser.add_argument('--x', type=float, default=0.15, help='X coordinate in meters (default: 0.15)')
    parser.add_argument('--y', type=float, default=0.0, help='Y coordinate in meters (default: 0.0)')
    parser.add_argument('--z', type=float, default=0.2, help='Z coordinate in meters (default: 0.2)')
    parser.add_argument('--visualize', action='store_true', help='Visualize the robot arm configuration')
    
    # Add calibration options
    parser.add_argument('--invert-x', action='store_true', help='Invert the X direction')
    parser.add_argument('--invert-y', action='store_true', help='Invert the Y direction')
    parser.add_argument('--invert-z', action='store_true', help='Invert the Z direction')
    parser.add_argument('--offset-j1', type=int, default=0, help='Offset for joint 1 position')
    parser.add_argument('--offset-j2', type=int, default=0, help='Offset for joint 2 position')
    parser.add_argument('--offset-j3', type=int, default=0, help='Offset for joint 3 position')
    parser.add_argument('--offset-j4', type=int, default=0, help='Offset for joint 4 position')
    parser.add_argument('--offset-j5', type=int, default=0, help='Offset for joint 5 position')
    parser.add_argument('--offset-j6', type=int, default=0, help='Offset for joint 6 position')
    
    args = parser.parse_args()
    
    # First center all servos to ensure a safe starting position
    print("Centering all servos...")
    if not send_servo_command(args.port, "C"):
        print("Failed to center servos. Exiting.")
        return
    
    # Wait for servos to reach center position
    time.sleep(1)
    
    # Move to the specified position with calibration
    move_to_position(
        args.port, args.x, args.y, args.z, args.visualize,
        args.invert_x, args.invert_y, args.invert_z,
        args.offset_j1, args.offset_j2, args.offset_j3,
        args.offset_j4, args.offset_j5, args.offset_j6
    )

if __name__ == "__main__":
    main() 