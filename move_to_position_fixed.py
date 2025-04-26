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
                     wrist_pitch=0, wrist_rotation=0, gripper_close=0, invert_servo3=False):
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
        
        # Apply inversion for Servo 3 if requested
        if invert_servo3:
            # Calculate the center position
            center = (robot.joint_dxl_limits[2][0] + robot.joint_dxl_limits[2][1]) / 2
            # Invert the position relative to center
            dynamixel_positions[2] = int(center * 2 - dynamixel_positions[2])
            print(f"Servo 3 inverted from original position")
        
        # Apply additional adjustments based on clarified servo roles
        # Servo 4: Controls wrist pitch (up/down)
        # Servo 5: Controls wrist rotation
        # Servo 6: Controls gripper opening/closing
        dynamixel_positions[3] += wrist_pitch       # Servo 4: Wrist pitch
        dynamixel_positions[4] += wrist_rotation    # Servo 5: Wrist rotation
        dynamixel_positions[5] += gripper_close     # Servo 6: Gripper
        
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
        
        print("\nDynamixel position values with adjustments:")
        for i, pos in enumerate(dynamixel_positions):
            print(f"Joint {i+1}: {pos}")
        
        # Verify the solution with forward kinematics
        actual_position, _, _ = robot.forward_kinematics(joint_angles)
        print("\nActual position that will be reached (before adjustments):")
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
    parser = argparse.ArgumentParser(description='Move robot arm to a specific position with corrected mappings')
    parser.add_argument('port', type=str, help='Serial port for ESP32 (e.g., /dev/cu.usbserial-59100209741)')
    parser.add_argument('--x', type=float, default=0.15, help='X coordinate in meters (default: 0.15)')
    parser.add_argument('--y', type=float, default=0.0, help='Y coordinate in meters (default: 0.0)')
    parser.add_argument('--z', type=float, default=0.2, help='Z coordinate in meters (default: 0.2)')
    parser.add_argument('--visualize', action='store_true', help='Visualize the robot arm configuration')
    
    # Add calibration options
    parser.add_argument('--invert-x', action='store_true', help='Invert the X direction')
    parser.add_argument('--invert-y', action='store_true', help='Invert the Y direction')
    parser.add_argument('--invert-z', action='store_true', help='Invert the Z direction')
    parser.add_argument('--invert-servo3', action='store_true', help='Invert the direction of Servo 3 (elbow)')
    
    # New options with corrected mapping
    parser.add_argument('--wrist-pitch', type=int, default=0, help='Adjustment for wrist pitch (servo 4)')
    parser.add_argument('--wrist-rotation', type=int, default=0, help='Adjustment for wrist rotation (servo 5)')
    parser.add_argument('--gripper-close', type=int, default=0, help='Adjustment for gripper (servo 6, positive=close)')
    
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
        args.wrist_pitch, args.wrist_rotation, args.gripper_close,
        args.invert_servo3
    )

if __name__ == "__main__":
    main() 