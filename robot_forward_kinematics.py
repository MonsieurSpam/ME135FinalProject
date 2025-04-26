#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Forward Kinematics Calculator for 6-Axis Robot Arm
- First 3 joints: Dynamixel XL430-W250 motors
- Last 3 joints: Dynamixel XL330 motors

This script calculates the end-effector position given joint angles.
"""

import numpy as np
from math import pi, sin, cos
import argparse

class RobotArm:
    def __init__(self):
        # Define DH parameters [a, alpha, d, theta]
        # These values need to be adjusted based on your specific robot dimensions
        self.dh_params = [
            [0, pi/2, 0.1, 0],       # Joint 1 (base)
            [0.13, 0, 0, 0],         # Joint 2 (shoulder)
            [0.12, 0, 0, 0],         # Joint 3 (elbow)
            [0, pi/2, 0, 0],         # Joint 4 (wrist 1)
            [0, -pi/2, 0.1, 0],      # Joint 5 (wrist 2)
            [0, 0, 0.05, 0]          # Joint 6 (wrist 3)
        ]
        
        # Dynamixel position conversion factors
        # XL430 has a range of 0-4095 for 0-360 degrees
        self.xl430_factor = 4095 / (2 * pi)
        # XL330 has a range of 0-4095 for 0-360 degrees
        self.xl330_factor = 4095 / (2 * pi)
        
        # Center position (2048 in Dynamixel units)
        self.center_position = 2048

    def transform_matrix(self, a, alpha, d, theta):
        """Calculate the homogeneous transformation matrix based on DH parameters."""
        ct = cos(theta)
        st = sin(theta)
        ca = cos(alpha)
        sa = sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles):
        """Calculate the end-effector position given joint angles."""
        T = np.eye(4)
        
        for i in range(6):
            # Update the theta parameter with the joint angle
            a, alpha, d, _ = self.dh_params[i]
            theta = joint_angles[i]
            
            # Calculate the transformation matrix for this joint
            Ti = self.transform_matrix(a, alpha, d, theta)
            
            # Update the cumulative transformation
            T = T @ Ti
        
        # Extract position from transformation matrix
        position = T[0:3, 3]
        
        # Extract rotation matrix
        rotation = T[0:3, 0:3]
        
        return position, rotation, T

    def angles_to_dynamixel(self, joint_angles):
        """Convert joint angles in radians to Dynamixel position values."""
        dynamixel_positions = []
        
        for i, angle in enumerate(joint_angles):
            # Convert to Dynamixel units
            if i < 3:  # XL430 motors
                position = self.center_position + int(angle * self.xl430_factor)
            else:      # XL330 motors
                position = self.center_position + int(angle * self.xl330_factor)
            
            # Ensure the position is within valid range (0-4095)
            position = max(0, min(4095, position))
            
            dynamixel_positions.append(position)
        
        return dynamixel_positions

    def dynamixel_to_angles(self, dynamixel_positions):
        """Convert Dynamixel position values to joint angles in radians."""
        joint_angles = []
        
        for i, position in enumerate(dynamixel_positions):
            # Calculate angle
            if i < 3:  # XL430 motors
                angle = (position - self.center_position) / self.xl430_factor
            else:      # XL330 motors
                angle = (position - self.center_position) / self.xl330_factor
            
            joint_angles.append(angle)
        
        return joint_angles

def main():
    parser = argparse.ArgumentParser(description='Calculate forward kinematics for 6-axis robot arm')
    parser.add_argument('--angles', type=float, nargs=6, required=True, 
                        help='Six joint angles in radians')
    parser.add_argument('--dynamixel', action='store_true', 
                        help='Input angles are in Dynamixel units instead of radians')
    parser.add_argument('--visualize', action='store_true', 
                        help='Visualize the robot arm (requires matplotlib)')
    
    args = parser.parse_args()
    
    # Create robot arm instance
    robot = RobotArm()
    
    # Get joint angles
    if args.dynamixel:
        dynamixel_positions = args.angles
        joint_angles = robot.dynamixel_to_angles(dynamixel_positions)
        print("\nDynamixel position values:")
        for i, position in enumerate(dynamixel_positions):
            print(f"Joint {i+1}: {position}")
    else:
        joint_angles = args.angles
        dynamixel_positions = robot.angles_to_dynamixel(joint_angles)
    
    # Print joint angles
    print("\nJoint angles (radians):")
    for i, angle in enumerate(joint_angles):
        print(f"Joint {i+1}: {angle:.4f} rad ({angle * 180/pi:.2f} degrees)")
    
    # If not provided as input, print Dynamixel positions
    if not args.dynamixel:
        print("\nDynamixel position values:")
        for i, position in enumerate(dynamixel_positions):
            print(f"Joint {i+1}: {position}")
    
    # Calculate forward kinematics
    position, rotation, T = robot.forward_kinematics(joint_angles)
    
    # Print end-effector position
    print("\nEnd-effector position:")
    print(f"X: {position[0]:.4f}")
    print(f"Y: {position[1]:.4f}")
    print(f"Z: {position[2]:.4f}")
    
    # Print rotation matrix
    print("\nEnd-effector orientation (rotation matrix):")
    for i in range(3):
        print(f"[{rotation[i,0]:.4f}, {rotation[i,1]:.4f}, {rotation[i,2]:.4f}]")
    
    # Calculate joint positions for visualization
    joint_positions = []
    T_cumulative = np.eye(4)
    joint_positions.append(T_cumulative[0:3, 3].tolist())  # Base position
    
    for i in range(6):
        a, alpha, d, _ = robot.dh_params[i]
        theta = joint_angles[i]
        
        Ti = robot.transform_matrix(a, alpha, d, theta)
        T_cumulative = T_cumulative @ Ti
        
        joint_positions.append(T_cumulative[0:3, 3].tolist())
    
    # Print all joint positions
    print("\nJoint positions:")
    for i, pos in enumerate(joint_positions):
        print(f"Joint {i}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
    
    # Visualize if requested
    if args.visualize:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            
            # Convert to separate x, y, z lists
            x_vals = [pos[0] for pos in joint_positions]
            y_vals = [pos[1] for pos in joint_positions]
            z_vals = [pos[2] for pos in joint_positions]
            
            # Create 3D plot
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # Plot robot arm links
            ax.plot(x_vals, y_vals, z_vals, 'bo-', linewidth=2, markersize=8)
            
            # Plot end-effector
            ax.scatter(position[0], position[1], position[2], 
                      color='red', s=100, label='End-effector')
            
            # Set labels and title
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('6-Axis Robot Arm')
            
            # Set equal aspect ratio
            max_range = max(
                max(x_vals) - min(x_vals),
                max(y_vals) - min(y_vals),
                max(z_vals) - min(z_vals)
            )
            mid_x = (max(x_vals) + min(x_vals)) / 2
            mid_y = (max(y_vals) + min(y_vals)) / 2
            mid_z = (max(z_vals) + min(z_vals)) / 2
            ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
            ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
            ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
            
            # Add coordinate frame at end-effector
            arrow_length = max_range / 10
            
            # X-axis (red)
            ax.quiver(position[0], position[1], position[2],
                     arrow_length * rotation[0, 0], arrow_length * rotation[1, 0], arrow_length * rotation[2, 0],
                     color='r', arrow_length_ratio=0.1)
            
            # Y-axis (green)
            ax.quiver(position[0], position[1], position[2],
                     arrow_length * rotation[0, 1], arrow_length * rotation[1, 1], arrow_length * rotation[2, 1],
                     color='g', arrow_length_ratio=0.1)
            
            # Z-axis (blue)
            ax.quiver(position[0], position[1], position[2],
                     arrow_length * rotation[0, 2], arrow_length * rotation[1, 2], arrow_length * rotation[2, 2],
                     color='b', arrow_length_ratio=0.1)
            
            plt.legend()
            plt.tight_layout()
            plt.show()
            
        except ImportError:
            print("\nVisualization requires matplotlib. Install with: pip install matplotlib")

if __name__ == "__main__":
    main() 