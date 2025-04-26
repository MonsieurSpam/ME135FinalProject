#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Inverse Kinematics Calculator for 6-Axis Robot Arm
- First 3 joints: Dynamixel XL430-W250 motors
- Last 3 joints: Dynamixel XL330 motors

This script calculates the joint angles required to reach a target position.
"""

import numpy as np
from math import atan2, acos, sqrt, pi, sin, cos
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
        
        # Joint limits in radians [min, max]
        self.joint_limits = [
            [-pi, pi],       # Joint 1
            [-pi/2, pi/2],   # Joint 2
            [-pi/2, pi/2],   # Joint 3
            [-pi, pi],       # Joint 4
            [-pi/2, pi/2],   # Joint 5
            [-pi, pi]        # Joint 6
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
        
        return position, T

    def inverse_kinematics(self, target_position, target_orientation=None):
        """
        Calculate joint angles to reach the target position.
        Uses a numerical approach with the Jacobian pseudo-inverse.
        
        Args:
            target_position: [x, y, z] coordinates
            target_orientation: Optional orientation matrix or Euler angles
            
        Returns:
            joint_angles: List of 6 joint angles in radians
        """
        # Initial guess (neutral position)
        joint_angles = [0, 0, 0, 0, 0, 0]
        
        # Convert target position to numpy array
        target = np.array(target_position)
        
        # Maximum iterations and convergence threshold
        max_iterations = 100
        threshold = 0.001
        
        for iteration in range(max_iterations):
            # Calculate current position with forward kinematics
            current_position, _ = self.forward_kinematics(joint_angles)
            
            # Calculate error
            error = target - current_position
            error_magnitude = np.linalg.norm(error)
            
            # Check if we've reached the target
            if error_magnitude < threshold:
                break
                
            # Calculate the Jacobian matrix
            J = self.calculate_jacobian(joint_angles)
            
            # Calculate the pseudo-inverse of the Jacobian
            J_inv = np.linalg.pinv(J)
            
            # Calculate joint angle adjustments
            delta_theta = J_inv @ error
            
            # Update joint angles
            joint_angles = joint_angles + 0.5 * delta_theta  # Damping factor of 0.5
            
            # Apply joint limits
            for i in range(6):
                joint_angles[i] = max(self.joint_limits[i][0], min(self.joint_limits[i][1], joint_angles[i]))
        
        return joint_angles

    def calculate_jacobian(self, joint_angles):
        """Calculate the Jacobian matrix for the current joint configuration."""
        delta = 0.0001
        J = np.zeros((3, 6))
        
        # For each joint
        for i in range(6):
            # Make a small change to the joint angle
            joint_angles_plus = joint_angles.copy()
            joint_angles_plus[i] += delta
            
            # Calculate positions
            pos1, _ = self.forward_kinematics(joint_angles)
            pos2, _ = self.forward_kinematics(joint_angles_plus)
            
            # Calculate partial derivative
            J[:, i] = (pos2 - pos1) / delta
        
        return J

    def angles_to_dynamixel(self, joint_angles):
        """Convert joint angles in radians to Dynamixel position values."""
        dynamixel_positions = []
        
        for i, angle in enumerate(joint_angles):
            # Center the angle around 0
            centered_angle = angle
            
            # Convert to Dynamixel units
            if i < 3:  # XL430 motors
                position = self.center_position + int(centered_angle * self.xl430_factor)
            else:      # XL330 motors
                position = self.center_position + int(centered_angle * self.xl330_factor)
            
            # Ensure the position is within valid range (0-4095)
            position = max(0, min(4095, position))
            
            dynamixel_positions.append(position)
        
        return dynamixel_positions

    def dynamixel_to_angles(self, dynamixel_positions):
        """Convert Dynamixel position values to joint angles in radians."""
        joint_angles = []
        
        for i, position in enumerate(dynamixel_positions):
            # Calculate centered angle
            if i < 3:  # XL430 motors
                angle = (position - self.center_position) / self.xl430_factor
            else:      # XL330 motors
                angle = (position - self.center_position) / self.xl330_factor
            
            joint_angles.append(angle)
        
        return joint_angles

def main():
    parser = argparse.ArgumentParser(description='Calculate inverse kinematics for 6-axis robot arm')
    parser.add_argument('--x', type=float, required=True, help='Target X coordinate')
    parser.add_argument('--y', type=float, required=True, help='Target Y coordinate')
    parser.add_argument('--z', type=float, required=True, help='Target Z coordinate')
    parser.add_argument('--visualize', action='store_true', help='Visualize the robot arm (requires matplotlib)')
    
    args = parser.parse_args()
    
    # Create robot arm instance
    robot = RobotArm()
    
    # Target position
    target_position = [args.x, args.y, args.z]
    
    print(f"Target position: {target_position}")
    
    # Calculate inverse kinematics
    joint_angles = robot.inverse_kinematics(target_position)
    
    # Convert to Dynamixel position values
    dynamixel_positions = robot.angles_to_dynamixel(joint_angles)
    
    # Print results
    print("\nJoint angles (radians):")
    for i, angle in enumerate(joint_angles):
        print(f"Joint {i+1}: {angle:.4f} rad ({angle * 180/pi:.2f} degrees)")
    
    print("\nDynamixel position values:")
    for i, position in enumerate(dynamixel_positions):
        print(f"Joint {i+1}: {position}")
    
    # Calculate the actual position achieved with these joint angles
    actual_position, _ = robot.forward_kinematics(joint_angles)
    
    print("\nActual position achieved:")
    print(f"X: {actual_position[0]:.4f}")
    print(f"Y: {actual_position[1]:.4f}")
    print(f"Z: {actual_position[2]:.4f}")
    
    error = np.linalg.norm(np.array(target_position) - actual_position)
    print(f"\nPosition error: {error:.4f}")
    
    # Visualize if requested
    if args.visualize:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            
            # Calculate joint positions
            T = np.eye(4)
            joint_positions = [[0, 0, 0]]  # Start at origin
            
            for i in range(6):
                a, alpha, d, _ = robot.dh_params[i]
                theta = joint_angles[i]
                
                Ti = robot.transform_matrix(a, alpha, d, theta)
                T = T @ Ti
                
                joint_positions.append(T[0:3, 3].tolist())
            
            # Convert to separate x, y, z lists
            x_vals = [pos[0] for pos in joint_positions]
            y_vals = [pos[1] for pos in joint_positions]
            z_vals = [pos[2] for pos in joint_positions]
            
            # Create 3D plot
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # Plot robot arm links
            ax.plot(x_vals, y_vals, z_vals, 'bo-', linewidth=2, markersize=8)
            
            # Plot target position
            ax.scatter(target_position[0], target_position[1], target_position[2], 
                      color='red', s=100, label='Target')
            
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
            
            plt.legend()
            plt.tight_layout()
            plt.show()
            
        except ImportError:
            print("\nVisualization requires matplotlib. Install with: pip install matplotlib")

if __name__ == "__main__":
    main() 