#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SO100 Arm IKPy Model
This script creates an IKPy model for the SO100 robotic arm.
"""

import numpy as np
from math import pi
import ikpy
from ikpy.chain import Chain
from ikpy.link import URDFLink, OriginLink
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class SO100Arm:
    def __init__(self):
        # Define the kinematic chain for the SO100 arm
        self.chain = Chain(
            name="so100_arm",
            links=[
                OriginLink(),  # Base link
                URDFLink(
                    name="base_rotation",  # Joint 1 - Base rotation
                    origin_translation=[0, 0, 0.04],  # Base height
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],  # Z-axis rotation
                    joint_type="revolute",
                    bounds=(-pi/2, pi/2)  # ±90 degrees
                ),
                URDFLink(
                    name="shoulder",  # Joint 2 - Shoulder
                    origin_translation=[0, 0, 0.02],  # Link length
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # Y-axis rotation
                    joint_type="revolute",
                    bounds=(-pi/2, pi/4)  # -90 to +45 degrees
                ),
                URDFLink(
                    name="elbow",  # Joint 3 - Elbow
                    origin_translation=[0.1, 0, 0],  # Upper arm length (reduced from 0.12)
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # Y-axis rotation
                    joint_type="revolute",
                    bounds=(0, pi)  # 0 to 180 degrees
                ),
                URDFLink(
                    name="wrist_pitch",  # Joint 4 - Wrist pitch
                    origin_translation=[0.1, 0, 0],  # Forearm length (reduced from 0.14)
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # Y-axis rotation
                    joint_type="revolute",
                    bounds=(0, pi)  # 0 to 180 degrees
                ),
                URDFLink(
                    name="wrist_roll",  # Joint 5 - Wrist roll (orientation only)
                    origin_translation=[0, 0, 0],  # No translation
                    origin_orientation=[0, 0, 0],
                    rotation=[1, 0, 0],  # X-axis rotation
                    joint_type="revolute",
                    bounds=(-pi/2, pi/2)  # ±90 degrees
                ),
                URDFLink(
                    name="gripper",  # Fixed link for gripper
                    origin_translation=[0, 0, 0.05],  # End effector length
                    origin_orientation=[0, 0, 0],
                    joint_type="fixed"
                )
            ]
        )

    def forward_kinematics(self, joint_angles):
        """Calculate the end-effector position given joint angles."""
        # Add 0 for the base link
        full_angles = [0] + list(joint_angles)
        # Ensure we have the right number of angles
        if len(full_angles) < len(self.chain.links):
            full_angles.extend([0] * (len(self.chain.links) - len(full_angles)))
        elif len(full_angles) > len(self.chain.links):
            full_angles = full_angles[:len(self.chain.links)]
        
        try:
            # Calculate the transformation matrix
            transformation_matrix = self.chain.forward_kinematics(full_angles)
            # Extract position and rotation
            position = transformation_matrix[0:3, 3]
            rotation = transformation_matrix[0:3, 0:3]
            return position, rotation, transformation_matrix
        except Exception as e:
            print(f"Error in forward kinematics calculation: {e}")
            return np.array([0, 0, 0]), np.eye(3), np.eye(4)

    def inverse_kinematics(self, target_position, target_orientation=None, initial_position=None, max_retries=10):
        """Calculate joint angles to reach the target position with error < 0.01m."""
        if initial_position is None:
            initial_position = [0, 0, 0, 0, 0]
        
        best_error = float('inf')
        best_angles = None
        
        # Try different initial positions
        for attempt in range(max_retries):
            # Add 0 for the base link
            initial_full = [0] + list(initial_position)
            
            # Ensure we have the right number of angles
            if len(initial_full) < len(self.chain.links):
                initial_full.extend([0] * (len(self.chain.links) - len(initial_full)))
            elif len(initial_full) > len(self.chain.links):
                initial_full = initial_full[:len(self.chain.links)]
            
            try:
                if target_orientation is not None:
                    # Create the full 4x4 transformation matrix
                    target_matrix = np.eye(4)
                    target_matrix[0:3, 0:3] = target_orientation
                    target_matrix[0:3, 3] = target_position
                    # With orientation constraint
                    joint_angles = self.chain.inverse_kinematics(
                        target_matrix,
                        initial_position=initial_full,
                        orientation_mode="all"
                    )
                else:
                    # Position only - ignore wrist roll for position-only IK
                    # First solve for position with wrist roll at 0
                    initial_full[5] = 0  # Set wrist roll to 0
                    joint_angles = self.chain.inverse_kinematics(
                        target_position,
                        initial_position=initial_full,
                        orientation_mode=None
                    )
                    # Keep the wrist roll at 0 since it doesn't affect position
                    joint_angles[4] = 0
                
                # Verify the solution
                achieved_position, _, _ = self.forward_kinematics(joint_angles[1:6])
                error = np.linalg.norm(np.array(target_position) - achieved_position)
                
                if error < best_error:
                    best_error = error
                    best_angles = joint_angles[1:6]
                    print(f"New best solution found with error: {error:.3f}m")
                
                # If we found a good solution, return it
                if error < 0.01:  # 1 cm tolerance
                    return joint_angles[1:6]
                
                # Try a different initial position for next attempt
                # Use joint-specific bounds for random initialization
                initial_position = [
                    np.random.uniform(-pi/2, pi/2),  # base_rotation
                    np.random.uniform(-pi/2, pi/4),  # shoulder
                    np.random.uniform(0, pi),        # elbow
                    np.random.uniform(0, pi),        # wrist_pitch
                    0                               # wrist_roll (kept at 0)
                ]
                
            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {e}")
                continue
        
        if best_error < float('inf'):
            print(f"Warning: Best solution found has error of {best_error:.3f}m")
            return best_angles
        else:
            print("Error: Could not find a solution within tolerance")
            return [0, 0, 0, 0, 0]

    def visualize(self, joint_angles, target_position=None):
        """Visualize the robot arm configuration in 3D."""
        # Create figure and 3D axis
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Add 0 for the base link and ensure we have the right number of angles
        full_angles = [0] + list(joint_angles)
        if len(full_angles) < len(self.chain.links):
            full_angles.extend([0] * (len(self.chain.links) - len(full_angles)))
        elif len(full_angles) > len(self.chain.links):
            full_angles = full_angles[:len(self.chain.links)]
        
        # Get the transformation matrices for each link
        transformations = self.chain.forward_kinematics(full_angles, full_kinematics=True)
        
        # Plot each link
        points = []
        for transform in transformations:
            points.append(transform[0:3, 3])
        points = np.array(points)
        
        # Plot the links
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 'b-', linewidth=2, label='Arm')
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o')
        
        # Plot target position if provided
        if target_position is not None:
            ax.scatter(target_position[0], target_position[1], target_position[2], 
                     color='red', s=100, label='Target')
            ax.legend()
        
        # Set labels and title
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('SO100 Arm Configuration\n(Click and drag to rotate)')
        
        # Set equal aspect ratio
        max_range = np.array([
            points[:, 0].max() - points[:, 0].min(),
            points[:, 1].max() - points[:, 1].min(),
            points[:, 2].max() - points[:, 2].min()
        ]).max() / 2.0
        
        mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
        mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
        mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # Add grid
        ax.grid(True)
        
        # Set initial viewpoint
        ax.view_init(elev=30, azim=45)
        
        # Enable interactive mode
        plt.ion()
        
        return fig, ax

def main():
    # Example usage
    arm = SO100Arm()
    
    # Test forward kinematics
    test_angles = [0, 0, 0, 0, 0]  # All joints at 0 degrees
    position, rotation, _ = arm.forward_kinematics(test_angles)
    print("Forward Kinematics Test:")
    print(f"Position: {position}")
    print(f"Rotation:\n{rotation}")
    
    # Test a single target position
    target_position = [0.15, 0.15, 0.0]  # Low and directly in front
    
    print(f"\nTesting Target Position: {target_position}")
    
    # Calculate inverse kinematics 
    joint_angles = arm.inverse_kinematics(target_position)
    print(f"Joint Angles (radians): {joint_angles}")
    print(f"Joint Angles (degrees): {np.degrees(joint_angles)}")
    
    # Verify the solution
    achieved_position, _, _ = arm.forward_kinematics(joint_angles)
    print(f"Achieved Position: {achieved_position}")
    error = np.linalg.norm(np.array(target_position) - achieved_position)
    print(f"Position Error (meters): {error:.6f}")
    
    # Visualize the solution
    fig, ax = arm.visualize(joint_angles, target_position)
    
    # Keep the plot window open and interactive
    plt.show(block=True)
    
    # Print instructions for the user
    print("\nVisualization Controls:")
    print("- Click and drag to rotate the view")
    print("- Use the mouse wheel to zoom in/out")
    print("- Press 'q' to close the window")

if __name__ == "__main__":
    main() 