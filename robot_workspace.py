#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Arm Workspace Visualization
This script visualizes the workspace of the 6-axis robot arm
to help debug reachability issues and plan trajectories.
"""

import numpy as np
from math import pi, cos, sin
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ikpy.chain
from ikpy.link import OriginLink, URDFLink
import random
import time

# Import constants from the main script
from robot_ikpy_kinematics import (
    BASE_HEIGHT, LINK_1_2_LENGTH, LINK_2_3_LENGTH, LINK_3_4_LENGTH,
    LINK_4_5_LENGTH, LINK_5_6_LENGTH, END_EFFECTOR_LENGTH, RobotArmIKPy
)

def generate_random_joint_angles(joint_limits, num_samples=1):
    """Generate random joint angles within the specified limits."""
    angles_list = []
    for _ in range(num_samples):
        angles = []
        for limits in joint_limits:
            angle = random.uniform(limits[0], limits[1])
            angles.append(angle)
        angles_list.append(angles)
    return angles_list

def calculate_workspace_points(robot, num_samples=1000):
    """Calculate reachable points to visualize the robot's workspace."""
    joint_limits = robot.joint_limits
    
    # Generate random joint configurations
    angles_list = generate_random_joint_angles(joint_limits, num_samples)
    
    # Calculate end-effector positions for each configuration
    positions = []
    for angles in angles_list:
        try:
            position, _, _ = robot.forward_kinematics(angles)
            positions.append(position)
        except Exception as e:
            print(f"Error calculating forward kinematics: {e}")
    
    return np.array(positions)

def test_specific_positions(robot, positions):
    """Test if specific positions are reachable."""
    results = []
    
    for i, pos in enumerate(positions):
        print(f"\nTesting position {i+1}: {pos}")
        start_time = time.time()
        
        try:
            # Try to calculate inverse kinematics
            joint_angles = robot.inverse_kinematics(pos)
            
            # Calculate the actual position achieved
            actual_pos, _, _ = robot.forward_kinematics(joint_angles)
            
            # Calculate error
            error = np.linalg.norm(np.array(pos) - actual_pos)
            
            result = {
                'target': pos,
                'actual': actual_pos,
                'error': error,
                'joint_angles': joint_angles,
                'reachable': error < 0.01  # Consider it reachable if error is small
            }
            
            # Print result
            print(f"  Result: {'✓ Reachable' if result['reachable'] else '✗ Unreachable'}")
            print(f"  Error: {error:.4f}")
            print(f"  Joint angles: {[f'{a:.2f}' for a in joint_angles]}")
            print(f"  Time: {(time.time() - start_time)*1000:.2f} ms")
            
        except Exception as e:
            print(f"  Error: {e}")
            result = {
                'target': pos,
                'actual': None,
                'error': None,
                'joint_angles': None,
                'reachable': False
            }
        
        results.append(result)
    
    return results

def plot_workspace(positions, test_positions=None, test_results=None):
    """Visualize the robot's workspace in 3D."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the workspace points
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
              c='blue', marker='.', alpha=0.1, label='Workspace')
    
    # Plot test positions if provided
    if test_positions is not None and test_results is not None:
        for i, pos in enumerate(test_positions):
            if test_results[i]['reachable']:
                ax.scatter(pos[0], pos[1], pos[2], c='green', marker='o', s=100, 
                          label='Reachable' if i == 0 else None)
            else:
                ax.scatter(pos[0], pos[1], pos[2], c='red', marker='x', s=100,
                          label='Unreachable' if i == 0 else None)
    
    # Plot robot base
    ax.scatter(0, 0, 0, c='black', marker='s', s=100, label='Robot Base')
    
    # Set labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Robot Arm Workspace')
    
    # Add legend
    handles, labels = ax.get_legend_handles_labels()
    unique_labels = []
    unique_handles = []
    for handle, label in zip(handles, labels):
        if label not in unique_labels:
            unique_labels.append(label)
            unique_handles.append(handle)
    ax.legend(unique_handles, unique_labels, loc='best')
    
    # Equal aspect ratio
    max_range = max(
        np.max(positions[:, 0]) - np.min(positions[:, 0]),
        np.max(positions[:, 1]) - np.min(positions[:, 1]),
        np.max(positions[:, 2]) - np.min(positions[:, 2])
    )
    mid_x = (np.max(positions[:, 0]) + np.min(positions[:, 0])) / 2
    mid_y = (np.max(positions[:, 1]) + np.min(positions[:, 1])) / 2
    mid_z = (np.max(positions[:, 2]) + np.min(positions[:, 2])) / 2
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
    
    # Add grid
    ax.grid(True)
    
    return fig, ax

def visualize_arm_reach():
    """Visualize the robot arm's workspace and test specific positions."""
    print("Initializing robot model...")
    robot = RobotArmIKPy()
    
    # Calculate workspace
    print("Generating workspace visualization (this may take a moment)...")
    workspace_points = calculate_workspace_points(robot, num_samples=2000)
    
    # Define test positions
    test_positions = [
        [0.20, 0.0, 0.15],  # Should be reachable
        [0.25, 0.0, 0.05],  # The position that was failing
        [0.35, 0.0, 0.15],  # Might be at the edge of workspace
        [0.10, 0.20, 0.10], # Off to the side
        [0.15, -0.15, 0.20]  # Another position to test
    ]
    
    # Test specific positions
    print("\nTesting specific positions for reachability:")
    test_results = test_specific_positions(robot, test_positions)
    
    # Visualize the workspace
    print("\nCreating workspace visualization...")
    fig, ax = plot_workspace(workspace_points, test_positions, test_results)
    
    # Display statistics
    total_points = len(workspace_points)
    x_min, x_max = np.min(workspace_points[:, 0]), np.max(workspace_points[:, 0])
    y_min, y_max = np.min(workspace_points[:, 1]), np.max(workspace_points[:, 1])
    z_min, z_max = np.min(workspace_points[:, 2]), np.max(workspace_points[:, 2])
    
    print("\nWorkspace Statistics:")
    print(f"Total points sampled: {total_points}")
    print(f"X range: {x_min:.2f} to {x_max:.2f} meters")
    print(f"Y range: {y_min:.2f} to {y_max:.2f} meters")
    print(f"Z range: {z_min:.2f} to {z_max:.2f} meters")
    print(f"Maximum reach: {max(np.linalg.norm(workspace_points, axis=1)):.2f} meters")
    
    # Show the plot
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    visualize_arm_reach() 