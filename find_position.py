#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Find end effector position from Dynamixel positions
This script converts Dynamixel positions to IKPy angles and calculates the end effector position.
"""

import numpy as np
from so100_ikpy_model import SO100Arm
from math import radians, degrees
import matplotlib.pyplot as plt

def dynamixel_to_ik_angles(dxl_positions):
    """Convert Dynamixel positions to IKPy angles."""
    # Dynamixel position to angle conversion
    # 0 -> 0°
    # 2048 -> 180°
    # 4095 -> 360°
    angles_deg = [pos * (360.0 / 4095.0) for pos in dxl_positions]
    
    # Convert to IKPy coordinate system
    ik_angles = []
    for i, angle_deg in enumerate(angles_deg):
        if i == 0:  # Base
            angle_rad = radians(angle_deg - 180)  # Shift to match IKPy range
        elif i == 1:  # Shoulder
            angle_rad = radians(angle_deg - 270)  # Shift to match IKPy range
        elif i == 2:  # Elbow
            angle_rad = radians(angle_deg - 90)  # Shift to match IKPy range
        elif i == 3:  # Wrist rotation
            angle_rad = radians(angle_deg - 90)  # Shift to match IKPy range
        elif i == 4:  # Wrist pitch
            angle_rad = radians(angle_deg)  # Already in correct range
        elif i == 5:  # Gripper
            angle_rad = radians(angle_deg)  # Already in correct range
        ik_angles.append(angle_rad)
    return ik_angles

def ik_angles_to_dynamixel(ik_angles):
    """Convert IKPy angles to Dynamixel positions."""
    # Convert IKPy angles to degrees
    angles_deg = [degrees(angle) for angle in ik_angles]
    
    # Convert to Dynamixel coordinate system
    dxl_positions = []
    for i, angle_deg in enumerate(angles_deg):
        if i == 0:  # Base
            dxl_pos = (angle_deg + 180) * (4095.0 / 360.0)  # Shift to match Dynamixel range
        elif i == 1:  # Shoulder
            dxl_pos = (angle_deg + 270) * (4095.0 / 360.0)  # Shift to match Dynamixel range
        elif i == 2:  # Elbow
            dxl_pos = (angle_deg + 90) * (4095.0 / 360.0)  # Shift to match Dynamixel range
        elif i == 3:  # Wrist rotation
            dxl_pos = (angle_deg + 90) * (4095.0 / 360.0)  # Shift to match Dynamixel range
        elif i == 4:  # Wrist pitch
            dxl_pos = angle_deg * (4095.0 / 360.0)  # Already in correct range
        elif i == 5:  # Gripper
            dxl_pos = angle_deg * (4095.0 / 360.0)  # Already in correct range
        
        # Ensure position is within 0-4095 range
        while dxl_pos < 0:
            dxl_pos += 4095
        while dxl_pos >= 4095:
            dxl_pos -= 4095
        dxl_positions.append(int(dxl_pos))
    return dxl_positions

def main():
    # Initialize the arm model
    arm = SO100Arm()
    
    # Dynamixel positions to test
    dxl_positions = [2048, 1800, 1900, 3000, 0]  # Base, Shoulder, Elbow, Wrist, Gripper
    
    # Convert to IKPy angles
    ik_angles = dynamixel_to_ik_angles(dxl_positions)
    print(f"IKPy angles (radians): {ik_angles}")
    print(f"IKPy angles (degrees): {np.degrees(ik_angles)}")
    
    # Calculate end effector position using forward kinematics
    position, rotation, _ = arm.forward_kinematics(ik_angles)
    print(f"\nEnd effector position (meters): {position}")
    print(f"End effector rotation matrix:\n{rotation}")
    
    # Visualize the configuration
    fig, ax = arm.visualize(ik_angles)
    plt.show(block=True)

if __name__ == "__main__":
    main() 