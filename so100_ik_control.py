#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SO100 Arm IK Control
This script interfaces between the SO100 IKPy model and the ESP32 using Dynamixel protocol.
"""

import serial
import time
import numpy as np
from so100_ikpy_model import SO100Arm
from math import degrees

class SO100IKControl:
    def __init__(self, port='/dev/tty.usbserial-59100209741', baudrate=115200):
        """Initialize the SO100 IK control system."""
        self.arm = SO100Arm()
        self.serial = None
        self.port = port
        self.baudrate = baudrate
        
    def connect(self):
        """Connect to the ESP32 via serial port."""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from the ESP32."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected from ESP32")
            
    def convert_ik_angles_to_dynamixel(self, joint_angles):
        """Convert IKPy joint angles to Dynamixel angles."""
        angles_deg = []
        for i, angle in enumerate(joint_angles):
            # Convert from IKPy coordinate system to Dynamixel coordinate system
            if i == 0:  # Base
                angle_deg = 180 + degrees(angle)  # Shift to match Dynamixel range
            elif i == 1:  # Shoulder
                angle_deg = 270 + degrees(angle)  # Shift to match Dynamixel range
            elif i == 2:  # Elbow
                angle_deg = 90 + degrees(angle)  # Shift to match Dynamixel range (90° is straight)
            elif i == 3:  # Wrist rotation
                angle_deg = 90 + degrees(angle)  # Shift to match Dynamixel range
            elif i == 4:  # Wrist pitch
                angle_deg = degrees(angle)  # Already in correct range
            elif i == 5:  # Gripper
                angle_deg = degrees(angle)  # Already in correct range
            
            # Ensure angle is within 0-360° range
            while angle_deg < 0:
                angle_deg += 360
            while angle_deg >= 360:
                angle_deg -= 360
            
            angles_deg.append(angle_deg)
        return angles_deg
        
    def send_joint_angles(self, angles_degrees):
        """Send joint angles to the ESP32 using the I command format."""
        if not self.serial or not self.serial.is_open:
            print("Error: Not connected to ESP32")
            return False
            
        # Format command: Iangle1,angle2,angle3,angle4,angle5,angle6
        angles_str = ','.join([f"{angle:.2f}" for angle in angles_degrees])
        cmd = f"I{angles_str}\n"
        
        try:
            self.serial.write(cmd.encode())
            print(f"Sent command: {cmd.strip()}")
            return True
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            return False
            
    def move_to_position(self, target_position, target_orientation=None):
        """Move the arm to a target position using inverse kinematics."""
        # Calculate joint angles using IK
        joint_angles = self.arm.inverse_kinematics(target_position, target_orientation)
        
        if joint_angles is None:
            print("Error: Could not find valid joint angles for target position")
            return False
            
        # Convert IKPy angles to Dynamixel angles
        joint_angles_degrees = self.convert_ik_angles_to_dynamixel(joint_angles)
        print(f"Joint angles (degrees): {joint_angles_degrees}")
        
        # Send to ESP32
        return self.send_joint_angles(joint_angles_degrees)
        
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

def main():
    # Example usage
    target_position = [0.12, -0.09, 0.1]  # Target position in meters
    target_orientation = np.array([[0, 0, 1],
                                 [0, 1, 0],
                                 [-1, 0, 0]])  # Gripper pointing downward
    
    with SO100IKControl() as arm_control:
        # Move to target position
        success = arm_control.move_to_position(target_position, target_orientation)
        
        if success:
            print("Movement command sent successfully")
        else:
            print("Failed to send movement command")
            
        # Wait for movement to complete
        time.sleep(2.0)

if __name__ == "__main__":
    main() 