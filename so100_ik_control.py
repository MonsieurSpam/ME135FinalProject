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
import argparse
import matplotlib.pyplot as plt

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
            
        # Format command: Iangle1,angle2,angle3,angle4
        # Only send first 4 angles (base, shoulder, elbow, wrist)
        angles_str = ','.join([f"{angle:.2f}" for angle in angles_degrees[:4]])
        cmd = f"I{angles_str}\n"
        
        try:
            self.serial.write(cmd.encode())
            print(f"Sent command: {cmd.strip()}")
            return True
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            return False
            
    def open_gripper(self):
        """Open the gripper."""
        if not self.serial or not self.serial.is_open:
            print("Error: Not connected to ESP32")
            return False
            
        try:
            self.serial.write(b"O\n")
            # Keep reading responses for up to 2 seconds
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if self.serial.in_waiting:
                    response = self.serial.readline().decode().strip()
                    print(f"Gripper open response: {response}")
                    if response == "OK":
                        return True
                time.sleep(0.1)
            return False
        except serial.SerialException as e:
            print(f"Error sending open gripper command: {e}")
            return False
            
    def close_gripper(self):
        """Close the gripper."""
        if not self.serial or not self.serial.is_open:
            print("Error: Not connected to ESP32")
            return False
            
        try:
            self.serial.write(b"G\n")
            # Keep reading responses for up to 2 seconds
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if self.serial.in_waiting:
                    response = self.serial.readline().decode().strip()
                    print(f"Gripper close response: {response}")
                    if response == "OK":
                        return True
                time.sleep(0.1)
            return False
        except serial.SerialException as e:
            print(f"Error sending close gripper command: {e}")
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
        
        # Check if any joint angle change is too large (> 90 degrees)
        max_angle_change = 90.0  # Maximum allowed angle change in degrees
        needs_intermediate = False
        
        # If this is not the first movement, check angle changes
        if hasattr(self, 'last_joint_angles'):
            for i in range(4):  # Check first 4 joints
                angle_change = abs(joint_angles_degrees[i] - self.last_joint_angles[i])
                if angle_change > max_angle_change:
                    needs_intermediate = True
                    break
        
        if needs_intermediate:
            print("Large joint angle change detected, adding intermediate position...")
            # Create intermediate position halfway between current and target
            intermediate_position = [
                (target_position[0] + self.last_position[0]) / 2,
                (target_position[1] + self.last_position[1]) / 2,
                (target_position[2] + self.last_position[2]) / 2
            ]
            
            # Move to intermediate position first
            if not self.move_to_position(intermediate_position, target_orientation):
                return False
            time.sleep(1)  # Wait at intermediate position
            
            # Then move to final position
            return self.move_to_position(target_position, target_orientation)
        
        # Store current position and angles for next movement
        self.last_position = target_position
        self.last_joint_angles = joint_angles_degrees
        
        # Send to ESP32
        return self.send_joint_angles(joint_angles_degrees)
        
    def execute_pick_and_place(self, pick_position, place_position, 
                             approach_height=0.05,  # Height above target
                             grip_height=0.02):     # Height for gripping
        """Execute a pick and place sequence."""
        try:
            # 1. Move to home position
            print("\n=== Step 1: Moving to home position ===")
            self.send_command("C")
            time.sleep(4)  # Wait for movement to complete
            
            # 2. Open gripper
            print("\n=== Step 2: Opening gripper ===")
            if not self.open_gripper():
                print("Error: Failed to open gripper")
                return False
            time.sleep(1)  # Wait for gripper to open
            
            # 3. Move to approach position above pick location
            print("\n=== Step 3: Moving to pick approach position ===")
            approach_pos = [pick_position[0], pick_position[1], pick_position[2] + approach_height]
            print(f"Target position: {approach_pos}")
            if not self.move_to_position(approach_pos):
                print("Error: Failed to move to approach position")
                return False
            time.sleep(2)  # Wait for movement to complete
            
            # 4. Lower to grip position
            print("\n=== Step 4: Lowering to grip position ===")
            grip_pos = [pick_position[0], pick_position[1], pick_position[2] + grip_height]
            print(f"Target position: {grip_pos}")
            if not self.move_to_position(grip_pos):
                print("Error: Failed to move to grip position")
                return False
            time.sleep(2)  # Wait for movement to complete
            
            # 5. Close gripper
            print("\n=== Step 5: Closing gripper ===")
            if not self.close_gripper():
                print("Error: Failed to close gripper")
                return False
            time.sleep(1)  # Wait for gripper to close
            
            # 6. Raise with object
            print("\n=== Step 6: Raising with object ===")
            print(f"Target position: {approach_pos}")
            if not self.move_to_position(approach_pos):
                print("Error: Failed to raise with object")
                return False
            time.sleep(2)  # Wait for movement to complete

            print("\n=== Step 7: Returning to home position ===")
            self.send_command("C")
            time.sleep(4)  # Wait for movement to complete
            
            print("\n=== Step 8: Moving to place approach position ===")
            place_approach = [place_position[0], place_position[1], place_position[2] + approach_height]
            print(f"Target position: {place_approach}")
            if not self.move_to_position(place_approach):
                print("Error: Failed to move to place approach position")
                return False
            time.sleep(2)  # Wait for movement to complete
            
            # 8. Lower to place position
            print("\n=== Step 9: Lowering to place position ===")
            place_pos = [place_position[0], place_position[1], place_position[2] + grip_height]
            print(f"Target position: {place_pos}")
            if not self.move_to_position(place_pos):
                print("Error: Failed to move to place position")
                return False
            time.sleep(2)  # Wait for movement to complete
            
            # 9. Open gripper
            print("\n=== Step 10: Opening gripper to release object ===")
            if not self.open_gripper():
                print("Error: Failed to open gripper")
                return False
            time.sleep(1)  # Wait for gripper to open
            
            # 10. Raise arm
            print("\n=== Step 11: Raising arm ===")
            print(f"Target position: {place_approach}")
            if not self.move_to_position(place_approach):
                print("Error: Failed to raise arm")
                return False
            time.sleep(2)  # Wait for movement to complete
            
            # 11. Return home
            print("\n=== Step 12: Returning to home position ===")
            self.send_command("C")
            time.sleep(2)  # Wait for movement to complete
            
            print("Pick and place sequence completed successfully!")
            return True
            
        except Exception as e:
            print(f"Error during pick and place sequence: {e}")
            return False
        
    def send_command(self, cmd):
        """Send a raw command to the ESP32."""
        if not self.serial or not self.serial.is_open:
            print("Error: Not connected to ESP32")
            return False
        
        try:
            self.serial.write(f"{cmd}\n".encode())
            response = self.serial.readline().decode().strip()
            print(f"Command response: {response}")
            return response == "OK"
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            return False
        
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

def main():
    parser = argparse.ArgumentParser(description='SO100 Arm Control')
    parser.add_argument('--mode', choices=['move', 'pickplace'], required=True,
                      help='Mode: "move" for direct IK movement, "pickplace" for pick and place sequence')
    parser.add_argument('--x', type=float, help='X coordinate in meters')
    parser.add_argument('--y', type=float, help='Y coordinate in meters')
    parser.add_argument('--z', type=float, help='Z coordinate in meters')
    parser.add_argument('--pick-x', type=float, help='Pick X coordinate in meters')
    parser.add_argument('--pick-y', type=float, help='Pick Y coordinate in meters')
    parser.add_argument('--pick-z', type=float, help='Pick Z coordinate in meters')
    parser.add_argument('--place-x', type=float, help='Place X coordinate in meters')
    parser.add_argument('--place-y', type=float, help='Place Y coordinate in meters')
    parser.add_argument('--place-z', type=float, help='Place Z coordinate in meters')
    
    args = parser.parse_args()
    
    with SO100IKControl() as arm_control:
        if args.mode == 'move':
            if args.x is None or args.y is None or args.z is None:
                print("Error: X, Y, and Z coordinates are required for move mode")
                return
                
            target_position = [args.x, args.y, args.z]
            target_orientation = np.array([[0, 0, 1],
                                         [0, 1, 0],
                                         [-1, 0, 0]])  # Gripper pointing downward
            
            print(f"Moving to position: {target_position}")
            success = arm_control.move_to_position(target_position, target_orientation)
            
            if success:
                print("Movement completed successfully!")
            else:
                print("Movement failed!")
                
        elif args.mode == 'pickplace':
            if (args.pick_x is None or args.pick_y is None or args.pick_z is None or
                args.place_x is None or args.place_y is None or args.place_z is None):
                print("Error: Pick and place coordinates are required for pickplace mode")
                return
                
            pick_position = [args.pick_x, args.pick_y, args.pick_z]
            place_position = [args.place_x, args.place_y, args.place_z]
            
            print(f"Pick position: {pick_position}")
            print(f"Place position: {place_position}")
            
            success = arm_control.execute_pick_and_place(pick_position, place_position)
            
            if success:
                print("Pick and place sequence completed successfully!")
            else:
                print("Pick and place sequence failed!")

if __name__ == "__main__":
    main() 