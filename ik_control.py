#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import sys
import glob
import numpy as np
import argparse
from robot_ikpy_kinematics import RobotArmIKPy

def find_esp32_port():
    """Find the ESP32 port by looking for common port patterns"""
    # Common ESP32 port patterns
    patterns = [
        '/dev/tty.SLAB_USBtoUART*',  # macOS with CP210x
        '/dev/tty.usbserial*',       # macOS with FTDI
        '/dev/ttyUSB*',              # Linux
        '/dev/ttyACM*'               # Linux alternative
    ]
    
    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]  # Return the first matching port
    
    return None

class RobotController:
    def __init__(self, port=None, baudrate=115200):
        # If no port specified, try to find it automatically
        if port is None:
            port = find_esp32_port()
            if port is None:
                raise RuntimeError("Could not find ESP32 port. Please specify it manually.")
        
        print(f"Connecting to ESP32 on port {port}...")
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.ik_solver = RobotArmIKPy()
        time.sleep(2)  # Wait for serial connection to establish
        print("Connected to ESP32!")
        
    def send_command(self, command):
        """Send a command to the ESP32 and wait for response"""
        print(f"Sending: {command}")
        self.serial.write((command + '\n').encode())
        time.sleep(0.1)  # Small delay to ensure command is processed
        
        # Read response
        response = self.serial.readline().decode().strip()
        while response:
            print(f"Received: {response}")
            response = self.serial.readline().decode().strip()
    
    def move_to_position(self, x, y, z, visualize=False, visualize_only=False):
        """Move the arm to a specific position using IK"""
        # Calculate IK solution
        print(f"Calculating IK for position ({x}, {y}, {z})...")
        joint_angles = self.ik_solver.inverse_kinematics([x, y, z])
        
        if joint_angles is None:
            print("Error: Could not find valid IK solution")
            return
        
        # Convert angles to degrees
        joint_angles_deg = np.degrees(joint_angles)
        print(f"Joint angles (degrees): {joint_angles_deg}")
        
        if not visualize_only:
            # Send angles using I command format
            angles_str = ','.join([f"{angle:.2f}" for angle in joint_angles_deg])
            self.send_command(f"I{angles_str}")
            
            # Wait for the movement to complete
            time.sleep(4)  # Adjust this based on your movement time
        
        if visualize:
            try:
                # Visualize the robot configuration
                self.ik_solver.plot_arm(joint_angles, [x, y, z])
                import matplotlib.pyplot as plt
                plt.show()
            except Exception as e:
                print(f"Visualization error: {e}")
                print("Continuing without visualization...")
    
    def close(self):
        """Close the serial connection"""
        self.serial.close()

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Control robot arm using IK')
    parser.add_argument('--port', type=str, help='Serial port for ESP32')
    parser.add_argument('--position', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                       help='Target position in meters')
    parser.add_argument('--visualize', action='store_true',
                       help='Visualize the robot configuration')
    parser.add_argument('--visualize-only', action='store_true',
                       help='Only visualize the robot configuration without moving it')
    args = parser.parse_args()
    
    try:
        # Initialize the controller
        controller = RobotController(port=args.port)
        
        if args.position:
            # Move to specified position
            x, y, z = args.position
            controller.move_to_position(x, y, z, args.visualize, args.visualize_only)
        else:
            # Interactive mode
            while True:
                # Get target position from user
                try:
                    x = float(input("Enter X position (meters): "))
                    y = float(input("Enter Y position (meters): "))
                    z = float(input("Enter Z position (meters): "))
                    
                    # Move to the target position
                    controller.move_to_position(x, y, z, args.visualize, args.visualize_only)
                    
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                    continue
                
                # Ask if user wants to continue
                if input("Move to another position? (y/n): ").lower() != 'y':
                    break
                    
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print("Please make sure:")
        print("1. The ESP32 is connected")
        print("2. The correct port is specified")
        print("3. No other program is using the port")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.close()

if __name__ == "__main__":
    main() 