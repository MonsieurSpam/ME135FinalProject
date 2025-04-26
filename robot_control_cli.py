#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Arm Control CLI
A command-line interface for controlling the 6-axis robot arm using inverse kinematics.
This is a simplified version that doesn't require tkinter.
"""

import os
import sys
import time
import numpy as np
from math import degrees

# Add the current directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from robot_ikpy_kinematics import RobotArmIKPy
    from esp32_robot_interface import ESP32RobotInterface
except ImportError as e:
    print(f"Error importing required modules: {e}")
    print("Make sure robot_ikpy_kinematics.py and esp32_robot_interface.py are in the current directory.")
    sys.exit(1)

class RobotControlCLI:
    """Command-line interface for controlling the robot arm."""
    
    def __init__(self):
        """Initialize the CLI."""
        # Initialize robot arm
        self.robot = RobotArmIKPy()
        self.esp32 = None
        
        # Current positions
        self.current_joint_angles = [0, 0, 0, 0, 0, 0]
        self.current_target = [0.15, 0.0, 0.2]  # Default target position
        
        # Preset positions
        self.presets = {
            "home": [0.15, 0.0, 0.2],
            "forward": [0.25, 0.0, 0.1],
            "side": [0.1, 0.15, 0.15],
            "low": [0.2, 0.0, 0.05],
            "high": [0.1, 0.0, 0.3]
        }
    
    def connect_to_esp32(self, port=None):
        """Connect to the ESP32."""
        try:
            self.esp32 = ESP32RobotInterface(port=port)
            if self.esp32.connect():
                print(f"Connected to ESP32 on {self.esp32.port}")
                return True
            else:
                self.esp32 = None
                print("Failed to connect to ESP32")
                return False
        except Exception as e:
            self.esp32 = None
            print(f"Error connecting to ESP32: {e}")
            return False
    
    def disconnect_from_esp32(self):
        """Disconnect from the ESP32."""
        if self.esp32 and self.esp32.connected:
            self.esp32.disconnect()
            self.esp32 = None
            print("Disconnected from ESP32")
    
    def calculate_ik(self, target_position):
        """
        Calculate inverse kinematics for the target position.
        
        Args:
            target_position: List of [x, y, z] coordinates in meters
            
        Returns:
            bool: True if successful, False if failed
        """
        try:
            print(f"Calculating inverse kinematics for target: [{target_position[0]:.4f}, {target_position[1]:.4f}, {target_position[2]:.4f}]")
            
            # Calculate joint angles using inverse kinematics
            joint_angles = self.robot.inverse_kinematics(target_position)
            self.current_joint_angles = joint_angles
            self.current_target = target_position
            
            # Calculate the corresponding Dynamixel positions
            dynamixel_positions = self.robot.angles_to_dynamixel(joint_angles)
            
            # Print joint angles
            print("\nJoint angles (radians):")
            for i, angle in enumerate(joint_angles):
                print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f}°)")
            
            # Print Dynamixel positions
            print("\nDynamixel positions:")
            for i, position in enumerate(dynamixel_positions):
                print(f"Joint {i+1}: {position}")
            
            # Check if target is reachable by calculating forward kinematics
            actual_position, _, _ = self.robot.forward_kinematics(joint_angles)
            error = np.linalg.norm(np.array(target_position) - actual_position)
            
            print(f"\nActual position: [{actual_position[0]:.4f}, {actual_position[1]:.4f}, {actual_position[2]:.4f}]")
            print(f"Position error: {error:.4f}m")
            
            if error < 0.02:  # Less than 2cm error
                print("Target is reachable!")
            else:
                print("Warning: Target may not be fully reachable")
            
            return True
        except Exception as e:
            print(f"Error calculating inverse kinematics: {e}")
            return False
    
    def move_robot(self):
        """Send the calculated joint angles to the robot."""
        if not self.esp32 or not self.esp32.connected:
            print("Error: Not connected to ESP32")
            return False
        
        try:
            # Convert joint angles to Dynamixel positions
            dynamixel_positions = self.robot.angles_to_dynamixel(self.current_joint_angles)
            
            # Send to ESP32
            print("Sending positions to ESP32...")
            
            success = self.esp32.set_all_servo_positions(dynamixel_positions)
            
            if success:
                print("Robot moved successfully!")
            else:
                print("Failed to move robot")
            
            return success
        except Exception as e:
            print(f"Error moving robot: {e}")
            return False
    
    def center_all_servos(self):
        """Center all servos (set to neutral position)."""
        self.current_joint_angles = [0, 0, 0, 0, 0, 0]
        
        if self.esp32 and self.esp32.connected:
            print("Centering all servos...")
            success = self.esp32.center_all_servos()
            if success:
                print("Servos centered successfully")
            else:
                print("Failed to center servos")
            return success
        else:
            print("Not connected to ESP32")
            return False
    
    def read_positions(self):
        """Read current servo positions from the ESP32."""
        if not self.esp32 or not self.esp32.connected:
            print("Error: Not connected to ESP32")
            return False
        
        try:
            print("Reading positions from ESP32...")
            
            dynamixel_positions = self.esp32.read_all_servo_positions()
            
            if dynamixel_positions:
                # Convert to joint angles
                joint_angles = self.robot.dynamixel_to_angles(dynamixel_positions)
                self.current_joint_angles = joint_angles
                
                # Print joint angles
                print("\nJoint angles (radians):")
                for i, angle in enumerate(joint_angles):
                    print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f}°)")
                
                # Print Dynamixel positions
                print("\nDynamixel positions:")
                for i, position in enumerate(dynamixel_positions):
                    print(f"Joint {i+1}: {position}")
                
                return True
            else:
                print("Failed to read positions")
                return False
        except Exception as e:
            print(f"Error reading positions: {e}")
            return False
    
    def list_available_ports(self):
        """List available serial ports."""
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            
            if not ports:
                print("No serial ports found")
                return []
            
            print("\nAvailable serial ports:")
            for i, port in enumerate(ports):
                print(f"{i+1}: {port.device} - {port.description}")
            
            return [port.device for port in ports]
        except Exception as e:
            print(f"Error listing serial ports: {e}")
            return []
    
    def run_interactive(self):
        """Run the interactive command-line interface."""
        print("\n=== 6-Axis Robot Arm Control CLI ===")
        
        # Keep track of connected status
        connected = False
        
        # Main loop
        while True:
            print("\nMain Menu:")
            print("1. Connect to ESP32")
            print("2. Disconnect from ESP32")
            print("3. Set target position")
            print("4. Use preset position")
            print("5. Calculate inverse kinematics")
            print("6. Move robot to target")
            print("7. Center all servos")
            print("8. Read current positions")
            print("9. List available ports")
            print("0. Exit")
            
            choice = input("\nEnter choice (0-9): ").strip()
            
            if choice == '0':
                # Exit
                if connected:
                    self.disconnect_from_esp32()
                print("Exiting...")
                break
                
            elif choice == '1':
                # Connect to ESP32
                if connected:
                    print("Already connected to ESP32")
                else:
                    ports = self.list_available_ports()
                    if ports:
                        port_idx = input(f"Select port (1-{len(ports)}, or enter full path): ").strip()
                        try:
                            port = ports[int(port_idx) - 1] if port_idx.isdigit() and 1 <= int(port_idx) <= len(ports) else port_idx
                            connected = self.connect_to_esp32(port)
                        except Exception as e:
                            print(f"Error selecting port: {e}")
                    else:
                        port = input("Enter port manually (e.g., /dev/ttyUSB0): ").strip()
                        if port:
                            connected = self.connect_to_esp32(port)
            
            elif choice == '2':
                # Disconnect from ESP32
                if connected:
                    self.disconnect_from_esp32()
                    connected = False
                else:
                    print("Not connected to ESP32")
            
            elif choice == '3':
                # Set target position
                try:
                    x = float(input("Enter X position (meters): ").strip())
                    y = float(input("Enter Y position (meters): ").strip())
                    z = float(input("Enter Z position (meters): ").strip())
                    self.current_target = [x, y, z]
                    print(f"Target set to: [{x:.4f}, {y:.4f}, {z:.4f}]")
                except ValueError:
                    print("Invalid input. Please enter numeric values.")
            
            elif choice == '4':
                # Use preset position
                print("\nAvailable presets:")
                for i, (name, pos) in enumerate(self.presets.items()):
                    print(f"{i+1}: {name} - [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
                
                preset_choice = input(f"Select preset (1-{len(self.presets)}): ").strip()
                try:
                    if preset_choice.isdigit() and 1 <= int(preset_choice) <= len(self.presets):
                        preset_name = list(self.presets.keys())[int(preset_choice) - 1]
                        self.current_target = self.presets[preset_name]
                        print(f"Target set to {preset_name}: [{self.current_target[0]:.4f}, {self.current_target[1]:.4f}, {self.current_target[2]:.4f}]")
                except Exception as e:
                    print(f"Error selecting preset: {e}")
            
            elif choice == '5':
                # Calculate inverse kinematics
                self.calculate_ik(self.current_target)
            
            elif choice == '6':
                # Move robot to target
                if connected:
                    self.move_robot()
                else:
                    print("Not connected to ESP32")
            
            elif choice == '7':
                # Center all servos
                if connected:
                    self.center_all_servos()
                else:
                    print("Not connected to ESP32")
            
            elif choice == '8':
                # Read current positions
                if connected:
                    self.read_positions()
                else:
                    print("Not connected to ESP32")
            
            elif choice == '9':
                # List available ports
                self.list_available_ports()
            
            else:
                print("Invalid choice. Please try again.")

def main():
    """Main function."""
    cli = RobotControlCLI()
    try:
        cli.run_interactive()
    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if cli.esp32 and cli.esp32.connected:
            cli.disconnect_from_esp32()

if __name__ == "__main__":
    main() 