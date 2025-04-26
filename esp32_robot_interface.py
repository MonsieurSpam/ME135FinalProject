#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ESP32 Robot Interface
This script provides communication between Python scripts and the ESP32 microcontroller
running the Dynamixel servo controller.
"""

import serial
import time
import argparse
import os
import struct
import threading
from math import degrees

class ESP32RobotInterface:
    """Interface for communicating with the ESP32 robot controller."""
    
    def __init__(self, port=None, baudrate=115200, timeout=1.0):
        """
        Initialize the ESP32 interface.
        
        Args:
            port: Serial port (e.g., /dev/ttyUSB0 on Linux, COM3 on Windows)
            baudrate: Baud rate for serial communication
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.command_lock = threading.Lock()
        
        # Find port automatically if not specified
        if self.port is None:
            self.port = self._find_esp32_port()
    
    def _find_esp32_port(self):
        """
        Attempt to automatically find the ESP32 serial port.
        """
        import serial.tools.list_ports
        
        # Common ESP32 USB-UART bridge identifiers
        esp32_identifiers = [
            "CP210x", "CP2102", "SLAB_USBtoUART", "Silicon Labs",
            "FT232R", "USB Serial", "ESP32"
        ]
        
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            print("No serial ports found.")
            return None
        
        # First try to find ESP32 by identifier
        for port in ports:
            for identifier in esp32_identifiers:
                if identifier in port.description:
                    print(f"Found ESP32 on {port.device} ({port.description})")
                    return port.device
        
        # If not found, return the first available port
        print(f"ESP32 not identified by name. Using first available port: {ports[0].device}")
        return ports[0].device
    
    def connect(self):
        """
        Connect to the ESP32 via serial port.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        if self.port is None:
            print("Error: No serial port specified.")
            return False
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Give the serial connection time to initialize
            time.sleep(2.0)
            
            # Flush any pending data
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            self.connected = True
            print(f"Connected to ESP32 on {self.port}")
            return True
            
        except serial.SerialException as e:
            print(f"Error connecting to ESP32: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close the serial connection to the ESP32."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            print("Disconnected from ESP32")
    
    def send_command(self, command, wait_for_response=True, timeout=1.0):
        """
        Send a command to the ESP32 and optionally wait for a response.
        
        Args:
            command: Command string to send
            wait_for_response: Whether to wait for a response
            timeout: How long to wait for a response in seconds
            
        Returns:
            str: Response from the ESP32, or None if no response/timeout
        """
        if not self.connected or not self.serial:
            print("Error: Not connected to ESP32")
            return None
        
        # Add a thread lock to prevent multiple commands at once
        with self.command_lock:
            try:
                # Ensure the command ends with a newline
                if not command.endswith('\n'):
                    command += '\n'
                
                # Send the command
                self.serial.write(command.encode('utf-8'))
                self.serial.flush()
                
                # Wait for response if requested
                if wait_for_response:
                    # Set a timeout for reading the response
                    start_time = time.time()
                    response = ""
                    
                    while (time.time() - start_time) < timeout:
                        if self.serial.in_waiting > 0:
                            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                            response += line + "\n"
                            
                            # Check if the response is complete (can customize based on ESP32 protocol)
                            if response.strip().endswith("OK") or response.strip().endswith("ERROR"):
                                break
                        
                        # Small delay to prevent CPU hogging
                        time.sleep(0.01)
                    
                    return response.strip()
                
                return "Command sent (no response requested)"
                
            except Exception as e:
                print(f"Error sending command to ESP32: {e}")
                return None
    
    def set_servo_position(self, servo_id, position):
        """
        Set a specific servo to a position (in Dynamixel units).
        
        Args:
            servo_id: Servo ID (1-6)
            position: Position value (0-4095)
            
        Returns:
            bool: True if successful, False otherwise
        """
        command = f"S{servo_id}P{int(position)}"
        response = self.send_command(command)
        return response is not None and "OK" in response
    
    def set_all_servo_positions(self, positions):
        """
        Set all servos to the specified positions.
        
        Args:
            positions: List of positions (in Dynamixel units) for servos 1-6
            
        Returns:
            bool: True if successful, False otherwise
        """
        if len(positions) != 6:
            print("Error: Must provide exactly 6 position values")
            return False
        
        command = "M" + "".join([f"{int(pos)}," for pos in positions])[:-1]  # Remove trailing comma
        response = self.send_command(command)
        return response is not None and "OK" in response
    
    def set_joint_angles(self, joint_angles, from_robot_ikpy=False):
        """
        Set joint angles (in radians) and convert to Dynamixel positions.
        
        Args:
            joint_angles: List of 6 joint angles in radians
            from_robot_ikpy: Whether angles come from the RobotArmIKPy class
            
        Returns:
            bool: True if successful, False otherwise
        """
        if len(joint_angles) != 6:
            print("Error: Must provide exactly 6 joint angles")
            return False
        
        if from_robot_ikpy:
            # If angles are from robot_ikpy_kinematics.py, they need to be converted
            from robot_ikpy_kinematics import RobotArmIKPy
            robot = RobotArmIKPy()
            positions = robot.angles_to_dynamixel(joint_angles)
        else:
            # Simple conversion for testing (assumes 2048 is center)
            positions = []
            for i, angle in enumerate(joint_angles):
                # Convert radian to position (very simplified)
                # This assumes 2048 is 0 degrees and each radian is ~650 units
                # You should use the proper conversion from your robot_ikpy_kinematics.py
                center = 2048
                scale = 650  # approx. conversion factor
                positions.append(int(center + angle * scale))
        
        # Send the positions to the ESP32
        return self.set_all_servo_positions(positions)
    
    def center_all_servos(self):
        """
        Move all servos to their center positions.
        
        Returns:
            bool: True if successful, False otherwise
        """
        command = "C"  # Command for centering all servos
        response = self.send_command(command)
        return response is not None and "OK" in response
    
    def read_all_servo_positions(self):
        """
        Read current positions of all servos.
        
        Returns:
            list: Positions of all servos, or None if failed
        """
        command = "R"  # Command for reading all servo positions
        response = self.send_command(command)
        
        if response and ":" in response:
            # Parse the response, expected format: "Positions: 2048,2048,2048,2048,2048,2048"
            try:
                positions_part = response.split(":")[-1].strip()
                positions = [int(p) for p in positions_part.split(",")]
                return positions
            except Exception as e:
                print(f"Error parsing servo positions: {e}")
        
        return None

def main():
    """Main function for testing the ESP32 interface."""
    parser = argparse.ArgumentParser(description="ESP32 Robot Interface")
    parser.add_argument('--port', type=str, help='Serial port for ESP32 (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--command', type=str, help='Send a raw command to ESP32')
    parser.add_argument('--center', action='store_true', help='Center all servos')
    parser.add_argument('--servo', type=int, choices=range(1,7), help='Servo ID (1-6) to control')
    parser.add_argument('--position', type=int, help='Position for the specified servo (0-4095)')
    parser.add_argument('--angles', type=float, nargs=6, help='Set all servo angles (in radians)')
    parser.add_argument('--read', action='store_true', help='Read current servo positions')
    
    args = parser.parse_args()
    
    # Create the interface
    interface = ESP32RobotInterface(port=args.port, baudrate=args.baudrate)
    
    # Connect to ESP32
    if not interface.connect():
        print("Failed to connect to ESP32.")
        return
    
    try:
        # Process commands
        if args.command:
            response = interface.send_command(args.command)
            print(f"Response: {response}")
        
        elif args.center:
            success = interface.center_all_servos()
            print(f"Center all servos: {'Success' if success else 'Failed'}")
        
        elif args.servo is not None and args.position is not None:
            success = interface.set_servo_position(args.servo, args.position)
            print(f"Set servo {args.servo} to position {args.position}: {'Success' if success else 'Failed'}")
        
        elif args.angles:
            success = interface.set_joint_angles(args.angles, from_robot_ikpy=True)
            print(f"Set joint angles: {'Success' if success else 'Failed'}")
            
            # Print the angles
            print("\nJoint angles (radians):")
            for i, angle in enumerate(args.angles):
                print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f}°)")
        
        elif args.read:
            positions = interface.read_all_servo_positions()
            if positions:
                print("Current servo positions:")
                for i, pos in enumerate(positions):
                    print(f"Servo {i+1}: {pos}")
            else:
                print("Failed to read servo positions")
        
        else:
            print("No action specified. Use --help to see available commands.")
    
    finally:
        # Disconnect from ESP32
        interface.disconnect()

if __name__ == "__main__":
    main() 