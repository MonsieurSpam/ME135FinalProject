#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple command interface for ESP32 Dynamixel controller
"""

import serial
import time
import sys

def send_command(port, command):
    """Send a command to the ESP32 and print the response"""
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=3)
        print(f"Connected to {port}")
        
        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Add newline if not present
        if not command.endswith('\n'):
            command += '\n'
        
        # Send the command
        print(f"Sending command: {command.strip()}")
        ser.write(command.encode('utf-8'))
        ser.flush()
        
        # Read response with timeout
        print("Response:")
        start_time = time.time()
        while (time.time() - start_time) < 5:  # 5 second timeout
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"  {line}")
                # If we get an OK or ERROR, we're done
                if "OK" in line or "ERROR" in line:
                    break
            else:
                time.sleep(0.1)
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

def print_help():
    """Print help information"""
    print("\nESP32 Dynamixel Controller Command Interface")
    print("Available commands:")
    print("  SxPyyy - Set servo x to position yyy (e.g., S1P2048)")
    print("  M2048,2048,2048,2048,2048,2048 - Move all servos to specified positions")
    print("  C - Center all servos")
    print("  R - Read all servo positions")
    print("\nExamples:")
    print("  python servo_command.py /dev/cu.usbserial-59100209741 C")
    print("  python servo_command.py /dev/cu.usbserial-59100209741 S1P2048")
    print("  python servo_command.py /dev/cu.usbserial-59100209741 R")

def interactive_mode(port):
    """Interactive command mode"""
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=3)
        print(f"Connected to {port}")
        
        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print("\nESP32 Dynamixel Controller - Interactive Mode")
        print("Type commands and press Enter. Type 'exit' to quit.")
        print("Available commands:")
        print("  SxPyyy - Set servo x to position yyy (e.g., S1P2048)")
        print("  M2048,2048,2048,2048,2048,2048 - Move all servos to specified positions")
        print("  C - Center all servos")
        print("  R - Read all servo positions")
        
        while True:
            # Get command from user
            command = input("\nEnter command (or 'exit' to quit): ")
            if command.lower() == 'exit':
                break
            
            # Add newline if not present
            if not command.endswith('\n'):
                command += '\n'
            
            # Send the command
            print(f"Sending: {command.strip()}")
            ser.write(command.encode('utf-8'))
            ser.flush()
            
            # Read response with timeout
            print("Response:")
            start_time = time.time()
            response_received = False
            
            while (time.time() - start_time) < 5:  # 5 second timeout
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"  {line}")
                    response_received = True
                    
                    # If we get an OK or ERROR, we're done
                    if "OK" in line or "ERROR" in line:
                        break
                else:
                    # If we've received some response but now there's a pause, break
                    if response_received and (time.time() - start_time) > 0.5:
                        break
                    time.sleep(0.1)
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python servo_command.py <port> [command]")
        print("Example: python servo_command.py /dev/cu.usbserial-59100209741 C")
        print("If no command is provided, interactive mode will be started.")
        print_help()
        sys.exit(1)
    
    port = sys.argv[1]
    
    if len(sys.argv) >= 3:
        command = sys.argv[2]
        send_command(port, command)
    else:
        interactive_mode(port) 