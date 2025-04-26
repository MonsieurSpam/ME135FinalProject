#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script to control only Servo 1 using interactive mode with W and S
"""

import serial
import time

def control_servo1(port):
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=2)
        print(f"Connected to {port}")
        
        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # First press B to ensure we're at the main menu
        print("\nSending B to go back to main menu...")
        ser.write(b'B\n')
        ser.flush()
        time.sleep(1)
        print("Response:")
        while ser.in_waiting > 0:
            print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
        
        # Select servo 1
        print("\nSending 1 to select servo 1...")
        ser.write(b'1\n')
        ser.flush()
        time.sleep(1)
        
        # Read response
        print("Response to selecting servo 1:")
        while ser.in_waiting > 0:
            print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
        
        # Now move the servo up with W
        print("\nSending W to move servo 1 up...")
        ser.write(b'W\n')
        ser.flush()
        time.sleep(1)
        
        # Read response
        print("Response to W command:")
        while ser.in_waiting > 0:
            print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
        
        # Wait for 2 seconds to observe the movement
        time.sleep(2)
        
        # Now move the servo down with S
        print("\nSending S to move servo 1 down...")
        ser.write(b'S\n')
        ser.flush()
        time.sleep(1)
        
        # Read response
        print("Response to S command:")
        while ser.in_waiting > 0:
            print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
        
        # Wait for 2 seconds to observe the movement
        time.sleep(2)
        
        # Center all servos before exiting
        print("\nSending C to center all servos...")
        ser.write(b'C\n')
        ser.flush()
        time.sleep(1)
        
        # Read response
        print("Response to C command:")
        while ser.in_waiting > 0:
            print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    port = "/dev/cu.usbserial-59100209741"
    control_servo1(port) 