#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script to sweep Servo 1 through its full range using direct position commands
"""

import serial
import time

def sweep_servo1(port):
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=2)
        print(f"Connected to {port}")
        
        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # First center all servos
        print("\nCentering all servos...")
        ser.write(b'C\n')
        ser.flush()
        time.sleep(2)  # Give time for the servos to center
        
        # Servo 1 limits
        min_pos = 1027  # 0 degrees (clockwise)
        center_pos = 2048  # ~90 degrees
        max_pos = 2975  # 180 degrees (counter-clockwise)
        
        # Move to minimum position
        print(f"\nMoving Servo 1 to minimum position: {min_pos}")
        command = f"S1P{min_pos}\n"
        ser.write(command.encode('utf-8'))
        ser.flush()
        time.sleep(2)  # Wait for movement
        
        # Move to center position
        print(f"\nMoving Servo 1 to center position: {center_pos}")
        command = f"S1P{center_pos}\n"
        ser.write(command.encode('utf-8'))
        ser.flush()
        time.sleep(2)  # Wait for movement
        
        # Move to maximum position
        print(f"\nMoving Servo 1 to maximum position: {max_pos}")
        command = f"S1P{max_pos}\n"
        ser.write(command.encode('utf-8'))
        ser.flush()
        time.sleep(2)  # Wait for movement
        
        # Sweep from maximum to minimum in steps
        print("\nSweeping from maximum to minimum position...")
        steps = 10
        step_size = (max_pos - min_pos) // steps
        
        for i in range(steps + 1):
            position = max_pos - (i * step_size)
            print(f"  Moving to position: {position}")
            command = f"S1P{position}\n"
            ser.write(command.encode('utf-8'))
            ser.flush()
            time.sleep(0.5)  # Small delay between steps
        
        # Return to center before exiting
        print(f"\nReturning Servo 1 to center position: {center_pos}")
        command = f"S1P{center_pos}\n"
        ser.write(command.encode('utf-8'))
        ser.flush()
        time.sleep(2)  # Wait for movement
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    port = "/dev/cu.usbserial-59100209741"
    sweep_servo1(port) 