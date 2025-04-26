#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script to send B command to go back to main menu, then C to center motors
"""

import serial
import time

def send_command_sequence(port):
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=2)
        print(f"Connected to {port}")
        
        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # First send B to go back to main menu
        print("\nSending B command to go back to main menu...")
        ser.write(b'B\n')
        ser.flush()
        time.sleep(1)  # Wait a moment
        
        # Read any responses
        print("Response to B command:")
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"  {line}")
        
        # Now send C to center the motors
        print("\nSending C command to center motors...")
        ser.write(b'C\n')
        ser.flush()
        time.sleep(1)  # Wait a moment
        
        # Read any responses
        print("Response to C command:")
        start_time = time.time()
        while (time.time() - start_time) < 5:  # Read for up to 5 seconds
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"  {line}")
            else:
                time.sleep(0.1)
                # If we've had some response and there's a pause, we can stop reading
                if time.time() - start_time > 2:
                    break
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    port = "/dev/cu.usbserial-59100209741"
    send_command_sequence(port) 