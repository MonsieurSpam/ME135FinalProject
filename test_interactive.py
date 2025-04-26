#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script to try interactive servo selection by sending numbers 1-6
"""

import serial
import time

def test_interactive_mode(port):
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
        while ser.in_waiting > 0:
            print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
        
        # Now try to select each servo by sending numbers 1-6
        for servo_id in range(1, 7):
            print(f"\nSending {servo_id} to select servo {servo_id}...")
            ser.write(f"{servo_id}\n".encode('utf-8'))
            ser.flush()
            time.sleep(0.5)
            
            # Read response
            print(f"Response to selecting servo {servo_id}:")
            start_time = time.time()
            while (time.time() - start_time) < 2:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"  {line}")
                else:
                    time.sleep(0.1)
                    
            # Now try sending W and S to move the selected servo
            print(f"\nSending W to move servo {servo_id} up...")
            ser.write(b'W\n')
            ser.flush()
            time.sleep(0.5)
            
            # Read response
            while ser.in_waiting > 0:
                print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
            
            print(f"\nSending S to move servo {servo_id} down...")
            ser.write(b'S\n')
            ser.flush()
            time.sleep(0.5)
            
            # Read response
            while ser.in_waiting > 0:
                print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
        
        # Center all servos at the end
        print("\nSending C to center all servos...")
        ser.write(b'C\n')
        ser.flush()
        time.sleep(1)
        
        # Read response
        while ser.in_waiting > 0:
            print(f"  {ser.readline().decode('utf-8', errors='ignore').strip()}")
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    port = "/dev/cu.usbserial-59100209741"
    test_interactive_mode(port) 