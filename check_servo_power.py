#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to check if the servos are properly powered and responding
"""

import serial
import time
import sys

def check_servo_connection(port):
    try:
        # Open serial connection
        ser = serial.Serial(port=port, baudrate=115200, timeout=2)
        print(f"Connected to {port}")
        
        # Ensure connection is ready
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # First, let's send a command to check if the ESP32 is responding
        print("\n1. Testing ESP32 communication...")
        print("   Sending newline command to check response...")
        ser.write(b'\n')
        ser.flush()
        time.sleep(0.5)
        
        # Read response
        response = ""
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            response += line + "\n"
            print(f"   Response: {line}")
        
        if response:
            print("   ✅ ESP32 is responding to commands")
        else:
            print("   ❌ No response from ESP32")
        
        # Now let's check the servo status
        print("\n2. Checking servo status...")
        print("   Sending 'R' command to read servo positions...")
        ser.write(b'R\n')
        ser.flush()
        time.sleep(1)
        
        # Read response
        response = ""
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            response += line + "\n"
            print(f"   Response: {line}")
        
        if "Positions:" in response:
            print("   ✅ Servos are responding to position queries")
        else:
            print("   ❌ No position data received from servos")
            print("   This may indicate that servos are not powered or not connected")
        
        # Try to move servo 1 to center position
        print("\n3. Testing servo movement...")
        print("   Sending command to move servo 1 to center position (2048)...")
        ser.write(b'S1P2048\n')
        ser.flush()
        time.sleep(1)
        
        # Read response
        response = ""
        start_time = time.time()
        while (time.time() - start_time) < 3:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                response += line + "\n"
                print(f"   Response: {line}")
            else:
                time.sleep(0.1)
                if response and time.time() - start_time > 1:
                    break
        
        if "Position set" in response or "OK" in response:
            print("   ✅ Servo position command was acknowledged")
            print("   If the servo didn't move, check: ")
            print("     - Is external power connected to the servo bus?")
            print("     - Are the servo connections secure?")
            print("     - Is the correct servo ID (1) being used?")
        else:
            print("   ❌ No acknowledgment from servo position command")
        
        # Try to center all servos
        print("\n4. Testing center all servos command...")
        print("   Sending 'C' command to center all servos...")
        ser.write(b'C\n')
        ser.flush()
        time.sleep(1)
        
        # Read response
        response = ""
        start_time = time.time()
        while (time.time() - start_time) < 3:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                response += line + "\n"
                print(f"   Response: {line}")
            else:
                time.sleep(0.1)
                if response and time.time() - start_time > 1:
                    break
        
        # Print troubleshooting steps
        print("\n==== TROUBLESHOOTING STEPS ====")
        print("If servos are not moving:")
        print("1. Make sure the external power supply is connected to the servos")
        print("2. Check that the power supply provides adequate voltage (usually 12V)")
        print("3. Verify all connections between ESP32 and servo bus")
        print("4. Confirm the correct servo IDs are being used")
        print("5. Try disconnecting and reconnecting the power")
        print("6. Check if any servos are making noise or getting warm (indicating they're trying to move)")
        
        # Close connection
        ser.close()
        print("\nConnection closed")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python check_servo_power.py <port>")
        print("Example: python check_servo_power.py /dev/cu.usbserial-59100209741")
        sys.exit(1)
        
    port = sys.argv[1]
    check_servo_connection(port) 