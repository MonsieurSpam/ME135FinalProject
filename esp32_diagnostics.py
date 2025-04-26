#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ESP32 Communication Diagnostics
A tool to diagnose communication issues with the ESP32
"""

import os
import sys
import time
import argparse
import serial
import serial.tools.list_ports

def detect_ports():
    """Detect available serial ports."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found! Check your connections.")
        return []
    
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port.device} - {port.description}")
    
    return ports

def test_serial_connection(port, baudrate=115200, timeout=1.0):
    """Test basic serial connection."""
    print(f"\nTesting serial connection to {port} at {baudrate} baud...")
    
    try:
        # Try to open the port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE
        )
        
        print("✓ Serial port opened successfully")
        
        # Check if the port is actually open
        if ser.is_open:
            print("✓ Port is open and ready for communication")
        else:
            print("✗ Port opened but is not reporting as open")
            return False
        
        # Reset buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("✓ Input and output buffers cleared")
        
        # Close the port
        ser.close()
        print("✓ Port closed successfully")
        
        return True
    
    except serial.SerialException as e:
        print(f"✗ Failed to open serial port: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def send_command_get_response(port, command, baudrate=115200, timeout=5.0):
    """Send a command and get the response."""
    print(f"\nSending command: '{command}'")
    
    try:
        # Open the port
        with serial.Serial(port=port, baudrate=baudrate, timeout=timeout) as ser:
            # Clear buffers
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Ensure command ends with newline
            if not command.endswith('\n'):
                command += '\n'
            
            # Send the command
            ser.write(command.encode('utf-8'))
            ser.flush()
            print(f"✓ Command sent ({len(command)} bytes)")
            
            # Wait for response
            print("Waiting for response...")
            start_time = time.time()
            response = ""
            
            # Read for timeout seconds or until we get a complete response
            while (time.time() - start_time) < timeout:
                if ser.in_waiting > 0:
                    chunk = ser.readline().decode('utf-8', errors='replace')
                    response += chunk
                    print(f"Received: {chunk.strip()}")
                    
                    # Check for completion - either OK/ERROR or a reasonable amount of data
                    if "OK" in response or "ERROR" in response:
                        print("✓ Complete response received (OK/ERROR)")
                        break
                    
                    # For commands that don't have OK/ERROR responses, wait for either:
                    # 1. No more data for a short period
                    # 2. A minimum amount of response data
                    if (time.time() - start_time) > 1.0 and len(response) > 0:
                        if ser.in_waiting == 0:
                            # No more data for a moment, consider response complete
                            time.sleep(0.1)  # Small delay to make sure no more data
                            if ser.in_waiting == 0:
                                print("✓ Response appears complete (no more data)")
                                break
                
                # Small delay to prevent CPU hogging
                time.sleep(0.01)
            
            if not response:
                print("✗ No response received within timeout period")
            
            # Convert raw response to hex for detailed debugging
            raw_bytes = response.encode('utf-8')
            hex_repr = ' '.join([f"{b:02x}" for b in raw_bytes])
            print(f"\nRaw response as hex: {hex_repr}")
            
            return response
    
    except Exception as e:
        print(f"✗ Error during communication: {e}")
        return None

def run_diagnostics(port, baudrate=115200):
    """Run a full diagnostic test suite."""
    print(f"\n===== ESP32 COMMUNICATION DIAGNOSTICS =====")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print("=" * 42)
    
    # Test 1: Basic serial connection
    if not test_serial_connection(port, baudrate):
        print("\n❌ Basic serial connection test failed. Please check:")
        print("  - Is the ESP32 connected and powered on?")
        print("  - Do you have permission to access the serial port?")
        print("  - Is another program using the port?")
        return False
    
    print("\n✅ Basic serial connection test passed")
    
    # Test 2: Send commands
    print("\n----- Command Tests -----")
    
    # Test with empty line
    print("\nTesting with empty line (should be ignored):")
    send_command_get_response(port, "", baudrate)
    
    # Test read positions command
    print("\nTesting read positions command (R):")
    read_response = send_command_get_response(port, "R", baudrate)
    
    # Test center command
    print("\nTesting center command (C):")
    center_response = send_command_get_response(port, "C", baudrate)
    
    # Test move command for all servos
    print("\nTesting move all command (M2048,2048,2048,2048,2048,2048):")
    move_all_response = send_command_get_response(port, "M2048,2048,2048,2048,2048,2048", baudrate)
    
    # Test servo position command
    print("\nTesting servo position command (S1P2048):")
    servo_response = send_command_get_response(port, "S1P2048", baudrate)
    
    # Test invalid command
    print("\nTesting invalid command (Z):")
    invalid_response = send_command_get_response(port, "Z", baudrate)
    
    # Check if we got expected responses
    success = True
    
    if read_response and "Positions:" in read_response and "OK" in read_response:
        print("✅ Read positions command responded correctly")
        # Extract positions for verification
        try:
            pos_part = read_response.split("Positions:")[1].split("OK")[0].strip()
            pos_values = [int(p) for p in pos_part.split(",")]
            print(f"  Reported positions: {pos_values}")
        except:
            print("  Could not parse position values")
    else:
        print("❌ Read positions command did not respond as expected")
        success = False
    
    if center_response and "OK" in center_response:
        print("✅ Center command responded correctly")
    else:
        print("❌ Center command did not respond as expected")
        success = False
    
    if move_all_response and "OK" in move_all_response:
        print("✅ Move all command responded correctly")
    else:
        print("❌ Move all command did not respond as expected")
        success = False
    
    if servo_response and "OK" in servo_response:
        print("✅ Servo position command responded correctly")
    else:
        print("❌ Servo position command did not respond as expected")
        success = False
    
    if invalid_response and "ERROR" in invalid_response:
        print("✅ Invalid command handled correctly")
    else:
        print("❌ Invalid command not handled as expected")
        success = False
    
    # Overall result
    print("\n===== DIAGNOSTIC SUMMARY =====")
    if success:
        print("✅ All communication tests passed! The ESP32 appears to be responding correctly.")
    else:
        print("❌ Some communication tests failed. Check the logs above for details.")
        print("\nTroubleshooting suggestions:")
        print("1. Verify the correct baud rate (default in code: 115200).")
        print("2. Make sure the ESP32 firmware has the command handlers properly implemented.")
        print("3. Check the serial wiring and connections.")
        print("4. Try resetting the ESP32 and try again.")
        print("5. Check that no other program is currently connected to the ESP32.")
    
    return success

def test_servo_limits(port, baudrate=115200):
    """Test servo positions using the calibrated limits."""
    print(f"\n===== TESTING SERVO LIMITS =====")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print("=" * 42)
    
    # Joint limits as provided by the user
    joint_limits = {
        1: {"min": 1027, "max": 2975, "min_deg": 0, "max_deg": 180},  # Clockwise to counterclockwise
        2: {"min": 1582, "max": 2806, "min_deg": 0, "max_deg": 180},  # 0° to 180°
        3: {"min": 1241, "max": 3000, "min_deg": 0, "max_deg": 180},  # Straight to bent
        4: {"min": 1523, "max": 2476, "min_deg": 0, "max_deg": 90},   # Backward to forward
        5: {"min": 1830, "max": 2200, "min_deg": 0, "max_deg": 45},   # Counterclockwise to clockwise
        6: {"min": 2000, "max": 2634, "min_deg": 90, "max_deg": 0}    # Closed to open
    }
    
    success = True
    
    # First, center all servos
    print("\nCentering all servos...")
    center_response = send_command_get_response(port, "C", baudrate)
    if center_response and "OK" in center_response:
        print("✅ Servos centered successfully")
    else:
        print("❌ Failed to center servos")
        success = False
    
    time.sleep(2)  # Give time for servos to move
    
    # Test each servo at its minimum position
    print("\nTesting each servo at minimum position:")
    for servo_id, limits in joint_limits.items():
        min_pos = limits["min"]
        min_deg = limits["min_deg"]
        
        print(f"\nMoving Servo {servo_id} to minimum position: {min_pos} ({min_deg}°)")
        command = f"S{servo_id}P{min_pos}"
        response = send_command_get_response(port, command, baudrate)
        
        if response and "OK" in response:
            print(f"✅ Servo {servo_id} moved to minimum position")
        else:
            print(f"❌ Failed to move Servo {servo_id} to minimum position")
            success = False
        
        time.sleep(2)  # Give time for servo to move
    
    # Center again before testing maximum positions
    print("\nCentering all servos again...")
    center_response = send_command_get_response(port, "C", baudrate)
    
    time.sleep(2)  # Give time for servos to move
    
    # Test each servo at its maximum position
    print("\nTesting each servo at maximum position:")
    for servo_id, limits in joint_limits.items():
        max_pos = limits["max"]
        max_deg = limits["max_deg"]
        
        print(f"\nMoving Servo {servo_id} to maximum position: {max_pos} ({max_deg}°)")
        command = f"S{servo_id}P{max_pos}"
        response = send_command_get_response(port, command, baudrate)
        
        if response and "OK" in response:
            print(f"✅ Servo {servo_id} moved to maximum position")
        else:
            print(f"❌ Failed to move Servo {servo_id} to maximum position")
            success = False
        
        time.sleep(2)  # Give time for servo to move
    
    # Center one last time
    print("\nCentering all servos one last time...")
    center_response = send_command_get_response(port, "C", baudrate)
    
    # Overall result
    print("\n===== SERVO LIMITS TEST SUMMARY =====")
    if success:
        print("✅ All servo limit tests passed!")
    else:
        print("❌ Some servo limit tests failed. Check the logs above for details.")
    
    return success

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="ESP32 Communication Diagnostics")
    parser.add_argument('--port', type=str, help='Serial port path')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--list', action='store_true', help='List available serial ports')
    parser.add_argument('--command', type=str, help='Send a specific command to the ESP32')
    parser.add_argument('--test-limits', action='store_true', help='Test servo limits')
    
    args = parser.parse_args()
    
    # List ports if requested
    if args.list:
        detect_ports()
        return
    
    # Get port if not specified
    port = args.port
    if not port:
        ports = detect_ports()
        if not ports:
            return
        
        # Let user select a port
        port_idx = input("\nSelect port number (or enter full path): ")
        try:
            if port_idx.isdigit() and 1 <= int(port_idx) <= len(ports):
                port = ports[int(port_idx) - 1].device
            else:
                port = port_idx
        except:
            print("Invalid port selection")
            return
    
    # If command is specified, just send that command
    if args.command:
        send_command_get_response(port, args.command, args.baudrate)
    elif args.test_limits:
        test_servo_limits(port, args.baudrate)
    else:
        # Run full diagnostics
        run_diagnostics(port, args.baudrate)

if __name__ == "__main__":
    main() 