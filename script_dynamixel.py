#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Python script for controlling a Dynamixel XL430-W250 servo motor with ID 1
using a Waveshare bus driver connected via USB-C.
"""

import os
import sys
import time

# Add the DynamixelSDK library path if needed 
# (adjust this path if your DynamixelSDK is installed elsewhere)
# sys.path.append("/path/to/DynamixelSDK/python/dynamixel_sdk")

from dynamixel_sdk import *  # Import all Dynamixel SDK modules

# Dynamixel XL430-W250 Protocol 2.0 Control Table Addresses
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_OPERATING_MODE     = 11
ADDR_LED                = 65

# Protocol version
PROTOCOL_VERSION        = 2.0  # XL430 uses Protocol 2.0

# Default setting
DXL_ID                  = 1    # Dynamixel ID: 1
BAUDRATE                = 1000000  # Baudrate of Dynamixel
DEVICENAME              = '/dev/tty.usbmodem585A0079911'  # Check the USB port name on your system
                                                    # Example: Linux: /dev/ttyUSB0, Mac: /dev/tty.usbmodem1101

TORQUE_ENABLE           = 1     # Value for enabling torque
TORQUE_DISABLE          = 0     # Value for disabling torque
POSITION_CONTROL_MODE   = 3     # Value for Position Control Mode
MINIMUM_POSITION_VALUE  = 0     # Min position limit (0 to 4095)
MAXIMUM_POSITION_VALUE  = 4095  # Max position limit (0 to 4095)
DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Custom debug function for printing packet data
def debug_packet(direction, packet_data, description=""):
    if isinstance(packet_data, (list, tuple, bytes, bytearray)):
        hex_data = ' '.join([f"{byte:02X}" for byte in packet_data])
        print(f"[{direction}] {description} Packet: {hex_data}")
    else:
        print(f"[{direction}] {description}: {packet_data}")

# Original packetHandler methods to wrap with debug
original_write1ByteTxRx = packetHandler.write1ByteTxRx
original_write4ByteTxRx = packetHandler.write4ByteTxRx
original_read1ByteTxRx = packetHandler.read1ByteTxRx
original_read4ByteTxRx = packetHandler.read4ByteTxRx
original_ping = packetHandler.ping

# Override methods with debug versions
def debug_write1ByteTxRx(port, dxl_id, address, data):
    debug_packet("TX", [0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x04, 0x00, address & 0xFF, (address >> 8) & 0xFF, data & 0xFF], 
                f"write1ByteTxRx to ID:{dxl_id}, Address:{address}, Data:{data}")
    result, error = original_write1ByteTxRx(port, dxl_id, address, data)
    debug_packet("RX", f"Result:{result}, Error:{error}", "Response")
    return result, error

def debug_write4ByteTxRx(port, dxl_id, address, data):
    debug_packet("TX", [0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x07, 0x00, address & 0xFF, (address >> 8) & 0xFF, 
                       data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF], 
                f"write4ByteTxRx to ID:{dxl_id}, Address:{address}, Data:{data}")
    result, error = original_write4ByteTxRx(port, dxl_id, address, data)
    debug_packet("RX", f"Result:{result}, Error:{error}", "Response")
    return result, error

def debug_read1ByteTxRx(port, dxl_id, address):
    debug_packet("TX", [0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x03, 0x00, address & 0xFF, (address >> 8) & 0xFF], 
                f"read1ByteTxRx from ID:{dxl_id}, Address:{address}")
    value, result, error = original_read1ByteTxRx(port, dxl_id, address)
    debug_packet("RX", f"Value:{value}, Result:{result}, Error:{error}", "Response")
    return value, result, error

def debug_read4ByteTxRx(port, dxl_id, address):
    debug_packet("TX", [0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x03, 0x00, address & 0xFF, (address >> 8) & 0xFF], 
                f"read4ByteTxRx from ID:{dxl_id}, Address:{address}")
    value, result, error = original_read4ByteTxRx(port, dxl_id, address)
    debug_packet("RX", f"Value:{value}, Result:{result}, Error:{error}", "Response")
    return value, result, error

def debug_ping(port, dxl_id):
    debug_packet("TX", [0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x01, 0x00], f"ping to ID:{dxl_id}")
    model_number, result, error = original_ping(port, dxl_id)
    debug_packet("RX", f"Model:{model_number}, Result:{result}, Error:{error}", "Response")
    return model_number, result, error

# Replace original methods with debug versions
packetHandler.write1ByteTxRx = debug_write1ByteTxRx
packetHandler.write4ByteTxRx = debug_write4ByteTxRx
packetHandler.read1ByteTxRx = debug_read1ByteTxRx
packetHandler.read4ByteTxRx = debug_read4ByteTxRx
packetHandler.ping = debug_ping

def enable_torque():
    print(f"[TX] Sending torque enable command to ID: {DXL_ID}, Address: {ADDR_TORQUE_ENABLE}, Value: {TORQUE_ENABLE}")
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print(f"[RX] Communication result: {dxl_comm_result}, Error: {dxl_error}")
    if dxl_comm_result != COMM_SUCCESS:
        print("Torque enable failed:", packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("Torque enable error:", packetHandler.getRxPacketError(dxl_error))
        return False
    print("Dynamixel torque has been enabled")
    return True

def disable_torque():
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("Torque disable failed:", packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("Torque disable error:", packetHandler.getRxPacketError(dxl_error))
        return False
    print("Dynamixel torque has been disabled")
    return True

def set_operating_mode(mode):
    # Disable torque to change operating mode
    if not disable_torque():
        return False
    
    # Set operating mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_OPERATING_MODE, mode)
    if dxl_comm_result != COMM_SUCCESS:
        print("Operating mode change failed:", packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("Operating mode change error:", packetHandler.getRxPacketError(dxl_error))
        return False
    print(f"Operating mode has been set to {mode}")
    return True

def set_position(position):
    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID, ADDR_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        print("Position write failed:", packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("Position write error:", packetHandler.getRxPacketError(dxl_error))
        return False
    return True

def read_position():
    # Read present position
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("Position read failed:", packetHandler.getTxRxResult(dxl_comm_result))
        return None
    elif dxl_error != 0:
        print("Position read error:", packetHandler.getRxPacketError(dxl_error))
        return None
    return dxl_present_position

def toggle_led():
    # Toggle LED status
    dxl_led_value, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
        portHandler, DXL_ID, ADDR_LED)
    if dxl_comm_result != COMM_SUCCESS:
        print("LED read failed:", packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("LED read error:", packetHandler.getRxPacketError(dxl_error))
        return False
    
    # Toggle LED (0 -> 1, 1 -> 0)
    new_led_value = 1 if dxl_led_value == 0 else 0
    
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_LED, new_led_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("LED write failed:", packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("LED write error:", packetHandler.getRxPacketError(dxl_error))
        return False
    
    print(f"LED is now {'on' if new_led_value == 1 else 'off'}")
    return True

def main():
    try:
        # Open port
        if not portHandler.openPort():
            print("Failed to open the port")
            return

        # Set port baudrate
        if not portHandler.setBaudRate(BAUDRATE):
            print("Failed to change the baudrate")
            return
            
        # Add debug information to check connection
        print(f"Port opened successfully: {DEVICENAME}")
        print(f"Baudrate set to: {BAUDRATE}")
        
        # Ping the Dynamixel to check if it's responding
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)
        if dxl_comm_result != COMM_SUCCESS:
            print("Ping failed. Check your connections and device ID.")
            print("Error:", packetHandler.getTxRxResult(dxl_comm_result))
            return
        else:
            print(f"Dynamixel #{DXL_ID} responded. Model number: {dxl_model_number}")

        # Set operating mode to position control
        if not set_operating_mode(POSITION_CONTROL_MODE):
            return

        # Enable Dynamixel Torque
        if not enable_torque():
            return
        
        # Toggle LED as a visual indicator
        toggle_led()

        print("Moving to position 1000...")
        set_position(1000)
        time.sleep(2)  # Wait for the servo to reach position
        
        print("Current position:", read_position())
        
        print("Moving to position 3000...")
        set_position(3000)
        time.sleep(2)  # Wait for the servo to reach position
        
        print("Current position:", read_position())
        
        # Move back to the middle position
        print("Moving back to position 2048 (center)...")
        set_position(2048)
        time.sleep(2)  # Wait for the servo to reach position
        
        print("Current position:", read_position())
        
        # Toggle LED again
        toggle_led()

    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Disable Dynamixel Torque
        disable_torque()
        
        # Close port
        portHandler.closePort()
        print("Port closed")

if __name__ == "__main__":
    main()
