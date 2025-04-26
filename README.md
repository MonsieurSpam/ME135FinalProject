| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- | -------- | -------- |

# UART Echo Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example demonstrates how to utilize UART interfaces by echoing back to the sender any data received on
configured UART.

## How to use example

### Hardware Required

The example can be run on any development board, that is based on the Espressif SoC. The board shall be connected to a computer with a single USB cable for flashing and monitoring. The external interface should have 3.3V outputs. You may
use e.g. 3.3V compatible USB-to-Serial dongle.

### Setup the Hardware

Connect the external serial interface to the board as follows.

```
  -----------------------------------------------------------------------------------------
  | Target chip Interface | Kconfig Option     | Default ESP Pin      | External UART Pin |
  | ----------------------|--------------------|----------------------|--------------------
  | Transmit Data (TxD)   | EXAMPLE_UART_TXD   | GPIO4                | RxD               |
  | Receive Data (RxD)    | EXAMPLE_UART_RXD   | GPIO5                | TxD               |
  | Ground                | n/a                | GND                  | GND               |
  -----------------------------------------------------------------------------------------
```
Note: Some GPIOs can not be used with certain chips because they are reserved for internal use. Please refer to UART documentation for selected target.

Optionally, you can set-up and use a serial interface that has RTS and CTS signals in order to verify that the
hardware control flow works. Connect the extra signals according to the following table, configure both extra pins in
the example code `uart_echo_example_main.c` by replacing existing `UART_PIN_NO_CHANGE` macros with the appropriate pin
numbers and configure UART1 driver to use the hardware flow control by setting `.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS`
and adding `.rx_flow_ctrl_thresh = 122` to the `uart_config` structure.

```
  ---------------------------------------------------------------
  | Target chip Interface | Macro           | External UART Pin |
  | ----------------------|-----------------|--------------------
  | Transmit Data (TxD)   | ECHO_TEST_RTS   | CTS               |
  | Receive Data (RxD)    | ECHO_TEST_CTS   | RTS               |
  | Ground                | n/a             | GND               |
  ---------------------------------------------------------------
```

### Configure the project

Use the command below to configure project using Kconfig menu as showed in the table above.
The default Kconfig values can be changed such as: EXAMPLE_TASK_STACK_SIZE, EXAMPLE_UART_BAUD_RATE, EXAMPLE_UART_PORT_NUM (Refer to Kconfig file).
```
idf.py menuconfig
```

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

Type some characters in the terminal connected to the external serial interface. As result you should see echo in the same terminal which you used for typing the characters. You can verify if the echo indeed comes from ESP board by
disconnecting either `TxD` or `RxD` pin: no characters will appear when typing.

## Troubleshooting

You are not supposed to see the echo in the terminal which is used for flashing and monitoring, but in the other UART configured through Kconfig can be used.
# 6-Axis Robot Arm Control System

This project provides a complete system for controlling a 6-axis robot arm using an ESP32 microcontroller and Dynamixel servo motors. It includes:

- ESP32 firmware with Dynamixel servo control
- Python-based inverse kinematics solver using IKPy library
- Visualization tools for the robot arm
- Graphical user interface for easy control
- Serial communication between Python and ESP32

## Hardware Requirements

- ESP32 development board
- Dynamixel servo motors:
  - Joints 1-3: Dynamixel XL430-W250 motors
  - Joints 4-6: Dynamixel XL330 motors
- UART to TTL converter for Dynamixel communication
- USB cable for connecting ESP32 to computer

## Software Requirements

### ESP32 Setup

1. Install the ESP-IDF development environment
2. Configure the project:
   ```
   cd uart_echo
   idf.py menuconfig
   ```
3. Build and flash the firmware:
   ```
   idf.py build
   idf.py -p (PORT) flash
   ```

### Python Setup

1. Install required Python packages:
   ```
   pip install -r requirements.txt
   ```

## File Structure

- `main/uart_echo_example_main.c` - ESP32 firmware with Dynamixel motor control
- `main/dynamixel.h` and `main/dynamixel.c` - Dynamixel protocol implementation
- `robot_ikpy_kinematics.py` - Inverse kinematics solver using IKPy
- `esp32_robot_interface.py` - Serial communication with ESP32
- `robot_control_gui.py` - Graphical user interface for control
- `visualize_robot_frames.py` - Tool for visualizing joint coordinate frames

## Usage

### Using the Command-Line Interface

A simple command-line interface is available for systems that don't support tkinter:

```
python robot_control_cli.py
```

This provides a text-based menu with options to:
1. Connect to the ESP32
2. Set target positions manually or use presets
3. Calculate inverse kinematics
4. Send commands to the robot
5. Read current positions

### Using the GUI

For systems that support tkinter, the GUI provides a more visual way to control the robot:

```
python robot_control_gui.py
```

1. Connect to the ESP32 by selecting the correct port and clicking "Connect"
2. Enter target X, Y, Z coordinates or select a preset position
3. Click "Calculate IK" to compute the joint angles
4. Click "Move Robot" to send the commands to the ESP32

### Command Line Control

#### Inverse Kinematics

Calculate joint angles to reach a target position:

```
python robot_ikpy_kinematics.py --mode ik --position 0.1 0.0 0.2 --visualize
```

To send the calculated positions to the ESP32:

```
python robot_ikpy_kinematics.py --mode ik --position 0.1 0.0 0.2 --send-to-esp32 --port /dev/ttyUSB0
```

#### Forward Kinematics

Calculate end-effector position from joint angles:

```
python robot_ikpy_kinematics.py --mode fk --angles 0 0.5 -0.5 0 0 0 --visualize
```

#### Testing Individual Joints

Test a specific servo by setting its position:

```
python robot_ikpy_kinematics.py --mode test --joint 1 --value 2048 --visualize
```

### Visualizing Joint Frames

To visualize the coordinate frames for each joint:

```
python visualize_robot_frames.py
```

Show only specific frames:

```
python visualize_robot_frames.py --frames 0 1 2
```

Print transformation matrices:

```
python visualize_robot_frames.py --print
```

### Direct ESP32 Communication

Send commands directly to the ESP32:

```
python esp32_robot_interface.py --port /dev/ttyUSB0 --command "S1P2048"
```

Center all servos:

```
python esp32_robot_interface.py --port /dev/ttyUSB0 --center
```

Read current servo positions:

```
python esp32_robot_interface.py --port /dev/ttyUSB0 --read
```

## Serial Command Protocol

The ESP32 firmware accepts the following serial commands:

- `S{id}P{position}` - Set servo with ID to position (e.g., `S1P2048`)
- `M{pos1},{pos2},{pos3},{pos4},{pos5},{pos6}` - Set all servo positions at once
- `C` - Center all servos
- `R` - Read current positions of all servos

## Customizing the Robot

To adapt the system to your specific robot arm, modify the following:

1. Update the link lengths in `robot_ikpy_kinematics.py`:
   ```python
   BASE_HEIGHT = 0.04  # Base to joint 1 (Z axis) - 4 cm
   LINK_1_2_LENGTH = 0.02  # Joint 1 to joint 2 - 2 cm
   # ... etc.
   ```

2. Update the joint limits in `robot_ikpy_kinematics.py`:
   ```python
   JOINT_1_LIMITS = [1027, 2975, 0, 180]  # Joint 1 (base)
   JOINT_2_LIMITS = [1582, 2806, 0, 180]  # Joint 2 (shoulder)
   # ... etc.
   ```

## Troubleshooting

- **Connection issues**: Make sure you're using the correct serial port and the ESP32 is properly connected.
- **Movement problems**: Verify that your joint limits are set correctly to prevent the robot from attempting impossible positions.
- **IK failures**: If the inverse kinematics solver fails to find a solution, try a different target position that's within the robot's reachable workspace.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
