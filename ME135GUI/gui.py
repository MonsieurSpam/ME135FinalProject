import sys
import os
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QTextEdit, 
                             QLineEdit, QMessageBox, QComboBox, QFrame,
                             QSlider, QGroupBox, QSpacerItem, QSizePolicy)
from PySide6.QtCore import Slot, Qt, QProcess, QTimer, QDateTime
from PySide6.QtGui import QColor, QTextCursor
import serial.tools.list_ports
import serial
import math

# Add parent directory to path to import computer vision module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from computer_vision.final_detection_realtime import stablized_centers
from so100_ik_control import SO100IKControl

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Initialize IK control
        self.ik_control = None
        
        # Initialize serial connection
        self.serial_connection = None
        self.last_connected_port = None
        
        # Store detected positions
        self.detected_red_position = None
        self.detected_blue_position = None
        
        # Initialize position reading timer
        self.position_timer = QTimer()
        self.position_timer.timeout.connect(self.read_positions)
        self.position_timer.start(100)  # Read every 100ms
        
        # Initialize connection timeout timer (but don't start it yet)
        self.connection_timeout = QTimer()
        self.connection_timeout.timeout.connect(self.check_connection)
        
        # Initialize reconnection timer
        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self.attempt_reconnect)
        
        # Track last position update time
        self.last_position_update = QDateTime.currentMSecsSinceEpoch()
        
        # Set window properties
        self.setWindowTitle("ME135 Control Panel")
        self.setMinimumSize(800, 600)
        
        # Create central widget and layout
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        
        # Create top control panel
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        control_layout.setSpacing(30)
        
        # Common group box style
        group_style = """
            QGroupBox {
                border: 1px solid #cccccc;
                border-radius: 5px;
                margin-top: 15px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0px 5px 0px 5px;
                color: #666666;
            }
        """
        
        # Common text box style for dark mode
        text_box_style = """
            QLineEdit {
                padding: 2px;
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #404040;
                border-radius: 3px;
            }
        """
        
        # Style for action buttons (moved up so it's available for all buttons)
        button_style = """
            QPushButton {
                padding: 10px;
                background-color: #4a86e8;
                color: white;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #5c9ce6;
            }
        """
        
        # Connection Status Group
        connection_group = QGroupBox()
        connection_group.setStyleSheet(group_style)
        connection_layout = QVBoxLayout(connection_group)
        connection_layout.setSpacing(5)
        connection_label = QLabel("Connection Status")
        connection_label.setAlignment(Qt.AlignCenter)
        connection_layout.addWidget(connection_label)
        
        self.connection_status = QFrame()
        self.connection_status.setFixedSize(60, 60)
        self.connection_status.setStyleSheet("""
            QFrame {
                background-color: red;
                border-radius: 30px;
                border: 2px solid #666;
            }
        """)
        connection_layout.addWidget(self.connection_status, alignment=Qt.AlignCenter)
        control_layout.addWidget(connection_group, 1)  # Add stretch factor of 1
        
        # COM Port Group
        port_group = QGroupBox()
        port_group.setStyleSheet(group_style)
        port_layout = QVBoxLayout(port_group)
        port_layout.setSpacing(5)
        port_label = QLabel("COM Port")
        port_label.setAlignment(Qt.AlignCenter)
        port_layout.addWidget(port_label)
        
        self.port_display = QLineEdit()
        self.port_display.setReadOnly(True)
        self.port_display.setAlignment(Qt.AlignCenter)
        self.port_display.setFixedHeight(25)
        self.port_display.setStyleSheet(text_box_style)
        port_layout.addWidget(self.port_display)
        
        self.port_selector = QComboBox()
        self.port_selector.setFixedHeight(25)
        self.port_selector.setStyleSheet("""
            QComboBox {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #404040;
                border-radius: 3px;
                padding: 2px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                image: none;
                border: none;
            }
        """)
        self.port_selector.currentTextChanged.connect(self.port_changed)
        port_layout.addWidget(self.port_selector)
        self.scan_ports()
        control_layout.addWidget(port_group, 1)  # Add stretch factor of 1
        
        # Mode Switch Group
        mode_group = QGroupBox()
        mode_group.setStyleSheet(group_style)
        mode_layout = QVBoxLayout(mode_group)
        mode_layout.setSpacing(5)
        mode_label = QLabel("Operation Mode")
        mode_label.setAlignment(Qt.AlignCenter)
        mode_layout.addWidget(mode_label)
        
        # Create a horizontal layout for the slider and labels
        slider_layout = QHBoxLayout()
        slider_layout.setSpacing(15)
        
        # Add labels for the slider with fixed width
        teleop_label = QLabel("Teleop")
        teleop_label.setFixedWidth(45)
        teleop_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        auto_label = QLabel("Auto")
        auto_label.setFixedWidth(45)
        auto_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        
        # Create the slider
        self.mode_switch = QSlider(Qt.Horizontal)
        self.mode_switch.setMinimum(0)
        self.mode_switch.setMaximum(1)
        self.mode_switch.setFixedWidth(50)
        self.mode_switch.setFixedHeight(20)
        self.mode_switch.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #999999;
                height: 6px;
                background: #2d2d2d;
                margin: 2px 0;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #4a86e8;
                border: 1px solid #5c5c5c;
                width: 16px;
                margin: -2px 0;
                border-radius: 8px;
            }
        """)
        
        slider_layout.addWidget(teleop_label)
        slider_layout.addWidget(self.mode_switch)
        slider_layout.addWidget(auto_label)
        mode_layout.addLayout(slider_layout)
        
        # Mode Display
        mode_display_label = QLabel("Current Mode")
        mode_display_label.setAlignment(Qt.AlignCenter)
        mode_layout.addWidget(mode_display_label)
        
        self.mode_display = QLineEdit()
        self.mode_display.setReadOnly(True)
        self.mode_display.setAlignment(Qt.AlignCenter)
        self.mode_display.setFixedHeight(25)
        self.mode_display.setText("Teleop")
        self.mode_display.setStyleSheet(text_box_style)
        mode_layout.addWidget(self.mode_display)
        
        control_layout.addWidget(mode_group, 1)  # Add stretch factor of 1
        
        # Add control panel to main layout
        main_layout.addWidget(control_panel)
        
        # Create console output section EARLY so self.console_output exists before scan_ports/port_changed
        console_group = QGroupBox("Console Output")
        console_group.setStyleSheet(group_style)
        console_layout = QVBoxLayout(console_group)
        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 12px;
                padding: 5px;
                border: 1px solid #333333;
                border-radius: 3px;
            }
        """)
        console_layout.addWidget(self.console_output)
        
        # Add middle section (three columns)
        middle_section = QWidget()
        middle_layout = QHBoxLayout(middle_section)
        middle_layout.setSpacing(20)
        
        # Left column - Teleop Panel
        teleop_group = QGroupBox("Teleop Controls")
        teleop_group.setStyleSheet(group_style)
        teleop_layout = QVBoxLayout(teleop_group)
        teleop_layout.setSpacing(10)
        
        # Add gripper control button
        self.gripper_button = QPushButton("Gripper: Open")
        self.gripper_button.setCheckable(True)
        self.gripper_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:checked {
                background-color: #f44336;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:checked:hover {
                background-color: #da190b;
            }
        """)
        teleop_layout.addWidget(self.gripper_button)
        
        # Add joint angle sliders
        self.joint_sliders = []
        joint_names = ["Base", "Shoulder", "Elbow", "Wrist"]
        joint_ranges = [
            (90, 270),   # Base: 90 to 270
            (140, 270),  # Shoulder: 140 to 270
            (90, 270),   # Elbow: 90 to 270
            (90, 270)    # Wrist: 90 to 270
        ]
        initial_positions = [180, 160, 180, 270]  # Initial positions for each joint
        
        for i, (name, (min_val, max_val), init_pos) in enumerate(zip(joint_names, joint_ranges, initial_positions)):
            slider_layout = QVBoxLayout()
            label = QLabel(f"{name} Joint")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            slider.setValue(init_pos)  # Set initial position
            value_label = QLabel(f"{init_pos}°")
            value_label.setAlignment(Qt.AlignCenter)
            
            slider_layout.addWidget(label)
            slider_layout.addWidget(slider)
            slider_layout.addWidget(value_label)
            
            teleop_layout.addLayout(slider_layout)
            self.joint_sliders.append((slider, value_label))
        
        middle_layout.addWidget(teleop_group, 1)  # Add stretch factor of 1
        
        # Middle column - Status Display
        status_group = QGroupBox("Status Display")
        status_group.setStyleSheet(group_style)
        status_layout = QVBoxLayout(status_group)
        status_layout.setSpacing(10)
        
        # Joint angle displays
        self.joint_displays = []
        for name, (min_val, max_val), init_pos in zip(joint_names, joint_ranges, initial_positions):
            display_layout = QHBoxLayout()
            label = QLabel(f"{name}:")
            display = QLineEdit()
            display.setReadOnly(True)
            display.setAlignment(Qt.AlignCenter)
            display.setText(f"{init_pos}°")
            display.setStyleSheet(text_box_style)
            display_layout.addWidget(label)
            display_layout.addWidget(display)
            status_layout.addLayout(display_layout)
            self.joint_displays.append(display)
        
        # Add gripper status in the same format as joints
        gripper_layout = QHBoxLayout()
        gripper_label = QLabel("Gripper:")
        self.gripper_status = QLineEdit()
        self.gripper_status.setReadOnly(True)
        self.gripper_status.setAlignment(Qt.AlignCenter)
        self.gripper_status.setText("Open")
        self.gripper_status.setStyleSheet(text_box_style)
        gripper_layout.addWidget(gripper_label)
        gripper_layout.addWidget(self.gripper_status)
        status_layout.addLayout(gripper_layout)
        
        # Add stretch to push Home button to the bottom
        # Add Home button at the bottom
        self.home_button = QPushButton("Home")
        self.home_button.setStyleSheet(button_style)
        status_layout.addWidget(self.home_button)

        self.demo_button = QPushButton("Run Demo Sequence")
        self.demo_button.setStyleSheet(button_style)
        status_layout.addWidget(self.demo_button)
        
        middle_layout.addWidget(status_group, 1)  # Add stretch factor of 1
        
        # Right column - Action Buttons
        action_group = QGroupBox("Actions")
        action_group.setStyleSheet(group_style)
        action_layout = QVBoxLayout(action_group)
        action_layout.setSpacing(10)
        
        # Combine Place Location label and X/Y/Z inputs into a single compact widget
        place_location_widget = QWidget()
        place_location_vlayout = QVBoxLayout(place_location_widget)
        place_location_vlayout.setContentsMargins(0, 0, 0, 0)
        place_location_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        place_location_label = QLabel("Place Location (mm):")
        place_location_label.setAlignment(Qt.AlignCenter)
        place_location_vlayout.addWidget(place_location_label)
        place_location_vlayout.addSpacing(8)  # Fixed spacing between label and input boxes
        coord_layout = QHBoxLayout()
        coord_layout.setContentsMargins(0, 0, 0, 0)
        coord_layout.setSpacing(10)
        # X
        x_layout = QVBoxLayout()
        x_layout.setContentsMargins(0, 0, 0, 0)
        x_layout.setSpacing(2)
        x_label = QLabel("X:")
        self.x_coord = QLineEdit()
        self.x_coord.setPlaceholderText("X")
        self.x_coord.setStyleSheet(text_box_style)
        x_layout.addWidget(x_label)
        x_layout.addWidget(self.x_coord)
        coord_layout.addLayout(x_layout)
        # Y
        y_layout = QVBoxLayout()
        y_layout.setContentsMargins(0, 0, 0, 0)
        y_layout.setSpacing(2)
        y_label = QLabel("Y:")
        self.y_coord = QLineEdit()
        self.y_coord.setPlaceholderText("Y")
        self.y_coord.setStyleSheet(text_box_style)
        y_layout.addWidget(y_label)
        y_layout.addWidget(self.y_coord)
        coord_layout.addLayout(y_layout)
        # Z
        z_layout = QVBoxLayout()
        z_layout.setContentsMargins(0, 0, 0, 0)
        z_layout.setSpacing(2)
        z_label = QLabel("Z:")
        self.z_coord = QLineEdit()
        self.z_coord.setPlaceholderText("Z")
        self.z_coord.setStyleSheet(text_box_style)
        z_layout.addWidget(z_label)
        z_layout.addWidget(self.z_coord)
        coord_layout.addLayout(z_layout)
        # Prevent vertical stretching of the coordinate row
        coord_container = QWidget()
        coord_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        coord_container.setLayout(coord_layout)
        place_location_vlayout.addWidget(coord_container)
        action_layout.addWidget(place_location_widget)
        
        # Add action buttons
        self.find_red = QPushButton("Find Red")
        self.find_blue = QPushButton("Find Blue")
        self.pick_place = QPushButton("Execute Pick and Place")
        
        self.find_red.setStyleSheet(button_style)
        self.find_blue.setStyleSheet(button_style)
        self.pick_place.setStyleSheet(button_style)
        
        action_layout.addWidget(self.find_red)
        action_layout.addWidget(self.find_blue)
        action_layout.addWidget(self.pick_place)
        
        middle_layout.addWidget(action_group, 1)  # Add stretch factor of 1
        
        # Add middle section to main layout
        main_layout.addWidget(middle_section)
        # Add console section to main layout (now at the bottom)
        main_layout.addWidget(console_group)
        
        # Initialize process for command execution
        self.process = QProcess()
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        
        # Set central widget
        self.setCentralWidget(central_widget)
        
        # Connect signals and slots
        self.connect_signals()
        
        # Add initial console message
        self.console_output.append("Console initialized. Ready for commands...")
    
    def connect_signals(self):
        """Connect widget signals to slot functions"""
        self.mode_switch.valueChanged.connect(self.toggle_mode)
        self.port_selector.currentTextChanged.connect(self.port_changed)
        
        # Connect joint slider signals
        for i, (slider, label) in enumerate(self.joint_sliders):
            slider.valueChanged.connect(lambda v, i=i: self.update_joint_angle(i, v))
        
        # Connect action buttons
        self.gripper_button.clicked.connect(self.toggle_gripper)
        self.find_red.clicked.connect(self.find_red_cube)
        self.find_blue.clicked.connect(self.find_blue_cube)
        self.pick_place.clicked.connect(self.execute_pick_and_place)
        self.home_button.clicked.connect(lambda: self.send_serial_command("C"))
        self.demo_button.clicked.connect(lambda: self.send_serial_command("d"))

    @Slot(int)
    def toggle_mode(self, value):
        """Toggle between teleop and auto modes"""
        if value == 1:  # Auto mode
            self.mode_display.setText("Auto")
            self.console_output.append("Mode changed to: Auto")
            
            # Disable teleop controls (left section)
            self.gripper_button.setEnabled(False)
            for slider, _ in self.joint_sliders:
                slider.setEnabled(False)
            
            # Enable auto controls (right section)
            self.find_red.setEnabled(True)
            self.find_blue.setEnabled(True)
            self.pick_place.setEnabled(True)
            self.x_coord.setEnabled(True)
            self.y_coord.setEnabled(True)
            self.z_coord.setEnabled(True)
            
        else:  # Teleop mode
            self.mode_display.setText("Teleop")
            self.console_output.append("Mode changed to: Teleop")
            
            # Enable teleop controls (left section)
            self.gripper_button.setEnabled(True)
            for slider, _ in self.joint_sliders:
                slider.setEnabled(True)
            
            # Disable auto controls (right section)
            self.find_red.setEnabled(False)
            self.find_blue.setEnabled(False)
            self.pick_place.setEnabled(False)
            self.x_coord.setEnabled(False)
            self.y_coord.setEnabled(False)
            self.z_coord.setEnabled(False)
    
    def find_red_cube(self):
        """Find and store the position of the red cube"""
        try:
            self.console_output.append("Searching for red cube...")
            position = stablized_centers(override_color='red')
            position[1] = position[1] + 0.02 
            position[2] = position[2] + 0.05
            if position is not None:
                self.detected_red_position = position
                self.console_output.append(f"Red cube found at: {position}")
            else:
                self.console_output.append("No red cube detected")
        except Exception as e:
            self.console_output.append(f"Error finding red cube: {str(e)}")

    def find_blue_cube(self):
        """Find and store the position of the blue cube"""
        try:
            self.console_output.append("Searching for blue cube...")
            position = stablized_centers(override_color='blue')
            position[1] = position[1] + 0.02 
            position[2] = position[2] + 0.05
            if position is not None:
                self.detected_blue_position = position
                self.console_output.append(f"Blue cube found at: {position}")
            else:
                self.console_output.append("No blue cube detected")
        except Exception as e:
            self.console_output.append(f"Error finding blue cube: {str(e)}")

    def execute_pick_and_place(self):
        """Execute pick and place sequence using detected position and user-specified place position"""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.console_output.append("Error: Not connected to any port")
            return

        try:
            # Get place position from user input
            try:
                place_x = float(self.x_coord.text())
                place_y = float(self.y_coord.text())
                place_z = float(self.z_coord.text())
            except ValueError:
                self.console_output.append("Error: Invalid place position coordinates")
                return

            place_position = [place_x, place_y, place_z]

            # Determine which cube to pick based on which was last detected
            if self.detected_red_position is not None:
                pick_position = self.detected_red_position
                self.console_output.append("Using red cube position for pick")
            elif self.detected_blue_position is not None:
                pick_position = self.detected_blue_position
                self.console_output.append("Using blue cube position for pick")
            else:
                self.console_output.append("Error: No cube position detected. Please find a cube first.")
                return

            # Initialize IK control if not already done
            if self.ik_control is None:
                self.ik_control = SO100IKControl(port=self.last_connected_port)
                if not self.ik_control.connect():
                    self.console_output.append("Error: Failed to initialize IK control")
                    return

            # Execute pick and place sequence
            self.console_output.append("Executing pick and place sequence...")
            success = self.ik_control.execute_pick_and_place(pick_position, place_position)
            
            if success:
                self.console_output.append("Pick and place sequence completed successfully!")
            else:
                self.console_output.append("Pick and place sequence failed!")

        except Exception as e:
            self.console_output.append(f"Error during pick and place: {str(e)}")

    @Slot(str)
    def port_changed(self, port):
        """Handle port selection change"""
        self.port_display.setText(port)
        
        # Stop the connection timeout timer if it's running
        if self.connection_timeout.isActive():
            self.connection_timeout.stop()
        
        # Close existing connection if any
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.serial_connection = None
        
        # Close IK control if it exists
        if self.ik_control:
            self.ik_control.disconnect()
            self.ik_control = None
        
        try:
            # Create new serial connection
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=1
            )
            
            # Store the successfully connected port
            self.last_connected_port = port
            
            # Start the connection timeout timer
            self.connection_timeout.start(500)  # Check every 500ms
            
            # Update connection status
            self.connection_status.setStyleSheet("""
                QFrame {
                    background-color: green;
                    border-radius: 30px;
                    border: 2px solid #666;
                }
            """)
            if hasattr(self, 'console_output'):
                self.console_output.append(f"Connected to port: {port}")
                
        except Exception as e:
            # Update connection status to indicate error
            self.connection_status.setStyleSheet("""
                QFrame {
                    background-color: red;
                    border-radius: 30px;
                    border: 2px solid #666;
                }
            """)
            if hasattr(self, 'console_output'):
                self.console_output.append(f"Error connecting to port {port}: {str(e)}")
            self.serial_connection = None
    
    def convert_to_dynamixel_angle(self, joint_index, angle):
        """Convert joint angle to Dynamixel angle format"""
        # Convert from GUI angle to Dynamixel angle
        if joint_index == 0:  # Base
            angle_deg = 180 + angle  # Shift to match Dynamixel range
        elif joint_index == 1:  # Shoulder
            angle_deg = 270 + angle  # Shift to match Dynamixel range
        elif joint_index == 2:  # Elbow
            angle_deg = 90 + angle  # Shift to match Dynamixel range
        elif joint_index == 3:  # Wrist
            angle_deg = 90 + angle  # Shift to match Dynamixel range
        
        # Ensure angle is within 0-360° range
        while angle_deg < 0:
            angle_deg += 360
        while angle_deg >= 360:
            angle_deg -= 360
            
        return angle_deg

    @Slot(int, int)
    def update_joint_angle(self, joint_index, value):
        """Update joint angle display when slider changes"""
        self.joint_sliders[joint_index][1].setText(f"{value}°")
        # Remove the joint display update since it's now handled by read_positions
        self.console_output.append(f"Joint {joint_index + 1} angle changed to {value}°")
        
        # Get all current joint angles
        angles = [slider.value() for slider, _ in self.joint_sliders[:4]]  # Only first 4 joints
        
        # Format and send the I command
        angles_str = ','.join([f"{angle:.2f}" for angle in angles])
        cmd = f"I{angles_str}\n"
        
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(cmd.encode())
                self.serial_connection.flush()
                self.console_output.append(f"Sent command: {cmd.strip()}")
            except Exception as e:
                self.console_output.append(f"Error sending command: {str(e)}")
    
    @Slot()
    def toggle_gripper(self):
        """Toggle gripper state"""
        if self.gripper_button.isChecked(): 
            self.send_serial_command("G")
            self.gripper_button.setText("Gripper: Closed")
            self.gripper_status.setText("Closed")
            self.console_output.append("Gripper closed")
        else:
            self.send_serial_command("O")
            self.gripper_button.setText("Gripper: Open")
            self.gripper_status.setText("Open")
            self.console_output.append("Gripper opened")
    
    def handle_stdout(self):
        """Handle standard output from process"""
        data = self.process.readAllStandardOutput().data().decode()
        self.console_output.append(data.strip())
    
    def handle_stderr(self):
        """Handle standard error from process"""
        data = self.process.readAllStandardError().data().decode()
        self.console_output.append(f"Error: {data.strip()}")
    
    def execute_command(self, command):
        """Execute a system command and display output in console"""
        self.console_output.append(f"> {command}")
        self.process.start(command)
    
    def scan_ports(self):
        """Scan for available serial ports and update the dropdown menu."""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_selector.clear()
        if port_list:
            self.port_selector.addItems(port_list)
        else:
            self.port_selector.addItem("No ports found")

    def send_serial_command(self, command):
        """Send a command over the serial connection"""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.console_output.append("Error: Not connected to any port")
            return False
            
        try:
            # Ensure command ends with newline
            if not command.endswith('\n'):
                command += '\n'
                
            # Send the command
            self.serial_connection.write(command.encode('utf-8'))
            self.serial_connection.flush()
            self.console_output.append(f"Sent command: {command.strip()}")
            return True
            
        except Exception as e:
            self.console_output.append(f"Error sending command: {str(e)}")
            return False

    def check_connection(self):
        """Check if we've received a position update recently"""
        current_time = QDateTime.currentMSecsSinceEpoch()
        if current_time - self.last_position_update > 500:  # Changed from 200 to 500
            # No position update received in the last 500ms
            self.connection_status.setStyleSheet("""
                QFrame {
                    background-color: red;
                    border-radius: 30px;
                    border: 2px solid #666;
                }
            """)
            if self.serial_connection and self.serial_connection.is_open:
                self.console_output.append("Connection lost - No position updates received")
                self.serial_connection.close()
                self.serial_connection = None
                # Stop the connection timeout timer
                self.connection_timeout.stop()
                # Start reconnection attempts
                if self.last_connected_port:
                    self.reconnect_timer.start(1000)  # Try to reconnect every second

    def attempt_reconnect(self):
        """Attempt to reconnect to the last known port"""
        if not self.last_connected_port:
            self.reconnect_timer.stop()
            return
            
        try:
            # Try to create new serial connection
            self.serial_connection = serial.Serial(
                port=self.last_connected_port,
                baudrate=115200,
                timeout=1
            )
            
            # If successful, stop reconnection attempts and start connection monitoring
            self.reconnect_timer.stop()
            self.connection_timeout.start(500)
            self.console_output.append(f"Reconnected to port: {self.last_connected_port}")
            
        except Exception as e:
            # Connection failed, keep trying
            self.console_output.append(f"Reconnection attempt failed: {str(e)}")
            if self.serial_connection:
                self.serial_connection.close()
                self.serial_connection = None

    def read_positions(self):
        """Read current joint positions from serial buffer"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return
            
        try:
            # Read all available data
            while self.serial_connection.in_waiting:
                line = self.serial_connection.readline().decode().strip()
                
                # Check if this is a joint position update (starts with 'J')
                if line.startswith('J'):
                    # Update last position update time
                    self.last_position_update = QDateTime.currentMSecsSinceEpoch()
                    
                    # Update connection status to connected
                    self.connection_status.setStyleSheet("""
                        QFrame {
                            background-color: green;
                            border-radius: 30px;
                            border: 2px solid #666;
                        }
                    """)
                    
                    # Parse positions
                    positions = line[1:].split(',')  # Remove 'J' prefix and split
                    if len(positions) >= 6:  # We expect 6 positions
                        # Convert positions to angles and update displays
                        for i in range(4):  # First 4 are joint positions
                            try:
                                position = int(positions[i])
                                # Convert from 0-4096 to degrees (assuming 0-360 mapping)
                                angle = (position / 4096.0) * 360.0
                                self.joint_displays[i].setText(f"{angle:.1f}°")
                            except ValueError:
                                continue
                        
                        # Handle gripper position
                        try:
                            gripper_pos = int(positions[5])  # 6th position is gripper
                            gripper_state = "Open" if gripper_pos > 2900 else "Closed"
                            self.gripper_status.setText(gripper_state)
                        except ValueError:
                            pass
                            
        except Exception as e:
            # Only log the error if it's not a temporary connection issue
            if not isinstance(e, (serial.SerialException, serial.SerialTimeoutException)):
                self.console_output.append(f"Error reading positions: {str(e)}")
            
            # Don't immediately close the connection on error
            # Instead, let the connection timeout handler deal with it
            if self.serial_connection and self.serial_connection.is_open:
                # Just update the status to indicate potential issues
                self.connection_status.setStyleSheet("""
                    QFrame {
                        background-color: yellow;
                        border-radius: 30px;
                        border: 2px solid #666;
                    }
                """)

def main():
    # Create the Qt Application
    app = QApplication(sys.argv)
    
    # Create and show the main window
    window = MainWindow()
    window.show()
    
    # Execute the application
    sys.exit(app.exec())

if __name__ == "__main__":
    main()