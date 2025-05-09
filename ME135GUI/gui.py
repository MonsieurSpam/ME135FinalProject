import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QTextEdit, 
                             QLineEdit, QMessageBox, QComboBox, QFrame,
                             QSlider, QGroupBox, QSpacerItem, QSizePolicy)
from PySide6.QtCore import Slot, Qt, QProcess
from PySide6.QtGui import QColor, QTextCursor
import serial.tools.list_ports

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
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
        for name in joint_names:
            slider_layout = QVBoxLayout()
            label = QLabel(f"{name} Joint")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(180)  # Assuming 0-180 degree range
            slider.setValue(90)  # Start at middle position
            value_label = QLabel("90°")
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
        for name in joint_names:
            display_layout = QHBoxLayout()
            label = QLabel(f"{name}:")
            display = QLineEdit()
            display.setReadOnly(True)
            display.setAlignment(Qt.AlignCenter)
            display.setText("90°")
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
        status_layout.addStretch(1)
        # Add Home button at the bottom
        self.home_button = QPushButton("Home")
        self.home_button.setStyleSheet(button_style)
        status_layout.addWidget(self.home_button)
        
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
        self.find_red.clicked.connect(lambda: self.execute_command("find_red"))
        self.find_blue.clicked.connect(lambda: self.execute_command("find_blue"))
        self.pick_place.clicked.connect(lambda: self.execute_command("pick_and_place"))
        self.home_button.clicked.connect(lambda: self.execute_command("return_home"))
    
    @Slot(int)
    def toggle_mode(self, value):
        """Toggle between teleop and auto modes"""
        if value == 1:
            self.mode_display.setText("Auto")
            self.console_output.append("Mode changed to: Auto")
        else:
            self.mode_display.setText("Teleop")
            self.console_output.append("Mode changed to: Teleop")
    
    @Slot(str)
    def port_changed(self, port):
        """Handle port selection change"""
        self.port_display.setText(port)
        self.connection_status.setStyleSheet("""
            QFrame {
                background-color: green;
                border-radius: 30px;
                border: 2px solid #666;
            }
        """)
        if hasattr(self, 'console_output'):
            self.console_output.append(f"Connected to port: {port}")
    
    @Slot(int, int)
    def update_joint_angle(self, joint_index, value):
        """Update joint angle display when slider changes"""
        self.joint_sliders[joint_index][1].setText(f"{value}°")
        self.joint_displays[joint_index].setText(f"{value}°")
        self.console_output.append(f"Joint {joint_index + 1} angle changed to {value}°")
    
    @Slot()
    def toggle_gripper(self):
        """Toggle gripper state"""
        if self.gripper_button.isChecked():
            self.gripper_button.setText("Gripper: Closed")
            self.gripper_status.setText("Closed")
            self.console_output.append("Gripper closed")
        else:
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