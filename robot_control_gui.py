#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Arm Control GUI
A graphical interface for controlling the 6-axis robot arm using inverse kinematics
and visualizing the arm's movement.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import threading
import time
import sys
import os

# Add the current directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from robot_ikpy_kinematics import RobotArmIKPy
from esp32_robot_interface import ESP32RobotInterface

class RobotControlGUI:
    """GUI for controlling the robot arm."""
    
    def __init__(self, master):
        """Initialize the GUI."""
        self.master = master
        self.master.title("6-Axis Robot Arm Control")
        self.master.geometry("1200x800")
        self.master.resizable(True, True)
        
        # Set up variables
        self.x_var = tk.DoubleVar(value=0.1)
        self.y_var = tk.DoubleVar(value=0.0)
        self.z_var = tk.DoubleVar(value=0.15)
        self.port_var = tk.StringVar()
        
        # Initialize robot arm and ESP32 interface
        self.robot = RobotArmIKPy()
        self.esp32 = None
        
        # Track current positions
        self.current_joint_angles = [0, 0, 0, 0, 0, 0]
        self.current_angles_display = [tk.StringVar() for _ in range(6)]
        self.current_dynamixel_display = [tk.StringVar() for _ in range(6)]
        
        # Setup the main interface
        self.create_widgets()
        
        # Initialize the visualization
        self.init_visualization()
        
        # Update the plot with initial position
        self.update_visualization()
    
    def create_widgets(self):
        """Create all the GUI widgets."""
        # Create the main frame with two columns
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Left column for controls
        control_frame = ttk.LabelFrame(main_frame, text="Robot Control", padding="10")
        control_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5, pady=5)
        
        # Right column for visualization
        viz_frame = ttk.LabelFrame(main_frame, text="Robot Visualization", padding="10")
        viz_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # --- Control Frame ---
        # Target position section
        pos_frame = ttk.LabelFrame(control_frame, text="Target Position (meters)", padding="10")
        pos_frame.pack(fill=tk.X, pady=5)
        
        # X position
        ttk.Label(pos_frame, text="X:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(pos_frame, textvariable=self.x_var, width=10).grid(row=0, column=1, padx=5, pady=5)
        
        # Y position
        ttk.Label(pos_frame, text="Y:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(pos_frame, textvariable=self.y_var, width=10).grid(row=1, column=1, padx=5, pady=5)
        
        # Z position
        ttk.Label(pos_frame, text="Z:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(pos_frame, textvariable=self.z_var, width=10).grid(row=2, column=1, padx=5, pady=5)
        
        # Create a preset positions section
        preset_frame = ttk.LabelFrame(control_frame, text="Preset Positions", padding="10")
        preset_frame.pack(fill=tk.X, pady=5)
        
        # Add preset position buttons
        ttk.Button(preset_frame, text="Home", command=lambda: self.set_preset_position(0.15, 0.0, 0.2)).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(preset_frame, text="Forward", command=lambda: self.set_preset_position(0.25, 0.0, 0.1)).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(preset_frame, text="Side", command=lambda: self.set_preset_position(0.1, 0.15, 0.15)).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(preset_frame, text="Low", command=lambda: self.set_preset_position(0.2, 0.0, 0.05)).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(preset_frame, text="High", command=lambda: self.set_preset_position(0.1, 0.0, 0.3)).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(preset_frame, text="Center", command=self.center_all_servos).grid(row=1, column=2, padx=5, pady=5)
        
        # ESP32 Connection
        conn_frame = ttk.LabelFrame(control_frame, text="ESP32 Connection", padding="10")
        conn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        
        # Port selection dropdown
        port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        port_combo.grid(row=0, column=1, padx=5, pady=5)
        
        # Populate with available ports
        try:
            import serial.tools.list_ports
            ports = [port.device for port in serial.tools.list_ports.comports()]
            if ports:
                port_combo['values'] = ports
                port_combo.current(0)  # Select first port
        except Exception as e:
            print(f"Error getting serial ports: {e}")
        
        # Connect/Disconnect Button
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5, pady=5)
        
        # Action buttons
        action_frame = ttk.LabelFrame(control_frame, text="Actions", padding="10")
        action_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(action_frame, text="Calculate IK", command=self.calculate_ik).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(action_frame, text="Move Robot", command=self.move_robot).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(action_frame, text="Read Positions", command=self.read_positions).grid(row=0, column=2, padx=5, pady=5)
        
        # Joint angles display
        angles_frame = ttk.LabelFrame(control_frame, text="Joint Angles (radians)", padding="10")
        angles_frame.pack(fill=tk.X, pady=5)
        
        for i in range(6):
            ttk.Label(angles_frame, text=f"Joint {i+1}:").grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            ttk.Label(angles_frame, textvariable=self.current_angles_display[i], width=10).grid(row=i, column=1, padx=5, pady=2)
        
        # Dynamixel positions display
        dyn_frame = ttk.LabelFrame(control_frame, text="Dynamixel Positions", padding="10")
        dyn_frame.pack(fill=tk.X, pady=5)
        
        for i in range(6):
            ttk.Label(dyn_frame, text=f"Joint {i+1}:").grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            ttk.Label(dyn_frame, textvariable=self.current_dynamixel_display[i], width=10).grid(row=i, column=1, padx=5, pady=2)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_frame = ttk.Frame(control_frame)
        status_frame.pack(fill=tk.X, pady=10)
        ttk.Label(status_frame, text="Status:").pack(side=tk.LEFT)
        ttk.Label(status_frame, textvariable=self.status_var).pack(side=tk.LEFT, padx=5)
        
        # --- Visualization Frame ---
        self.viz_container = ttk.Frame(viz_frame)
        self.viz_container.pack(fill=tk.BOTH, expand=True)
    
    def init_visualization(self):
        """Initialize the 3D visualization of the robot arm."""
        # Create a figure for plotting
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Embed the figure in the tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.viz_container)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add the matplotlib toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.viz_container)
        self.toolbar.update()
    
    def update_visualization(self):
        """Update the 3D visualization with current joint angles."""
        # Clear the axes
        self.ax.clear()
        
        # Get target position
        target_position = [self.x_var.get(), self.y_var.get(), self.z_var.get()]
        
        # Plot the robot arm
        try:
            self.robot.plot_arm(self.current_joint_angles, target_position, self.ax)
            
            # Set labels and title
            self.ax.set_xlabel('X (m)')
            self.ax.set_ylabel('Y (m)')
            self.ax.set_zlabel('Z (m)')
            self.ax.set_title('6-Axis Robot Arm')
            
            # Set equal aspect ratio
            limits = []
            for dim in [self.ax.get_xlim(), self.ax.get_ylim(), self.ax.get_zlim()]:
                limits.append(max(abs(dim[0]), abs(dim[1])))
            limit = max(limits) * 1.1  # Add 10% margin
            self.ax.set_xlim(-limit, limit)
            self.ax.set_ylim(-limit, limit)
            self.ax.set_zlim(-limit, limit)
            
            # Add grid
            self.ax.grid(True)
            
            # Set reasonable viewpoint
            self.ax.view_init(elev=30, azim=45)
            
            # Draw the canvas
            self.canvas.draw()
        except Exception as e:
            messagebox.showerror("Visualization Error", f"Error updating visualization: {e}")
            print(f"Error in visualization: {e}")
    
    def set_preset_position(self, x, y, z):
        """Set a preset position and update the visualization."""
        self.x_var.set(x)
        self.y_var.set(y)
        self.z_var.set(z)
        self.calculate_ik()
    
    def center_all_servos(self):
        """Center all servos (set to neutral position)."""
        # Set all joint angles to 0 (centered)
        self.current_joint_angles = [0, 0, 0, 0, 0, 0]
        
        # Calculate the corresponding Dynamixel positions
        dynamixel_positions = self.robot.angles_to_dynamixel(self.current_joint_angles)
        
        # Update displays
        self.update_angle_displays()
        
        # Update visualization
        self.update_visualization()
        
        # Send to ESP32 if connected
        if self.esp32 and self.esp32.connected:
            success = self.esp32.center_all_servos()
            if success:
                self.status_var.set("Servos centered successfully")
            else:
                self.status_var.set("Failed to center servos")
        else:
            self.status_var.set("Servos centered (visualization only)")
    
    def toggle_connection(self):
        """Connect or disconnect from the ESP32."""
        if self.esp32 and self.esp32.connected:
            # Disconnect
            self.esp32.disconnect()
            self.esp32 = None
            self.connect_btn.config(text="Connect")
            self.status_var.set("Disconnected from ESP32")
        else:
            # Connect
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Connection Error", "Please select a serial port")
                return
            
            try:
                self.esp32 = ESP32RobotInterface(port=port)
                if self.esp32.connect():
                    self.connect_btn.config(text="Disconnect")
                    self.status_var.set(f"Connected to ESP32 on {port}")
                else:
                    self.esp32 = None
                    messagebox.showerror("Connection Error", f"Failed to connect to ESP32 on {port}")
            except Exception as e:
                self.esp32 = None
                messagebox.showerror("Connection Error", f"Error connecting to ESP32: {e}")
    
    def calculate_ik(self):
        """Calculate inverse kinematics for the target position."""
        # Get target position
        target_position = [self.x_var.get(), self.y_var.get(), self.z_var.get()]
        
        try:
            # Calculate joint angles using inverse kinematics
            self.status_var.set("Calculating inverse kinematics...")
            self.master.update_idletasks()
            
            joint_angles = self.robot.inverse_kinematics(target_position)
            self.current_joint_angles = joint_angles
            
            # Calculate the corresponding Dynamixel positions
            dynamixel_positions = self.robot.angles_to_dynamixel(joint_angles)
            
            # Update displays
            self.update_angle_displays()
            
            # Update visualization
            self.update_visualization()
            
            # Check if target is reachable by calculating forward kinematics
            actual_position, _, _ = self.robot.forward_kinematics(joint_angles)
            error = np.linalg.norm(np.array(target_position) - actual_position)
            
            if error < 0.02:  # Less than 2cm error
                self.status_var.set(f"IK calculated successfully (error: {error:.4f}m)")
            else:
                self.status_var.set(f"Target not fully reachable (error: {error:.4f}m)")
            
        except Exception as e:
            messagebox.showerror("IK Error", f"Error calculating inverse kinematics: {e}")
            self.status_var.set(f"IK calculation failed: {str(e)}")
    
    def move_robot(self):
        """Send the calculated joint angles to the robot."""
        if not self.esp32 or not self.esp32.connected:
            messagebox.showwarning("Not Connected", "Please connect to the ESP32 first")
            return
        
        try:
            # Convert joint angles to Dynamixel positions
            dynamixel_positions = self.robot.angles_to_dynamixel(self.current_joint_angles)
            
            # Send to ESP32
            self.status_var.set("Sending positions to ESP32...")
            self.master.update_idletasks()
            
            success = self.esp32.set_all_servo_positions(dynamixel_positions)
            
            if success:
                self.status_var.set("Robot moved successfully")
            else:
                self.status_var.set("Failed to move robot")
                messagebox.showerror("Movement Error", "Failed to send positions to ESP32")
            
        except Exception as e:
            messagebox.showerror("Movement Error", f"Error moving robot: {e}")
            self.status_var.set(f"Movement failed: {str(e)}")
    
    def read_positions(self):
        """Read current servo positions from the ESP32."""
        if not self.esp32 or not self.esp32.connected:
            messagebox.showwarning("Not Connected", "Please connect to the ESP32 first")
            return
        
        try:
            # Read positions from ESP32
            self.status_var.set("Reading positions from ESP32...")
            self.master.update_idletasks()
            
            dynamixel_positions = self.esp32.read_all_servo_positions()
            
            if dynamixel_positions:
                # Convert to joint angles
                joint_angles = self.robot.dynamixel_to_angles(dynamixel_positions)
                self.current_joint_angles = joint_angles
                
                # Update displays
                self.update_angle_displays()
                
                # Update visualization
                self.update_visualization()
                
                self.status_var.set("Positions read successfully")
            else:
                self.status_var.set("Failed to read positions")
                messagebox.showerror("Read Error", "Failed to read positions from ESP32")
            
        except Exception as e:
            messagebox.showerror("Read Error", f"Error reading positions: {e}")
            self.status_var.set(f"Read failed: {str(e)}")
    
    def update_angle_displays(self):
        """Update the displayed joint angles and Dynamixel positions."""
        joint_angles = self.current_joint_angles
        dynamixel_positions = self.robot.angles_to_dynamixel(joint_angles)
        
        for i in range(6):
            self.current_angles_display[i].set(f"{joint_angles[i]:.4f}")
            self.current_dynamixel_display[i].set(f"{dynamixel_positions[i]}")

def main():
    """Main function to start the GUI."""
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main() 