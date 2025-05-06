#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Inverse Kinematics for 6-Axis Robot Arm using IKPy library
- First 3 joints: Dynamixel XL430-W250 motors
- Last 3 joints: Dynamixel XL330 motors

This script uses the IKPy library to calculate inverse kinematics
more efficiently and accurately.
"""

import numpy as np
from math import pi, sin, cos, degrees, radians
import argparse
import time
import ikpy.chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Robot‑arm geometry — update with your real measurements  (all in metres)
BASE_HEIGHT        = 0.04   # Base to joint‑1 axis  (Z)
LINK_1_2_LENGTH    = 0.02   # Joint‑1 → joint‑2  offset
LINK_2_3_LENGTH    = 0.15   # Joint‑2 → joint‑3  (upper arm length)
LINK_3_4_LENGTH    = 0.15   # Joint‑3 → joint‑4  (fore‑arm length)
LINK_4_5_LENGTH    = 0.05   # Joint‑4 → joint‑5  (wrist yaw → wrist pitch)
LINK_5_6_LENGTH    = 0.00   # Joint‑5 → gripper pivot (inside shell) – no offset
END_EFFECTOR_LENGTH = 0.05  # Gripper pivot → TCP (camera lens / finger tips)

# Custom Dynamixel limits  [min_pos, max_pos, min_deg, max_deg]
JOINT_1_LIMITS = [1027, 3096,  90, 270]   # base yaw
JOINT_2_LIMITS = [ 140, 2806, 140, 270]   # shoulder pitch
JOINT_3_LIMITS = [1024, 3000, 180, 270]   # elbow pitch
JOINT_4_LIMITS = [1523, 2476,  90, 270]   # wrist yaw (roll)
JOINT_5_LIMITS = [1830, 2200,   0,  45]   # wrist pitch
JOINT_6_LIMITS = [2000, 2634,   0,  90]   # gripper (NOT part of IK)

from math import pi
import ikpy
from ikpy.chain import Chain
from ikpy.link  import URDFLink, OriginLink

class RobotArmIKPy:
    def __init__(self):
        # ------------------ build kinematic chain ------------------
        # Joint‑6 (gripper) is kept as a *fixed* link (rotation=None)
        # so IKPy solves only the 5 positioning joints.
        self.chain = Chain(
            name="robot_arm",
            links=[
                OriginLink(),  # World / base frame
                URDFLink(
                    name="joint1",                                  # base yaw (Z)
                    origin_translation=[0, 0, BASE_HEIGHT],
                    origin_orientation=[0, 0, 0],
                    use_symbolic_matrix=True,
                    joint_type="revolute",
                    rotation=[0, 0, 1],
                    bounds=(-pi/2, pi/2)  # Add joint limits
                ),
                URDFLink(
                    name="joint2",                                  # shoulder pitch (Y)
                    origin_translation=[0, 0, LINK_1_2_LENGTH],
                    origin_orientation=[0, 0, 0],
                    use_symbolic_matrix=True,
                    joint_type="revolute",
                    rotation=[0, 1, 0],
                    bounds=(-pi/2, 0)  # Add joint limits
                ),
                URDFLink(
                    name="joint3",                                  # elbow pitch (Y)
                    origin_translation=[LINK_2_3_LENGTH, 0, 0],
                    origin_orientation=[0, 0, 0],
                    use_symbolic_matrix=True,
                    joint_type="revolute",
                    rotation=[0, 1, 0],
                    bounds=(0, pi/2)  # Add joint limits
                ),
                URDFLink(
                    name="joint4",                                  # wrist yaw / roll (Z)
                    origin_translation=[LINK_3_4_LENGTH, 0, 0],
                    origin_orientation=[0, 0, 0],
                    use_symbolic_matrix=True,
                    joint_type="revolute",
                    rotation=[0, 0, 1],
                    bounds=(0, pi/2)  # Add joint limits
                ),
                URDFLink(
                    name="joint5",                                  # wrist pitch (Y)
                    origin_translation=[0, 0, LINK_4_5_LENGTH],
                    origin_orientation=[0, 0, 0],
                    use_symbolic_matrix=True,
                    joint_type="revolute",
                    rotation=[0, 1, 0],
                    bounds=(-pi/4, pi/4)  # Add joint limits
                ),
                URDFLink(                                           # gripper pivot – NO DoF
                    name="joint6",
                    origin_translation=[0, 0, LINK_5_6_LENGTH],
                    origin_orientation=[0, 0, 0],
                    use_symbolic_matrix=True,
                    joint_type="fixed",
                    rotation=None,            # <- fixed link, removed from IK DoF
                    translation=None
                ),
                URDFLink(                                           # TCP
                    name="end_effector",
                    origin_translation=[0, 0, END_EFFECTOR_LENGTH],
                    origin_orientation=[0, 0, 0],
                    use_symbolic_matrix=True,
                    joint_type="fixed",
                    rotation=None,       # fixed link
                    translation=None
                )
            ]
        )

        # ------------------ IKPy joint‑angle limits -----------------
        # Angles are expressed in *radians* and centred around the
        # mechanical zero you defined above.
        self.joint_limits = [
            [-pi/2,  pi/2],    # J1:  90° → 270°
            [-pi/2,     0],    # J2: 140° → 270°
            [    0,  pi/2],    # J3: 180° → 270°
            [    0,  pi/2],    # J4:  90° → 180°
            [-pi/4, pi/4],     # J5:   0° →  45°
            [-pi/2, pi/2]      # J6 (gripper) – kept for completeness, not used by IK
        ]

        # Save Dynamixel ranges for later degree/step conversion
        self.joint_dxl_limits = [
            JOINT_1_LIMITS,
            JOINT_2_LIMITS,
            JOINT_3_LIMITS,
            JOINT_4_LIMITS,
            JOINT_5_LIMITS,
            JOINT_6_LIMITS
        ]

        # Dynamixel "center" (depends on model firmware; 2048 for XL‑series at 12‑bit)
        self.center_position = 2048

        
    def forward_kinematics(self, joint_angles):
        """Calculate the end-effector position given joint angles."""
        # Make sure we have the right number of angles for all links
        full_angles = [0]  # First angle is always 0 for the base link
        
        # Add the provided joint angles
        for angle in joint_angles:
            full_angles.append(angle)
        
        # Ensure full_angles has the correct length (should match the number of links)
        if len(full_angles) != len(self.chain.links):
            if len(full_angles) < len(self.chain.links):
                # Add zeros for any missing angles
                full_angles.extend([0] * (len(self.chain.links) - len(full_angles)))
            else:
                # Truncate if we have too many angles
                full_angles = full_angles[:len(self.chain.links)]
        
        try:
            # Calculate the transformation matrix
            transformation_matrix = self.chain.forward_kinematics(full_angles)
            
            # Extract position and rotation
            position = transformation_matrix[0:3, 3]
            rotation = transformation_matrix[0:3, 0:3]
            
            return position, rotation, transformation_matrix
        except Exception as e:
            print(f"Error in forward kinematics calculation: {e}")
            # Return a default position as fallback
            return np.array([0, 0, 0]), np.eye(3), np.eye(4)

    def inverse_kinematics(self, target_position, target_orientation=None, initial_position=None):
        """
        Calculate joint angles to reach the target position.
        
        Args:
            target_position: [x, y, z] coordinates
            target_orientation: Optional 3x3 rotation matrix
            initial_position: Optional initial guess for joint angles
            
        Returns:
            joint_angles: List of 6 joint angles in radians
        """
        # Prepare initial position if provided
        if initial_position is None:
            # Create initial position for 6 joints (one angle per active joint)
            initial_position = [0, 0, 0, 0, 0, 0]
        
        # Add 0 at the beginning for the base link (fixed) and make sure we have enough angles
        initial_full = [0] + list(initial_position)
        
        # Check if we need to add an extra 0 for the end effector (which is a fixed link)
        if len(initial_full) < len(self.chain.links):
            initial_full.append(0)
        
        # Make sure the length matches exactly
        if len(initial_full) != len(self.chain.links):
            # Adjust length to match the chain's links
            if len(initial_full) < len(self.chain.links):
                initial_full.extend([0] * (len(self.chain.links) - len(initial_full)))
            else:
                initial_full = initial_full[:len(self.chain.links)]
        
        # Prepare target matrix
        if target_orientation is not None:
            # Create the full 4x4 transformation matrix
            target_matrix = np.eye(4)
            target_matrix[0:3, 0:3] = target_orientation
            target_matrix[0:3, 3] = target_position
        else:
            target_matrix = None
        
        try:
            # Calculate inverse kinematics
            if target_matrix is not None:
                # With orientation constraint
                joint_angles = self.chain.inverse_kinematics(
                    target_matrix,
                    initial_position=initial_full,
                    orientation_mode="all"
                )
            else:
                # Position only
                joint_angles = self.chain.inverse_kinematics(
                    target_position,
                    initial_position=initial_full,
                    orientation_mode=None
                )
            
            # Remove the first angle (base link) and the last angle (end effector) if present
            joint_angles = joint_angles[1:7]  # Keep only the 6 joint angles
            
            # Apply joint limits
            for i in range(min(6, len(joint_angles))):
                joint_angles[i] = max(self.joint_limits[i][0], min(self.joint_limits[i][1], joint_angles[i]))
            
            # Ensure we return exactly 6 angles
            if len(joint_angles) < 6:
                joint_angles = list(joint_angles) + [0] * (6 - len(joint_angles))
            elif len(joint_angles) > 6:
                joint_angles = joint_angles[:6]
            
            # Print both raw angles and Dynamixel positions
            print("\nRaw joint angles (IKPy coordinates):")
            for i, angle in enumerate(joint_angles):
                print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f}°)")
            
            # Calculate and print Dynamixel positions
            dxl_positions = self.angles_to_dynamixel(joint_angles)
            
            return joint_angles
        
        except Exception as e:
            print(f"Error in inverse kinematics calculation: {e}")
            print("Returning neutral position as fallback")
            return [0, 0, 0, 0, 0, 0]

    def angles_to_dynamixel(self, joint_angles):
        """Convert joint angles in radians to Dynamixel position values."""
        dynamixel_positions = []
        
        print("\nDynamixel angles (radians):")
        for i, angle in enumerate(joint_angles):
            # Get the limits for this joint
            min_pos, max_pos, min_deg, max_deg = self.joint_dxl_limits[i]
            
            # Convert angle to degrees
            angle_deg = degrees(angle)
            
            # Transform angles to match Dynamixel coordinate system
            if i == 0:  # Base
                # IKPy: -90° to 90° -> Dynamixel: 90° to 270°
                angle_deg = 180 + angle_deg  # Shift to match Dynamixel range
            elif i == 1:  # Shoulder
                # IKPy: -90° to 90° -> Dynamixel: 140° to 270°
                # Negative IKPy angle means bending backward
                angle_deg = 270 + angle_deg  # Shift to match Dynamixel range
            elif i == 2:  # Elbow
                # IKPy: -90° to 90° -> Dynamixel: 180° to 270°
                # Negative IKPy angle means bending backward
                angle_deg = 180 + angle_deg  # Shift to match Dynamixel range
            elif i == 3:  # Wrist rotation
                # IKPy: -90° to 90° -> Dynamixel: 90° to 270°
                # 0° in IKPy should be 90° in Dynamixel (pointing up)
                angle_deg = 90 + angle_deg  # Shift to match Dynamixel range
            elif i == 4:  # Wrist pitch
                # IKPy: -45° to 45° -> Dynamixel: 0° to 45°
                angle_deg = angle_deg  # Already in correct range
            elif i == 5:  # Gripper
                # IKPy: -90° to 90° -> Dynamixel: 0° to 90°
                angle_deg = angle_deg  # Already in correct range
            
            # Ensure angle is within Dynamixel range
            if angle_deg < 0:
                angle_deg += 360
            elif angle_deg >= 360:
                angle_deg -= 360
            
            # Convert back to radians for printing
            angle_rad = radians(angle_deg)
            print(f"Joint {i+1}: {angle_rad:.4f} rad ({angle_deg:.2f}°)")
            
            # Map angle to Dynamixel position
            pos_range = max_pos - min_pos
            angle_range = max_deg - min_deg
            position = min_pos + int((angle_deg - min_deg) * pos_range / angle_range)
            
            # Ensure position is within limits
            position = max(min_pos, min(max_pos, position))
            dynamixel_positions.append(position)
        
        return dynamixel_positions

    def dynamixel_to_angles(self, dynamixel_positions):
        """Convert Dynamixel position values to joint angles in radians using actual motor limits."""
        joint_angles = []
        
        for i, position in enumerate(dynamixel_positions):
            # Get the limits for this joint
            min_pos, max_pos, min_deg, max_deg = self.joint_dxl_limits[i]
            
            # Handle reversed range for joint 6 (gripper)
            if i == 5 and min_deg > max_deg:
                temp = min_deg
                min_deg = max_deg
                max_deg = temp
                temp = min_pos
                min_pos = max_pos
                max_pos = temp
            
            # Map the position to angle in degrees
            if max_pos == min_pos:
                angle_deg = min_deg  # Prevent division by zero
            else:
                angle_deg = min_deg + (position - min_pos) * (max_deg - min_deg) / (max_pos - min_pos)
            
            # Convert from actual angle range to the range expected by IKPy
            if i == 0 or i == 1 or i == 2:
                angle_deg -= 90  # Convert from [0, 180] to [-90, 90]
            elif i == 3:
                angle_deg -= 45  # Convert from [0, 90] to [-45, 45]
            elif i == 4:
                angle_deg -= 22.5  # Convert from [0, 45] to [-22.5, 22.5]
            elif i == 5:
                angle_deg -= 45  # Convert from [0, 90] to [-45, 45]
            
            # Convert to radians
            angle_rad = radians(angle_deg)
            
            joint_angles.append(angle_rad)
        
        return joint_angles

    def plot_arm(self, joint_angles, target_position=None):
        """Visualize the robot arm configuration."""
        # Make sure we have the right number of angles for the chain
        full_angles = [0]  # First angle is for the base link (fixed)
        
        # Add the joint angles
        for angle in joint_angles:
            full_angles.append(angle)
        
        # Ensure we have the correct number of angles
        if len(full_angles) < len(self.chain.links):
            full_angles.extend([0] * (len(self.chain.links) - len(full_angles)))
        elif len(full_angles) > len(self.chain.links):
            full_angles = full_angles[:len(self.chain.links)]
        
        # Create the figure
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        try:
            # Plot the arm
            self.chain.plot(full_angles, ax)
            
            # Plot target position if provided
            if target_position is not None:
                ax.scatter(target_position[0], target_position[1], target_position[2], 
                         color='red', s=100, label='Target')
                ax.legend()
            
            # Calculate the forward kinematics to get end-effector position
            position, _, _ = self.forward_kinematics(joint_angles)
            
            # Plot end-effector
            ax.scatter(position[0], position[1], position[2],
                      color='green', s=80, label='End-effector')
            
            # Set labels and title
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title('6-Axis Robot Arm')
            
            # Set equal aspect ratio
            limits = []
            for dim in [ax.get_xlim(), ax.get_ylim(), ax.get_zlim()]:
                limits.append(max(abs(dim[0]), abs(dim[1])))
            limit = max(limits) * 1.1  # Add 10% margin
            ax.set_xlim(-limit, limit)
            ax.set_ylim(-limit, limit)
            ax.set_zlim(-limit, limit)
            
            # Add grid
            ax.grid(True)
            
            # Set reasonable viewpoint
            ax.view_init(elev=30, azim=45)
            
            # Print the raw angles for debugging
            print("\nRaw joint angles (radians):")
            for i, angle in enumerate(joint_angles):
                print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f}°)")
            
        except Exception as e:
            print(f"Error plotting arm: {e}")
            # Add a basic representation if plotting fails
            ax.text(0, 0, 0, "Plotting error", color='red')
        
        return fig, ax

    def invert_servo3_position(self, position, servo_index=2):
        """
        Inverts the position of Servo 3 (or specified servo) relative to its center.
        
        Args:
            position: The original position value
            servo_index: The index of the servo to invert (default is 2 for Servo 3)
            
        Returns:
            The inverted position value
        """
        # Calculate the center position
        center = (self.joint_dxl_limits[servo_index][0] + self.joint_dxl_limits[servo_index][1]) / 2
        # Invert the position relative to center
        inverted_position = int(center * 2 - position)
        return inverted_position

def main():
    parser = argparse.ArgumentParser(description='Inverse Kinematics for 6-axis robot arm using IKPy')
    parser.add_argument('--mode', choices=['fk', 'ik', 'test'], default='ik',
                        help='Mode: fk (forward kinematics), ik (inverse kinematics), or test (test each joint)')
    parser.add_argument('--angles', type=float, nargs=6, 
                        help='Six joint angles in radians (for forward kinematics)')
    parser.add_argument('--position', type=float, nargs=3,
                        help='Target position [x, y, z] (for inverse kinematics)')
    parser.add_argument('--joint', type=int, choices=[1, 2, 3, 4, 5, 6],
                        help='Joint number to test (1-6, for test mode)')
    parser.add_argument('--value', type=int,
                        help='Dynamixel position value to test (0-4095, for test mode)')
    parser.add_argument('--dynamixel', action='store_true', 
                        help='Input angles are in Dynamixel units instead of radians')
    parser.add_argument('--visualize', action='store_true', 
                        help='Visualize the robot arm')
    parser.add_argument('--limits', action='store_true',
                        help='Show the joint limits')
    # Add ESP32 communication options
    parser.add_argument('--send-to-esp32', action='store_true',
                        help='Send the calculated positions to the ESP32')
    parser.add_argument('--port', type=str,
                        help='Serial port for ESP32 (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Baud rate for ESP32 communication (default: 115200)')
    
    args = parser.parse_args()
    
    try:
        # Create robot arm instance
        robot = RobotArmIKPy()
        
        # Show joint limits if requested
        if args.limits:
            print("\nJoint Limits (Dynamixel position and angle ranges):")
            for i, limits in enumerate(robot.joint_dxl_limits):
                min_pos, max_pos, min_deg, max_deg = limits
                print(f"Joint {i+1}: Position {min_pos}-{max_pos}, Angle {min_deg}°-{max_deg}°")
            
            print("\nJoint Limits (in radians for IKPy):")
            for i, (min_rad, max_rad) in enumerate(robot.joint_limits):
                print(f"Joint {i+1}: {min_rad:.4f} to {max_rad:.4f} rad ({degrees(min_rad):.2f}° to {degrees(max_rad):.2f}°)")
            
            return
        
        # Test mode for individual joint calibration
        if args.mode == 'test':
            if args.joint is None or args.value is None:
                print("Error: Joint number (--joint) and position value (--value) are required for test mode")
                print("Example: --mode test --joint 1 --value 2048")
                return
            
            joint_idx = args.joint - 1  # Convert to 0-based index
            position = args.value
            
            # Create a neutral position array
            positions = [robot.center_position] * 6
            
            # Set the specified joint position
            positions[joint_idx] = position
            
            # Convert to angles
            joint_angles = robot.dynamixel_to_angles(positions)
            
            # Print the joint angles
            print(f"\nTesting Joint {args.joint} at position {position}")
            print(f"Converted angle: {joint_angles[joint_idx]:.4f} rad ({degrees(joint_angles[joint_idx]):.2f}°)")
            
            # Print all joint values
            print("\nAll joint positions (Dynamixel units):")
            for i, pos in enumerate(positions):
                print(f"Joint {i+1}: {pos}")
            
            # Print all joint angles
            print("\nAll joint angles (radians):")
            for i, angle in enumerate(joint_angles):
                print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f}°)")
            
            # Calculate forward kinematics
            position, rotation, _ = robot.forward_kinematics(joint_angles)
            
            # Print end-effector position
            print("\nEnd-effector position:")
            print(f"X: {position[0]:.4f}")
            print(f"Y: {position[1]:.4f}")
            print(f"Z: {position[2]:.4f}")
            
            # Visualize if requested
            if args.visualize:
                try:
                    robot.plot_arm(joint_angles)
                    plt.tight_layout()
                    plt.show()
                except Exception as e:
                    print(f"Error in visualization: {e}")
            
            # Send to ESP32 if requested
            if args.send_to_esp32:
                try:
                    from esp32_robot_interface import ESP32RobotInterface
                    
                    # Create the interface
                    interface = ESP32RobotInterface(port=args.port, baudrate=args.baudrate)
                    
                    # Connect to ESP32
                    if interface.connect():
                        # Set the servo positions
                        success = interface.set_all_servo_positions(positions)
                        print(f"Sending positions to ESP32: {'Success' if success else 'Failed'}")
                        
                        # Disconnect from ESP32
                        interface.disconnect()
                    else:
                        print("Failed to connect to ESP32.")
                except ImportError:
                    print("Error: esp32_robot_interface.py not found. Make sure it's in the same directory.")
                except Exception as e:
                    print(f"Error communicating with ESP32: {e}")
            
            return
        
        # Forward Kinematics mode
        if args.mode == 'fk':
            if args.angles is None:
                print("Error: Joint angles are required for forward kinematics mode")
                return
            
            # Get joint angles
            if args.dynamixel:
                dynamixel_positions = args.angles
                joint_angles = robot.dynamixel_to_angles(dynamixel_positions)
                print("\nDynamixel position values:")
                for i, position in enumerate(dynamixel_positions):
                    print(f"Joint {i+1}: {position}")
            else:
                joint_angles = args.angles
                dynamixel_positions = robot.angles_to_dynamixel(joint_angles)
            
            # Print joint angles
            print("\nJoint angles (radians):")
            for i, angle in enumerate(joint_angles):
                print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f} degrees)")
            
            # If not provided as input, print Dynamixel positions
            if not args.dynamixel:
                print("\nDynamixel position values:")
                for i, position in enumerate(dynamixel_positions):
                    print(f"Joint {i+1}: {position}")
            
            # Calculate forward kinematics
            position, rotation, _ = robot.forward_kinematics(joint_angles)
            
            # Print end-effector position
            print("\nEnd-effector position:")
            print(f"X: {position[0]:.4f}")
            print(f"Y: {position[1]:.4f}")
            print(f"Z: {position[2]:.4f}")
            
            # Print rotation matrix
            print("\nEnd-effector orientation (rotation matrix):")
            for i in range(3):
                print(f"[{rotation[i,0]:.4f}, {rotation[i,1]:.4f}, {rotation[i,2]:.4f}]")
            
            # Send to ESP32 if requested
            if args.send_to_esp32:
                try:
                    from esp32_robot_interface import ESP32RobotInterface
                    
                    # Create the interface
                    interface = ESP32RobotInterface(port=args.port, baudrate=args.baudrate)
                    
                    # Connect to ESP32
                    if interface.connect():
                        # Set the servo positions
                        success = interface.set_all_servo_positions(dynamixel_positions)
                        print(f"Sending positions to ESP32: {'Success' if success else 'Failed'}")
                        
                        # Disconnect from ESP32
                        interface.disconnect()
                    else:
                        print("Failed to connect to ESP32.")
                except ImportError:
                    print("Error: esp32_robot_interface.py not found. Make sure it's in the same directory.")
                except Exception as e:
                    print(f"Error communicating with ESP32: {e}")
        
        # Inverse Kinematics mode
        else:
            if args.position is None:
                print("Error: Target position is required for inverse kinematics mode")
                return
            
            # Get target position
            target_position = args.position
            print(f"\nTarget position: [{target_position[0]:.4f}, {target_position[1]:.4f}, {target_position[2]:.4f}]")
            
            # Calculate inverse kinematics
            start_time = time.time()
            joint_angles = robot.inverse_kinematics(target_position)
            end_time = time.time()
            
            # Calculate dynamixel positions
            dynamixel_positions = robot.angles_to_dynamixel(joint_angles)
            
            # Print joint angles
            print("\nCalculated joint angles (radians):")
            for i, angle in enumerate(joint_angles):
                print(f"Joint {i+1}: {angle:.4f} rad ({degrees(angle):.2f} degrees)")
            
            # Print Dynamixel positions
            print("\nDynamixel position values:")
            for i, position in enumerate(dynamixel_positions):
                print(f"Joint {i+1}: {position}")
            
            try:
                # Calculate the actual position achieved with these joint angles
                actual_position, _, _ = robot.forward_kinematics(joint_angles)
                
                # Print actual position
                print("\nActual position achieved:")
                print(f"X: {actual_position[0]:.4f}")
                print(f"Y: {actual_position[1]:.4f}")
                print(f"Z: {actual_position[2]:.4f}")
                
                # Calculate error
                error = np.linalg.norm(np.array(target_position) - actual_position)
                print(f"\nPosition error: {error:.4f}")
            except Exception as e:
                print(f"\nError calculating actual position: {e}")
                
            print(f"Calculation time: {(end_time - start_time)*1000:.2f} ms")
            
            # Send to ESP32 if requested
            if args.send_to_esp32:
                try:
                    from esp32_robot_interface import ESP32RobotInterface
                    
                    # Create the interface
                    interface = ESP32RobotInterface(port=args.port, baudrate=args.baudrate)
                    
                    # Connect to ESP32
                    if interface.connect():
                        # Set the servo positions
                        success = interface.set_all_servo_positions(dynamixel_positions)
                        print(f"Sending positions to ESP32: {'Success' if success else 'Failed'}")
                        
                        # Disconnect from ESP32
                        interface.disconnect()
                    else:
                        print("Failed to connect to ESP32.")
                except ImportError:
                    print("Error: esp32_robot_interface.py not found. Make sure it's in the same directory.")
                except Exception as e:
                    print(f"Error communicating with ESP32: {e}")
        
        # Visualize if requested
        if args.visualize:
            try:
                if args.mode == 'fk':
                    robot.plot_arm(joint_angles)
                else:
                    robot.plot_arm(joint_angles, target_position)
                plt.tight_layout()
                plt.show()
            except Exception as e:
                print(f"Error in visualization: {e}")
                print("Try running without --visualize option")
                
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 