#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import GripperCommand, JointTrajectoryControllerState

import sys
import select
import termios
import tty
import time

MSG = """
Control Your Universal Robot Arm!
---------------------------
Joint Controls:
  q/a : shoulder_pan_joint +/-
  w/s : shoulder_lift_joint +/-
  e/d : elbow_joint +/-
  r/f : wrist_1_joint +/-
  t/g : wrist_2_joint +/-
  y/h : wrist_3_joint +/-
  o/p : gripper open/close

  CTRL-C to quit
"""

# Key mapping to joint indices
JOINT_KEYS = {
    'q': [0, 0.1],   # shoulder_pan_joint +
    'a': [0, -0.1],  # shoulder_pan_joint -
    'w': [1, 0.1],   # shoulder_lift_joint +
    's': [1, -0.1],  # shoulder_lift_joint -
    'e': [2, 0.1],   # elbow_joint +
    'd': [2, -0.1],  # elbow_joint -
    'r': [3, 0.1],   # wrist_1_joint +
    'f': [3, -0.1],  # wrist_1_joint -
    't': [4, 0.1],   # wrist_2_joint +
    'g': [4, -0.1],  # wrist_2_joint -
    'y': [5, 0.1],   # wrist_3_joint +
    'h': [5, -0.1],  # wrist_3_joint -
}

GRIPPER_KEYS = {
    'o': 0.0,  # open
    'p': 0.8,  # close
}

class KeyboardCommandProcessor(Node):
    def __init__(self):
        super().__init__('keyboard_command_processor')
        
        # Create publishers for joint trajectory commands
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
            
        # Create publisher for gripper commands
        self.gripper_pub = self.create_publisher(
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            10)
            
        # Joint names for the UR arm
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Current joint positions - we'll read these from feedback
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Movement increment (radians)
        self.movement_step = 0.05  # Reduced for more precise control
        
        # Gripper state
        self.gripper_position = 0.0
        
        # Subscribe to joint states to get current positions
        self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.joint_state_callback,
            10)
        
        self.get_logger().info('Keyboard command service is running!')
        self.get_logger().info('Joint controls:')
        self.get_logger().info('q/a : shoulder_pan_joint +/-')
        self.get_logger().info('w/s : shoulder_lift_joint +/-')
        self.get_logger().info('e/d : elbow_joint +/-')
        self.get_logger().info('r/f : wrist_1_joint +/-')
        self.get_logger().info('t/g : wrist_2_joint +/-')
        self.get_logger().info('y/h : wrist_3_joint +/-')
        self.get_logger().info('o/p : gripper open/close')

    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        if hasattr(msg, 'actual') and hasattr(msg.actual, 'positions') and len(msg.actual.positions) == 6:
            self.joint_positions = list(msg.actual.positions)

    def send_joint_trajectory(self):
        """Send the current joint positions as a trajectory"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start.sec = 0  # Move immediately
        point.time_from_start.nanosec = 500000000  # 0.5 seconds
        
        msg.points.append(point)
        self.joint_pub.publish(msg)
        
    def send_gripper_command(self, position):
        """Send a command to the gripper"""
        msg = GripperCommand()
        msg.position = position
        msg.max_effort = 5.0  # Lower effort to prevent excessive force
        
        self.gripper_pub.publish(msg)

    def process_key(self, key):
        if key in JOINT_KEYS:
            idx, change = JOINT_KEYS[key]
            # Apply change directly rather than accumulating
            self.joint_positions[idx] += change
            self.send_joint_trajectory()
            return True
        elif key in GRIPPER_KEYS:
            self.gripper_position = GRIPPER_KEYS[key]
            self.send_gripper_command(self.gripper_position)
            return True
        return False

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    
    try:
        node = KeyboardCommandProcessor()
        print(MSG)
        
        while True:
            key = get_key(settings)
            if key == '\x03':  # CTRL-C
                break
                
            # Process the key
            if node.process_key(key):
                print(f"Command sent: {key}")
            else:
                # Small spin to process callbacks
                rclpy.spin_once(node, timeout_sec=0.01)
            
    except Exception as e:
        print(e)
        
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        # Shutdown ROS 2
        rclpy.shutdown()

if __name__ == '__main__':
    main()
