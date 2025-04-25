#!/usr/bin/env python3
import socket
import threading
import rclpy
import sys
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

class CommandForwarder(Node):
    def _init_(self, sim_ip):
        super()._init_('command_forwarder')
        # Subscribe to local commands
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.forward_command,
            10)
        # Setup UDP socket to forward to simulation
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sim_address = (sim_ip, 12345)
        self.get_logger().info(f"Forwarding to {sim_ip}:12345")
        
    def forward_command(self, msg):
        # Serialize the message (simplified)
        joint_count = len(msg.joint_names)
        position_data = msg.points[0].positions if msg.points else []
        
        # Format: "joint_count,joint1,joint2,joint3,joint4,joint5,joint6"
        data = f"{joint_count},{','.join(map(str, position_data))}"
        
        # Send via UDP
        self.sock.sendto(data.encode(), self.sim_address)
        self.get_logger().info(f"Forwarded: {data}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 forward_commands.py <simulation_laptop_ip>")
        return
    
    sim_ip = sys.argv[1]
    
    rclpy.init()
    forwarder = CommandForwarder(sim_ip)
    rclpy.spin(forwarder)
    rclpy.shutdown()

if _name_ == '_main_':
    main()