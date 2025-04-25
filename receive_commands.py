#!/usr/bin/env python3
import socket
import threading
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class CommandReceiver(Node):
    def __init__(self):
        super().__init__('command_receiver')
        # Create publisher to control robot
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
            
        # Joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Setup UDP listening socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 12345))
        
        # Start listening thread
        self.listening = True
        self.listen_thread = threading.Thread(target=self.listen_for_commands)
        self.listen_thread.start()
        
    def listen_for_commands(self):
        while self.listening:
            try:
                data, addr = self.sock.recvfrom(1024)
                data_str = data.decode()
                self.get_logger().info(f"Received: {data_str}")
                
                # Parse data
                parts = data_str.split(',')
                joint_count = int(parts[0])
                positions = [float(p) for p in parts[1:joint_count+1]]
                
                # Create and publish message
                msg = JointTrajectory()
                msg.joint_names = self.joint_names
                
                point = JointTrajectoryPoint()
                point.positions = positions
                point.time_from_start = Duration(sec=0, nanosec=500000000)
                
                msg.points.append(point)
                self.publisher.publish(msg)
                
                
            except Exception as e:
                self.get_logger().error(f"Error: {str(e)}")
                
    def destroy_node(self):
        self.listening = False
        self.listen_thread.join()
        super().destroy_node()

def main():
    rclpy.init()
    receiver = CommandReceiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
