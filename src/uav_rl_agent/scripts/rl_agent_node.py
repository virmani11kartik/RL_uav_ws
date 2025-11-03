#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Imu

class RLAgentNode(Node):
    def __init__(self):
        super().__init__('rl_agent_node_py')
        # Sub: replace with your state topic(s)
        self.sub = self.create_subscription(Imu, '/imu/data', self.cb, 10)
        # Pub: replace with your actuation topic
        self.pub = self.create_publisher(UInt16MultiArray, '/motor_control/command', 10)
        # TODO: load your model, e.g. torch.jit.load('...pt')
        self.get_logger().info('uav_rl_agent (Python) ready.')

    def cb(self, msg: Imu):
        # TODO: convert observation -> policy(obs) -> action
        # Minimal placeholder: neutral 4x motors @ 1150 us
        out = UInt16MultiArray()
        out.data = [1150,1150,1150,1150]
        self.pub.publish(out)

def main():
    rclpy.init()
    node = RLAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
