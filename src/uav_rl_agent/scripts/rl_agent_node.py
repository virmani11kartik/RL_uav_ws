#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class RLAgentNode(Node):
    def __init__(self):
        super().__init__('rl_agent_node_py')
        self.get_logger().info('rl_agent_node.py up and running')

def main():
    rclpy.init()
    node = RLAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
