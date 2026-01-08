#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ArmOnce(Node):
    def __init__(self):
        super().__init__("arm_once")
        self.declare_parameter("rc_out_topic", "/rc_aetr_aux")
        self.declare_parameter("aux1_arm_us", 1100)
        self.declare_parameter("center_us", 1500)
        self.declare_parameter("throttle_us", 988)
        self.declare_parameter("aux_defaults_us", [1500, 1500, 1500])

        out = self.get_parameter("rc_out_topic").value
        self.pub = self.create_publisher(String, out, 10)

        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        A = E = R = int(self.get_parameter("center_us").value)
        T = int(self.get_parameter("throttle_us").value)
        aux1 = int(self.get_parameter("aux1_arm_us").value)
        aux2, aux3, aux4 = [int(x) for x in self.get_parameter("aux_defaults_us").value]

        msg = String()
        msg.data = f"{A} {E} {T} {R} {aux1} {aux2} {aux3} {aux4}"
        self.pub.publish(msg)
        self.get_logger().info(f"Sent ARM once: {msg.data}")
        rclpy.shutdown()

def main():
    rclpy.init()
    ArmOnce()
    rclpy.spin(rclpy.get_default_context())

if __name__ == "__main__":
    main()
