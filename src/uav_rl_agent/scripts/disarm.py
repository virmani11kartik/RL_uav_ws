#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class DisarmOnce(Node):
    def __init__(self):
        super().__init__("disarm_once")

        # Topics
        self.declare_parameter("rc_out_topic", "/rc_aetr_aux")   # your bridge expects String
        self.declare_parameter("kill_topic", "/rl/kill")          # Bool

        # RC values (microseconds)
        self.declare_parameter("center_us", 1500)
        self.declare_parameter("throttle_us", 988)

        # AUX behavior
        self.declare_parameter("aux1_disarm_us", 900)
        self.declare_parameter("aux_defaults_us", [1500, 1500, 1500])  # AUX2, AUX3, AUX4

        # How many times to publish (for reliability)
        self.declare_parameter("publish_reps", 5)
        self.declare_parameter("publish_period_s", 0.05)

        self.rc_out_topic = self.get_parameter("rc_out_topic").value
        self.kill_topic = self.get_parameter("kill_topic").value

        self.pub_rc = self.create_publisher(String, self.rc_out_topic, 10)
        self.pub_kill = self.create_publisher(Bool, self.kill_topic, 10)

        self.reps = int(self.get_parameter("publish_reps").value)
        self.sent = 0

        period = float(self.get_parameter("publish_period_s").value)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"disarm_once: will publish kill + disarm on '{self.kill_topic}' and '{self.rc_out_topic}' ({self.reps} reps)"
        )

    def _tick(self):
        # 1) Publish kill command
        kill = Bool()
        kill.data = True
        self.pub_kill.publish(kill)

        # 2) Publish disarm RC packet
        A = E = R = int(self.get_parameter("center_us").value)
        T = int(self.get_parameter("throttle_us").value)

        aux1 = int(self.get_parameter("aux1_disarm_us").value)
        aux2, aux3, aux4 = [int(x) for x in self.get_parameter("aux_defaults_us").value]

        msg = String()
        msg.data = f"{A} {E} {T} {R} {aux1} {aux2} {aux3} {aux4}"
        self.pub_rc.publish(msg)

        self.sent += 1
        if self.sent >= self.reps:
            self.get_logger().warn(f"disarm_once: sent {self.reps} reps. Exiting.")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = DisarmOnce()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
