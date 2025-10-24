import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty


HELP = """
Keyboard Teleop (Betaflight sticks)
----------------------------------
Roll : a/d
Pitch : w/s
Yaw : j/l
Thr : i/k (thr 0..1)
Arm : t (topic handled by safety_gate)
Kill : x (thr=0 and disarm)
Quit : q
"""

class KeyTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, 'betaflight/cmd_rc', 10)
        self.pub_arm = self.create_publisher(Twist, 'betaflight/cmd_aux', 10) # used by safety_gate
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0; self.thr = 0.0
        self.timer = self.create_timer(0.05, self.tick)
        print(HELP)

    def tick(self):
        msg = Twist()
        msg.angular.y = self.roll
        msg.angular.x = self.pitch
        msg.angular.z = self.yaw
        msg.linear.x = self.thr
        self.pub.publish(msg)

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                if select_ready(fd):
                    ch = sys.stdin.read(1)
                    if ch == 'q':
                        break
                    elif ch == 'a': self.roll -= 0.05
                    elif ch == 'd': self.roll += 0.05
                    elif ch == 'w': self.pitch += 0.05
                    elif ch == 's': self.pitch -= 0.05
                    elif ch == 'j': self.yaw -= 0.05
                    elif ch == 'l': self.yaw += 0.05
                    elif ch == 'i': self.thr = min(1.0, self.thr + 0.02)
                    elif ch == 'k': self.thr = max(0.0, self.thr - 0.02)
                    elif ch == 't':
                        aux = Twist(); aux.linear.y = 1.0 # request arm
                        self.pub_arm.publish(aux)
                    elif ch == 'x':
                        aux = Twist(); aux.linear.y = -1.0 # kill/disarm
                        self.pub_arm.publish(aux)
                        self.saturate()
                        rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def saturate(self):
        self.roll = max(-1.0, min(1.0, self.roll))
        self.pitch = max(-1.0, min(1.0, self.pitch))
        self.yaw = max(-1.0, min(1.0, self.yaw))
        self.thr = max(0.0, min(1.0, self.thr))

import select

def select_ready(fd):
    r, _, _ = select.select([fd], [], [], 0)
    return bool(r)

def main():
    print("Starting keyboard teleop...")
    rclpy.init()
    node = KeyTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()