import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')
        
        # Publish gated commands to MSP bridge
        self.pub = self.create_publisher(Twist, 'betaflight/cmd_rc', 10)
        
        # Subscribe to raw commands from teleop/AI
        self.sub_rc = self.create_subscription(Twist, 'cmd_raw', self.on_cmd, 10)
        
        # Subscribe to arm/disarm commands
        self.sub_aux = self.create_subscription(Twist, 'betaflight/cmd_aux', self.on_aux, 10)
        
        # Subscribe to flight controller alive status
        self.sub_alive = self.create_subscription(Bool, 'betaflight/alive', self.on_alive, 10)
        
        self.armed = False
        self.fc_alive = False
        self.last_cmd = Twist()
        
        # 50Hz gate update
        self.timer = self.create_timer(0.02, self.tick)

    def on_cmd(self, msg: Twist):
        """Store latest command from teleop/AI"""
        self.last_cmd = msg

    def on_aux(self, msg: Twist):
        """Handle arm/disarm requests via linear.y"""
        # linear.y: 1.0 => arm request, -1.0 => kill
        if msg.linear.y > 0.5:
            self.armed = True
            self.get_logger().info("ARMED")
        elif msg.linear.y < -0.5:
            self.armed = False
            self.get_logger().info("DISARMED")
    
    def on_alive(self, msg: Bool):
        """Track flight controller connection status"""
        was_alive = self.fc_alive
        self.fc_alive = msg.data
        if was_alive and not self.fc_alive:
            self.get_logger().warn("Flight controller connection lost!")

    def tick(self):
        """Gate commands based on armed state and FC connection"""
        out = Twist()
        
        if self.armed and self.fc_alive:
            # Pass through commands
            out = self.last_cmd
        else:
            # Safety: zero all commands
            out.angular.x = 0.0  # pitch
            out.angular.y = 0.0  # roll
            out.angular.z = 0.0  # yaw
            out.linear.x = 0.0   # throttle cut
        
        self.pub.publish(out)


def main():
    print("Starting Safety Gate...")
    rclpy.init()
    node = SafetyGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()