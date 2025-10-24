#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import Twist
import serial
import struct
import math


# MSP protocol constants
MSP_HEADER = b"$M<"
MSP_HEADER_RESP = b"$M>"
MSP_ATTITUDE = 108
MSP_ANALOG = 110
MSP_STATUS = 101
MSP_SET_RAW_RC = 200


def a2rc(a: float) -> int:
    """Map normalized action [-1,1] to RC microseconds [1000,2000]"""
    return max(1000, min(2000, int(1500 + 500.0 * a)))


class MSPClient:
    """Minimal MSP (MultiWii Serial Protocol) client for Betaflight"""
    
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.02):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)

    def _checksum(self, size, code, payload=b""):
        """Calculate MSP checksum"""
        c = size ^ code
        for b in payload:
            c ^= b
        return c.to_bytes(1, 'little')

    def _send(self, code: int, payload: bytes = b""):
        """Send MSP command"""
        size = len(payload).to_bytes(1, 'little')
        pkt = MSP_HEADER + size + code.to_bytes(1, 'little') + payload + self._checksum(len(payload), code, payload)
        self.ser.write(pkt)

    def _read(self):
        """Read MSP response"""
        # Read until header
        if self.ser.read(1) != b'$':
            return None, None
        if self.ser.read(1) != b'M':
            return None, None
        if self.ser.read(1) not in (b'>', b'<'):
            return None, None
        
        size = int.from_bytes(self.ser.read(1), 'little')
        code = int.from_bytes(self.ser.read(1), 'little')
        data = self.ser.read(size)
        _ = self.ser.read(1)  # checksum
        return code, data

    def get_attitude(self):
        """Get attitude (roll, pitch, yaw)"""
        self._send(MSP_ATTITUDE)
        code, data = self._read()
        if code != MSP_ATTITUDE or data is None or len(data) < 6:
            return None
        
        # roll, pitch in 0.1 deg; yaw in deg
        roll = struct.unpack('<h', data[0:2])[0] / 10.0
        pitch = struct.unpack('<h', data[2:4])[0] / 10.0
        yaw = struct.unpack('<h', data[4:6])[0]  # degrees
        return roll, pitch, yaw
    
    def get_analog(self):
        """Get battery and current info"""
        self._send(MSP_ANALOG)
        code, data = self._read()
        if code != MSP_ANALOG or data is None or len(data) < 7:
            return None
        
        vbat = data[0] / 10.0  # volts
        power = struct.unpack('<H', data[1:3])[0]  # mAh drawn or mW
        rssi = data[3]
        amperage = struct.unpack('<h', data[4:6])[0] / 100.0
        return vbat, amperage, power, rssi
    
    def send_rc(self, ch_us):
        """Send RC channels (microseconds)"""
        # ch_us: list of 8 ints in microseconds
        payload = b''.join([struct.pack('<H', int(v)) for v in ch_us[:8]])
        self._send(MSP_SET_RAW_RC, payload)


class MSPBridge(Node):
    """ROS2 bridge for Betaflight flight controller via MSP"""
    
    def __init__(self):
        super().__init__('msp_bridge')
        
        # Parameters
        port = self.declare_parameter('port', '/dev/ttyACM0').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value
        
        self.cli = MSPClient(port, baud)

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'betaflight/imu', 10)
        self.pub_batt = self.create_publisher(BatteryState, 'betaflight/battery', 10)
        self.pub_alive = self.create_publisher(Bool, 'betaflight/alive', 10)

        # Subscriber: incoming RC commands from safety_gate
        self.sub_cmd = self.create_subscription(Twist, 'betaflight/cmd_rc', self.on_cmd, 10)

        # Default RC frame: [roll, pitch, throttle, yaw, aux1-4]
        self.rc_frame = [1500] * 8
        self.rc_frame[2] = 1000  # throttle low by default
        self.armed = False

        # Timers
        self.create_timer(0.02, self.loop_rc)          # 50 Hz RC write
        self.create_timer(0.05, self.loop_att_batt)    # 20 Hz telemetry

    def on_cmd(self, msg: Twist):
        """Process incoming RC command from safety_gate"""
        # Expect normalized actions: roll, pitch, yaw in [-1,1]; throttle in [0,1]
        roll = max(-1.0, min(1.0, msg.angular.y))
        pitch = max(-1.0, min(1.0, msg.angular.x))
        yaw = max(-1.0, min(1.0, msg.angular.z))
        thr = max(0.0, min(1.0, msg.linear.x)) * 2.0 - 1.0  # map [0,1] -> [-1,1]
        
        self.rc_frame[0] = a2rc(roll)
        self.rc_frame[1] = a2rc(pitch)
        self.rc_frame[2] = a2rc(thr)
        self.rc_frame[3] = a2rc(yaw)

    def loop_rc(self):
        """Send RC commands to flight controller at 50Hz"""
        try:
            self.cli.send_rc(self.rc_frame)
            self.pub_alive.publish(Bool(data=True))
        except Exception as e:
            self.get_logger().warn(f"RC write failed: {e}")
            self.pub_alive.publish(Bool(data=False))

    def loop_att_batt(self):
        """Read telemetry from flight controller at 20Hz"""
        try:
            # Get attitude
            att = self.cli.get_attitude()
            if att:
                roll, pitch, yaw_deg = att
                msg = Imu()
                yaw = math.radians(yaw_deg)
                # Simple yaw-only quaternion
                msg.orientation.w = math.cos(yaw / 2.0)
                msg.orientation.z = math.sin(yaw / 2.0)
                self.pub_imu.publish(msg)
            
            # Get battery info
            ana = self.cli.get_analog()
            if ana:
                vbat, amperage, power, _ = ana
                b = BatteryState()
                b.voltage = vbat
                b.current = amperage
                self.pub_batt.publish(b)
        except Exception as e:
            self.get_logger().warn(f"Telemetry read failed: {e}")


def main(args=None):
    print("Starting MSP Bridge...")
    rclpy.init(args=args)
    node = MSPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()