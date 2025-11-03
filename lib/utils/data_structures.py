"""Data structures"""

from collections import namedtuple

# axis rates (deg/s): roll, pitch, yaw
AxisRates = namedtuple('AxisRates', 'p q r')

ControlCommands = namedtuple('ControlCommands', 'throttle roll pitch yaw')

class FlightMode:
    RATE = "rate"
    ANGLE = "angle"
    ALT_HOLD = "alt_hold"
    POS_HOLD = "pos_hold"

def create_axis_rates(roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0):
    return AxisRates(p=roll_rate, q=pitch_rate, r=yaw_rate)