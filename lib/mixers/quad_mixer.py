"""Quad mixers"""

from .base_mixer import BaseMixer

class QuadXMixer(BaseMixer):
    """quad-X mixer
    
    Motor layout:
        Front
      M1    M2
      M4    M3   Rear
    """
    
    def __init__(self):
        super().__init__(motor_count=4)
    
    def mix(self, throttle, roll_cmd, pitch_cmd, yaw_cmd):
        m1 = throttle + roll_cmd + pitch_cmd - yaw_cmd  # front-left
        m2 = throttle - roll_cmd + pitch_cmd + yaw_cmd  # front-right
        m3 = throttle - roll_cmd - pitch_cmd - yaw_cmd  # rear-right
        m4 = throttle + roll_cmd - pitch_cmd + yaw_cmd  # rear-left
        return [m1, m2, m3, m4]
    
    def get_motor_layout(self):
        return "Quad-X: M1(FL) M2(FR) M3(RR) M4(RL)"

class QuadPlusMixer(BaseMixer):
    """quad-+ mixer
    
    Motor layout:
          M1
          |
    M4 ---+--- M2
          |
          M3
    """
    
    def __init__(self):
        super().__init__(motor_count=4)
    
    def mix(self, throttle, roll_cmd, pitch_cmd, yaw_cmd):
        m1 = throttle + pitch_cmd - yaw_cmd  # front
        m2 = throttle + roll_cmd + yaw_cmd   # right
        m3 = throttle - pitch_cmd - yaw_cmd  # rear
        m4 = throttle - roll_cmd + yaw_cmd   # left
        return [m1, m2, m3, m4]
    
    def get_motor_layout(self):
        return "Quad-+: M1(F) M2(R) M3(B) M4(L)"