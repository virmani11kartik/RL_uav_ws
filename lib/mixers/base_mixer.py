"""Base mixer"""

from abc import ABC, abstractmethod

class BaseMixer(ABC):
    
    def __init__(self, motor_count):
        self.motor_count = motor_count
        self.motor_idle = 0.05
    
    @abstractmethod
    def mix(self, throttle, roll_cmd, pitch_cmd, yaw_cmd):
        pass
    
    @abstractmethod
    def get_motor_layout(self):
        pass
    
    def set_idle_throttle(self, idle):
        self.motor_idle = max(0.0, min(idle, 0.2))
    
    def get_motor_count(self):
        return self.motor_count