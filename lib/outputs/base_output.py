"""
Base ESC output
ESC protocols probably not needed for RL UAV project.
So feel free to skip this file if you are reading.
"""

from abc import ABC, abstractmethod

class ESCProtocol(ABC):
    
    def __init__(self, motor_count):
        self.motor_count = motor_count
        self.armed = False
    
    @abstractmethod
    def convert_outputs(self, normalized_outputs):
        """convert outputs [0..1] to protocol values"""
        pass
    
    @abstractmethod
    def get_idle_value(self):
        pass
    
    @abstractmethod
    def get_max_value(self):
        pass
    
    @abstractmethod
    def validate_output(self, value):
        pass
    
    def arm(self):
        self.armed = True
    
    def disarm(self):
        self.armed = False
    
    def is_armed(self):
        return self.armed
    
    def get_safe_outputs(self):
        idle_value = self.get_idle_value()
        return [idle_value] * self.motor_count
    
    def process_outputs(self, normalized_outputs):
        if not self.armed:
            return self.get_safe_outputs()
        
        if len(normalized_outputs) != self.motor_count:
            raise ValueError(f"Expected {self.motor_count} outputs, got {len(normalized_outputs)}")
        
        # clamp inputs
        clamped = [max(0.0, min(1.0, x)) for x in normalized_outputs]
        
        # convert to protocol
        protocol_outputs = self.convert_outputs(clamped)
        
        # validate
        for i, output in enumerate(protocol_outputs):
            if not self.validate_output(output):
                raise ValueError(f"Invalid output {output} for motor {i}")
        
        return protocol_outputs