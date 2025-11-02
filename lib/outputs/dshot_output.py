"""DShot output"""

from .base_output import ESCProtocol

class DshotOutput(ESCProtocol):
    """DShot ESC output
    
    DShot speeds: 150/300/600/1200 kbit/s
    Values: 0=disarm, 1-47=commands, 48-2047=throttle
    """
    
    def __init__(self, motor_count, dshot_speed=600, telemetry=False):
        super().__init__(motor_count)
        
        if dshot_speed not in [150, 300, 600, 1200]:
            raise ValueError("DShot speed must be 150/300/600/1200")
        
        self.dshot_speed = dshot_speed
        self.telemetry = telemetry
        
        # value ranges
        self.min_throttle = 48
        self.max_throttle = 2047
        self.throttle_range = self.max_throttle - self.min_throttle
        
        # special commands
        self.DISARM = 0
        self.BEEP1 = 1
        self.BEEP2 = 2
        self.BEEP3 = 3
    
    def convert_outputs(self, normalized_outputs):
        return [int(self.min_throttle + x * self.throttle_range) for x in normalized_outputs]
    
    def get_idle_value(self):
        return self.min_throttle
    
    def get_max_value(self):
        return self.max_throttle
    
    def validate_output(self, value):
        return 0 <= value <= 2047
    
    def get_safe_outputs(self):
        return [self.DISARM] * self.motor_count
    
    def beep_motors(self, beep_type=1):
        beep_commands = {1: self.BEEP1, 2: self.BEEP2, 3: self.BEEP3}
        if beep_type not in beep_commands:
            raise ValueError("Beep type must be 1-3")
        return [beep_commands[beep_type]] * self.motor_count