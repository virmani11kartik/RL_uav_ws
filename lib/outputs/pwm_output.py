"""PWM output"""

from .base_output import ESCProtocol

class PWMOutput(ESCProtocol):
    """PWM ESC output
    Standard: 1000-2000us @ 50Hz
    """
    
    def __init__(self, motor_count, us_min=1000, us_max=2000):
        super().__init__(motor_count)
        self.us_min = us_min
        self.us_max = us_max
        self.us_range = us_max - us_min
    
    def convert_outputs(self, normalized_outputs):
        return [int(self.us_min + x * self.us_range) for x in normalized_outputs]
    
    def get_idle_value(self):
        return self.us_min
    
    def get_max_value(self):
        return self.us_max
    
    def validate_output(self, value):
        return self.us_min <= value <= self.us_max

class OneShot125Output(ESCProtocol):
    """OneShot125 output
    Range: 125-250us, up to 4kHz
    """
    
    def __init__(self, motor_count):
        super().__init__(motor_count)
        self.us_min = 125
        self.us_max = 250
        self.us_range = 125
    
    def convert_outputs(self, normalized_outputs):
        return [int(self.us_min + x * self.us_range) for x in normalized_outputs]
    
    def get_idle_value(self):
        return self.us_min
    
    def get_max_value(self):
        return self.us_max
    
    def validate_output(self, value):
        return self.us_min <= value <= self.us_max