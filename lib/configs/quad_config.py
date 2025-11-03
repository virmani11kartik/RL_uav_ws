"""Quad configuration"""

from dataclasses import dataclass, asdict
import json

@dataclass
class PIDGains:
    kp: float = 0.08
    ki: float = 0.05  
    kd: float = 0.0015

@dataclass
class RateConfig:
    roll: PIDGains = None
    pitch: PIDGains = None
    yaw: PIDGains = None
    d_cut_hz: float = 90.0
    
    def __post_init__(self):
        if self.roll is None:
            self.roll = PIDGains(0.08, 0.05, 0.0015)
        if self.pitch is None:
            self.pitch = PIDGains(0.08, 0.05, 0.0015)
        if self.yaw is None:
            self.yaw = PIDGains(0.12, 0.05, 0.002)

@dataclass  
class RateMappingConfig:
    max_rates_dps: tuple = (600.0, 600.0, 500.0)  # roll, pitch, yaw
    expo: tuple = (0.25, 0.25, 0.20)
    super_rates: tuple = (0.7, 0.7, 0.6)

@dataclass
class MixerConfig:
    type: str = "quad_x"
    motor_idle: float = 0.05

@dataclass
class ESCConfig:
    protocol: str = "dshot600"
    motor_count: int = 4
    pwm_range: tuple = (1000, 2000)
    telemetry: bool = False

@dataclass
class QuadConfig:
    name: str = "Default Quad"
    description: str = "default quad config"
    
    rates: RateConfig = None
    rate_mapping: RateMappingConfig = None
    mixer: MixerConfig = None
    esc: ESCConfig = None
    
    loop_frequency: float = 500.0  # Hz
    motor_poles: int = 14
    battery_cells: int = 4
    
    def __post_init__(self):
        if self.rates is None:
            self.rates = RateConfig()
        if self.rate_mapping is None:
            self.rate_mapping = RateMappingConfig()
        if self.mixer is None:
            self.mixer = MixerConfig()
        if self.esc is None:
            self.esc = ESCConfig()
    
    def to_dict(self):
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data):
        # handle nested structures
        if 'rates' in data and isinstance(data['rates'], dict):
            rates_data = data['rates']
            data['rates'] = RateConfig(
                roll=PIDGains(**rates_data.get('roll', {})),
                pitch=PIDGains(**rates_data.get('pitch', {})),
                yaw=PIDGains(**rates_data.get('yaw', {})),
                d_cut_hz=rates_data.get('d_cut_hz', 90.0)
            )
        
        if 'rate_mapping' in data and isinstance(data['rate_mapping'], dict):
            rm_data = data['rate_mapping']
            data['rate_mapping'] = RateMappingConfig(
                max_rates_dps=tuple(rm_data.get('max_rates_dps', [600.0, 600.0, 500.0])),
                expo=tuple(rm_data.get('expo', [0.25, 0.25, 0.20])),
                super_rates=tuple(rm_data.get('super_rates', [0.7, 0.7, 0.6]))
            )
        
        if 'mixer' in data and isinstance(data['mixer'], dict):
            data['mixer'] = MixerConfig(**data['mixer'])
        
        if 'esc' in data and isinstance(data['esc'], dict):
            esc_data = data['esc']
            data['esc'] = ESCConfig(
                protocol=esc_data.get('protocol', 'dshot600'),
                motor_count=esc_data.get('motor_count', 4),
                pwm_range=tuple(esc_data.get('pwm_range', [1000, 2000])),
                telemetry=esc_data.get('telemetry', False)
            )
        
        return cls(**data)
    
    def validate(self):
        issues = []
        
        # check PID gains
        for axis_name, gains in [('roll', self.rates.roll), ('pitch', self.rates.pitch), ('yaw', self.rates.yaw)]:
            if gains.kp < 0:
                issues.append(f"{axis_name} kp < 0")
            if gains.ki < 0:
                issues.append(f"{axis_name} ki < 0")
            if gains.kd < 0:
                issues.append(f"{axis_name} kd < 0")
        
        # check rates
        if any(rate <= 0 for rate in self.rate_mapping.max_rates_dps):
            issues.append("max rates <= 0")
        
        return issues
    
    def is_valid(self):
        return len(self.validate()) == 0
    
    def get_pid_gains_tuple(self):
        kp = (self.rates.roll.kp, self.rates.pitch.kp, self.rates.yaw.kp)
        ki = (self.rates.roll.ki, self.rates.pitch.ki, self.rates.yaw.ki)
        kd = (self.rates.roll.kd, self.rates.pitch.kd, self.rates.yaw.kd)
        return kp, ki, kd
    
    def clone(self, **overrides):
        data = self.to_dict()
        data.update(overrides)
        return QuadConfig.from_dict(data)