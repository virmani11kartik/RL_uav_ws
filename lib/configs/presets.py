"""Size-based presets - 基于尺寸的预设"""

from .quad_config import QuadConfig, RateConfig, PIDGains, RateMappingConfig, MixerConfig, ESCConfig

# 2in micro quad
QUAD_2IN = QuadConfig(
    name="2in Micro",
    description="tiny whoop style",
    rates=RateConfig(
        roll=PIDGains(kp=0.15, ki=0.10, kd=0.003),
        pitch=PIDGains(kp=0.15, ki=0.10, kd=0.003),
        yaw=PIDGains(kp=0.18, ki=0.08, kd=0.0035),
        d_cut_hz=150.0
    ),
    rate_mapping=RateMappingConfig(
        max_rates_dps=(900.0, 900.0, 700.0),
        expo=(0.10, 0.10, 0.05),
        super_rates=(0.85, 0.85, 0.75)
    ),
    mixer=MixerConfig(type="quad_x", motor_idle=0.02),
    esc=ESCConfig(protocol="dshot300", telemetry=False),
    loop_frequency=2000.0,
    motor_poles=12,
    battery_cells=1
)

# 3in racing/freestyle
QUAD_3IN = QuadConfig(
    name="3in Racing",
    description="racing/freestyle",
    rates=RateConfig(
        roll=PIDGains(kp=0.12, ki=0.08, kd=0.002),
        pitch=PIDGains(kp=0.12, ki=0.08, kd=0.002),
        yaw=PIDGains(kp=0.15, ki=0.06, kd=0.0025),
        d_cut_hz=120.0
    ),
    rate_mapping=RateMappingConfig(
        max_rates_dps=(800.0, 800.0, 600.0),
        expo=(0.15, 0.15, 0.10),
        super_rates=(0.8, 0.8, 0.7)
    ),
    mixer=MixerConfig(type="quad_x", motor_idle=0.03),
    esc=ESCConfig(protocol="dshot600", telemetry=True),
    loop_frequency=1000.0,
    motor_poles=14,
    battery_cells=4
)

# 5in standard quad
QUAD_5IN = QuadConfig(
    name="5in Standard",
    description="standard quad",
    rates=RateConfig(
        roll=PIDGains(kp=0.08, ki=0.05, kd=0.0015),
        pitch=PIDGains(kp=0.08, ki=0.05, kd=0.0015),
        yaw=PIDGains(kp=0.12, ki=0.05, kd=0.002),
        d_cut_hz=90.0
    ),
    rate_mapping=RateMappingConfig(
        max_rates_dps=(600.0, 600.0, 500.0),
        expo=(0.25, 0.25, 0.20),
        super_rates=(0.7, 0.7, 0.6)
    ),
    mixer=MixerConfig(type="quad_x", motor_idle=0.05),
    esc=ESCConfig(protocol="dshot600", telemetry=False),
    loop_frequency=500.0,
    motor_poles=14,
    battery_cells=4
)

# 7in long range
QUAD_7IN = QuadConfig(
    name="7in Long Range",
    description="long range cruiser",
    rates=RateConfig(
        roll=PIDGains(kp=0.04, ki=0.02, kd=0.0008),
        pitch=PIDGains(kp=0.04, ki=0.02, kd=0.0008),
        yaw=PIDGains(kp=0.06, ki=0.015, kd=0.001),
        d_cut_hz=40.0
    ),
    rate_mapping=RateMappingConfig(
        max_rates_dps=(300.0, 300.0, 200.0),
        expo=(0.40, 0.40, 0.35),
        super_rates=(0.4, 0.4, 0.3)
    ),
    mixer=MixerConfig(type="quad_x", motor_idle=0.08),
    esc=ESCConfig(protocol="pwm", pwm_range=(1000, 2000)),
    loop_frequency=200.0,
    motor_poles=24,
    battery_cells=6
)

# preset collection
PRESET_CONFIGS = {
    "2in": QUAD_2IN,
    "3in": QUAD_3IN,
    "5in": QUAD_5IN,
    "7in": QUAD_7IN
}

def get_preset_config(size):
    """get preset config by size"""
    if size not in PRESET_CONFIGS:
        available = ", ".join(PRESET_CONFIGS.keys())
        raise KeyError(f"Size '{size}' not found. Available: {available}")
    return PRESET_CONFIGS[size].clone()