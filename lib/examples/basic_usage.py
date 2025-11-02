"""Basic usage example of the flight control library"""

import time
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from lib import (
    RateController3D, QuadXMixer, DshotOutput, 
    AxisRates, RateMapper, QuadConfig, get_preset_config
)
from lib.mixers import desaturate_and_clip


def basic_flight_controller_example():
    """
    Basic example showing how to set up and use the flight control library
    """
    print("=== Basic Flight Controller Example ===\n")
    
    # 1. Load configuration (use size-based preset)
    config = get_preset_config("5in")
    print(f"Loaded config: {config.name}")
    print(f"Description: {config.description}\n")
    
    # 2. Initialize components
    # Rate controller with PID gains from config
    kp, ki, kd = config.get_pid_gains_tuple()
    controller = RateController3D(
        kp=kp, ki=ki, kd=kd,
        d_cut_hz=config.rates.d_cut_hz,
        dt_init=1.0/config.loop_frequency
    )
    
    # Mixer for quad-X configuration
    mixer = QuadXMixer()
    mixer.set_idle_throttle(config.mixer.motor_idle)
    
    # ESC output protocol
    esc_output = DshotOutput(
        motor_count=config.esc.motor_count,
        dshot_speed=600,  # DShot600
        telemetry=config.esc.telemetry
    )
    
    # Rate mapper for stick inputs
    rate_mapper = RateMapper(
        max_rates_dps=config.rate_mapping.max_rates_dps,
        expo=config.rate_mapping.expo,
        super_rates=config.rate_mapping.super_rates
    )
    
    print("Components initialized successfully!\n")
    
    # 3. Simulation loop
    dt = 1.0 / config.loop_frequency  # 500Hz loop
    throttle = 0.3  # 30% throttle
    
    # Stick inputs (simulate pilot input)
    stick_roll = 0.2   # 20% right roll
    stick_pitch = 0.0  # Level pitch
    stick_yaw = 0.1    # 10% right yaw
    
    # Convert stick inputs to rate targets
    rate_targets = rate_mapper.map_sticks_to_rates(stick_roll, stick_pitch, stick_yaw)
    print(f"Rate targets: Roll={rate_targets.p:.1f}°/s, Pitch={rate_targets.q:.1f}°/s, Yaw={rate_targets.r:.1f}°/s")
    
    # Simulate current gyro readings (start at zero)
    current_rates = AxisRates(0.0, 0.0, 0.0)
    
    # Arm the ESCs
    esc_output.arm()
    print("ESCs armed\n")
    
    print("Starting control loop...")
    print("Time(s)  | Roll Cmd | Pitch Cmd | Yaw Cmd | Motors [M1, M2, M3, M4] | DShot Values")
    print("-" * 85)
    
    for i in range(50):  # Run for 50 iterations (0.1 seconds at 500Hz)
        # 4. Update rate controller
        roll_cmd, pitch_cmd, yaw_cmd = controller.update(rate_targets, current_rates, dt)
        
        # 5. Mix control commands to motor outputs
        motor_outputs = mixer.mix(throttle, roll_cmd, pitch_cmd, yaw_cmd)
        
        # 6. Desaturate and clip motor outputs
        motor_outputs = desaturate_and_clip(motor_outputs, idle=config.mixer.motor_idle)
        
        # 7. Convert to ESC protocol values
        dshot_values = esc_output.process_outputs(motor_outputs)
        
        # Print every 10th iteration
        if i % 10 == 0:
            print(f"{i*dt:6.3f}  | {roll_cmd:8.3f} | {pitch_cmd:9.3f} | {yaw_cmd:7.3f} | "
                  f"{[f'{m:.3f}' for m in motor_outputs]} | {dshot_values}")
        
        # 8. Simulate plant response (very basic)
        # In real implementation, this would be actual gyro readings
        response_rate = 0.15  # How quickly the quad responds
        current_rates = AxisRates(
            p=current_rates.p + response_rate * (rate_targets.p - current_rates.p),
            q=current_rates.q + response_rate * (rate_targets.q - current_rates.q),
            r=current_rates.r + response_rate * (rate_targets.r - current_rates.r)
        )
        
        # Simulate real-time delay
        time.sleep(dt)
    
    # Disarm ESCs
    esc_output.disarm()
    print("\nESCs disarmed")
    print("Example completed successfully!")


def configuration_example():
    """Example showing different ways to work with configurations"""
    print("\n=== Configuration Example ===\n")
    
    # Load different presets
    racing_config = get_preset_config("3in")
    cruiser_config = get_preset_config("7in")
    
    print(f"3in racing max roll rate: {racing_config.rate_mapping.max_rates_dps[0]}°/s")
    print(f"7in cruiser max roll rate: {cruiser_config.rate_mapping.max_rates_dps[0]}°/s")
    
    # Create custom configuration
    custom_config = racing_config.clone(
        name="Custom 3in",
        description="Modified 3in config with higher rates"
    )
    
    # Modify specific parameters
    custom_config.rate_mapping.max_rates_dps = (1000.0, 1000.0, 800.0)
    custom_config.rates.roll.kp = 0.15
    
    print(f"Custom config max roll rate: {custom_config.rate_mapping.max_rates_dps[0]}°/s")
    print(f"Custom config roll P gain: {custom_config.rates.roll.kp}")
    
    # Validate configuration
    if custom_config.is_valid():
        print("Custom configuration is valid")
    else:
        print("Custom configuration has issues:", custom_config.validate())


def mixer_comparison_example():
    """Example comparing different mixer types"""
    print("\n=== Mixer Comparison Example ===\n")
    
    from lib.mixers import QuadPlusMixer
    
    # Test inputs
    throttle = 0.5
    roll_cmd = 0.3
    pitch_cmd = 0.2
    yaw_cmd = -0.1
    
    # Compare X and + configurations
    quad_x = QuadXMixer()
    quad_plus = QuadPlusMixer()
    
    x_outputs = quad_x.mix(throttle, roll_cmd, pitch_cmd, yaw_cmd)
    plus_outputs = quad_plus.mix(throttle, roll_cmd, pitch_cmd, yaw_cmd)
    
    print(f"Input: T={throttle}, R={roll_cmd}, P={pitch_cmd}, Y={yaw_cmd}")
    print(f"Quad-X outputs:  {[f'{m:.3f}' for m in x_outputs]}")
    print(f"Quad-+ outputs:  {[f'{m:.3f}' for m in plus_outputs]}")
    
    print(f"\nQuad-X layout:\n{quad_x.get_motor_layout()}")
    print(f"\nQuad-+ layout:\n{quad_plus.get_motor_layout()}")


if __name__ == "__main__":
    # Run all examples
    basic_flight_controller_example()
    configuration_example()
    mixer_comparison_example()
    
    print("\n=== All Examples Completed ===")
    print("lib is ready for integration!")
