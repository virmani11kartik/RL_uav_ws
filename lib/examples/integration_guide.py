"""Integration guide showing how to integrate the library into existing flight controllers"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from lib import *
import time


class FlightControllerIntegration:
    """
    Example showing how to integrate the flight control library 
    into an existing flight controller architecture
    """
    
    def __init__(self, config_name="default"):
        """Initialize flight controller with configuration"""
        # Load configuration
        self.config = get_preset_config(config_name)
        
        # Initialize control components
        self._init_controllers()
        self._init_mixer()
        self._init_outputs()
        self._init_rate_mapper()
        
        # Control state
        self.armed = False
        self.flight_mode = "rate"  # rate, angle, altitude_hold, etc.
        self.last_update_time = time.time()
        
        print(f"Flight controller initialized with '{config_name}' configuration")
    
    def _init_controllers(self):
        """Initialize PID controllers"""
        kp, ki, kd = self.config.get_pid_gains_tuple()
        
        self.rate_controller = RateController3D(
            kp=kp, ki=ki, kd=kd,
            d_cut_hz=self.config.rates.d_cut_hz,
            dt_init=1.0/self.config.loop_frequency
        )
        
        # Set output limits if needed
        self.rate_controller.set_output_limits((-1.0, 1.0))
    
    def _init_mixer(self):
        """Initialize motor mixer"""
        if self.config.mixer.type == "quad_x":
            self.mixer = QuadXMixer()
        elif self.config.mixer.type == "quad_plus":
            from lib.mixers import QuadPlusMixer
            self.mixer = QuadPlusMixer()
        else:
            raise ValueError(f"Unsupported mixer type: {self.config.mixer.type}")
        
        self.mixer.set_idle_throttle(self.config.mixer.motor_idle)
    
    def _init_outputs(self):
        """Initialize ESC outputs"""
        protocol = self.config.esc.protocol.lower()
        
        if protocol.startswith("dshot"):
            speed = int(protocol.replace("dshot", ""))
            self.esc_output = DshotOutput(
                motor_count=self.config.esc.motor_count,
                dshot_speed=speed,
                telemetry=self.config.esc.telemetry
            )
        elif protocol == "pwm":
            self.esc_output = PWMOutput(
                motor_count=self.config.esc.motor_count,
                us_min=self.config.esc.pwm_range[0],
                us_max=self.config.esc.pwm_range[1]
            )
        elif protocol == "oneshot125":
            from lib.outputs.pwm_output import OneShot125Output
            self.esc_output = OneShot125Output(self.config.esc.motor_count)
        elif protocol == "oneshot42":
            from lib.outputs.pwm_output import OneShot42Output
            self.esc_output = OneShot42Output(self.config.esc.motor_count)
        else:
            raise ValueError(f"Unsupported ESC protocol: {protocol}")
    
    def _init_rate_mapper(self):
        """Initialize rate mapping"""
        self.rate_mapper = RateMapper(
            max_rates_dps=self.config.rate_mapping.max_rates_dps,
            expo=self.config.rate_mapping.expo,
            super_rates=self.config.rate_mapping.super_rates
        )
    
    def arm(self):
        """Arm the flight controller"""
        if not self.armed:
            self.esc_output.arm()
            self.rate_controller.reset()  # Reset integrators
            self.armed = True
            print("Flight controller ARMED")
    
    def disarm(self):
        """Disarm the flight controller"""
        if self.armed:
            self.esc_output.disarm()
            self.armed = False
            print("Flight controller DISARMED")
    
    def update_control_loop(self, rc_inputs, gyro_rates, throttle_input):
        """
        Main control loop update - call this at your loop frequency
        
        Args:
            rc_inputs: Dict with 'roll', 'pitch', 'yaw' stick values [-1..1]
            gyro_rates: AxisRates with current gyro readings (deg/s)
            throttle_input: Throttle value [0..1]
            
        Returns:
            Dict with motor outputs and debug info
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update controller timing
        self.rate_controller.set_dt(dt)
        
        if not self.armed:
            # Return safe outputs when disarmed
            safe_outputs = self.esc_output.get_safe_outputs()
            return {
                'motor_outputs': safe_outputs,
                'armed': False,
                'dt': dt
            }
        
        # Convert RC inputs to rate targets
        rate_targets = self.rate_mapper.map_sticks_to_rates(
            rc_inputs['roll'], 
            rc_inputs['pitch'], 
            rc_inputs['yaw']
        )
        
        # Update rate controller
        roll_cmd, pitch_cmd, yaw_cmd = self.rate_controller.update(
            rate_targets, gyro_rates, dt
        )
        
        # Mix to motor outputs
        motor_outputs = self.mixer.mix(throttle_input, roll_cmd, pitch_cmd, yaw_cmd)
        
        # Desaturate and clip
        from lib.mixers import desaturate_and_clip
        motor_outputs = desaturate_and_clip(motor_outputs, self.config.mixer.motor_idle)
        
        # Convert to ESC protocol
        esc_outputs = self.esc_output.process_outputs(motor_outputs)
        
        return {
            'motor_outputs': esc_outputs,
            'rate_targets': rate_targets,
            'rate_commands': (roll_cmd, pitch_cmd, yaw_cmd),
            'raw_motor_outputs': motor_outputs,
            'armed': True,
            'dt': dt
        }
    
    def update_configuration(self, new_config):
        """Update configuration at runtime"""
        self.config = new_config
        
        # Reinitialize components with new config
        self._init_controllers()
        self._init_mixer() 
        self._init_outputs()
        self._init_rate_mapper()
        
        print(f"Configuration updated to: {new_config.name}")
    
    def get_status(self):
        """Get flight controller status"""
        return {
            'armed': self.armed,
            'flight_mode': self.flight_mode,
            'config_name': self.config.name,
            'loop_frequency': self.config.loop_frequency,
            'mixer_type': self.config.mixer.type,
            'esc_protocol': self.config.esc.protocol,
            'controller_state': self.rate_controller.get_state() if hasattr(self.rate_controller, 'get_state') else None
        }


def integration_example():
    """Example showing how to use the integrated flight controller"""
    print("=== Flight Controller Integration Example ===\n")
    
    # Initialize flight controller with 5in config
    fc = FlightControllerIntegration("5in")
    
    # Simulate RC receiver inputs
    rc_inputs = {
        'roll': 0.3,    # 30% right roll stick
        'pitch': -0.1,  # 10% forward pitch stick  
        'yaw': 0.0      # Centered yaw stick
    }
    
    throttle = 0.4  # 40% throttle
    
    # Simulate gyro readings (starting from hover)
    gyro_rates = AxisRates(0.0, 0.0, 0.0)
    
    # Arm the flight controller
    fc.arm()
    
    print("Running control loop simulation...")
    print("Time(s) | Rate Targets (°/s) | Commands | Motor Outputs")
    print("-" * 60)
    
    for i in range(20):
        # Update control loop
        result = fc.update_control_loop(rc_inputs, gyro_rates, throttle)
        
        if i % 5 == 0:  # Print every 5th iteration
            targets = result['rate_targets']
            commands = result['rate_commands']
            motors = result['motor_outputs']
            
            print(f"{i*0.002:6.3f} | R:{targets.p:5.1f} P:{targets.q:5.1f} Y:{targets.r:5.1f} | "
                  f"R:{commands[0]:5.2f} P:{commands[1]:5.2f} Y:{commands[2]:5.2f} | {motors}")
        
        # Simulate gyro response (very basic)
        if result['armed']:
            alpha = 0.1
            target_rates = result['rate_targets']
            gyro_rates = AxisRates(
                p=gyro_rates.p + alpha * (target_rates.p - gyro_rates.p),
                q=gyro_rates.q + alpha * (target_rates.q - gyro_rates.q),
                r=gyro_rates.r + alpha * (target_rates.r - gyro_rates.r)
            )
        
        time.sleep(0.002)  # 500Hz loop
    
    # Disarm
    fc.disarm()
    
    # Show status
    status = fc.get_status()
    print(f"\nFinal Status: {status}")


def configuration_switching_example():
    """Example showing runtime configuration switching"""
    print("\n=== Configuration Switching Example ===\n")
    
    fc = FlightControllerIntegration("5in")
    
    # Test different configurations
    configs_to_test = ["3in", "7in", "2in"]
    
    for config_name in configs_to_test:
        print(f"Switching to '{config_name}' configuration...")
        
        new_config = get_preset_config(config_name)
        fc.update_configuration(new_config)
        
        # Test a quick control update
        rc_inputs = {'roll': 0.5, 'pitch': 0.0, 'yaw': 0.0}
        gyro_rates = AxisRates(0.0, 0.0, 0.0)
        
        fc.arm()
        result = fc.update_control_loop(rc_inputs, gyro_rates, 0.3)
        fc.disarm()
        
        targets = result['rate_targets']
        print(f"  Max roll rate: {targets.p:.1f}°/s")
        print(f"  Motor outputs: {result['motor_outputs']}")
        print()


# def custom_mixer_example():
#     """Example showing how to create and use custom mixers"""
#     # CustomQuadMixer not implemented yet
#     pass


if __name__ == "__main__":
    # Run integration examples
    integration_example()
    configuration_switching_example()
    # custom_mixer_example()  # not implemented yet
    
    print("\n=== Integration Guide Complete ===")
    print("lib is ready for use")
    print("\nNext steps:")
    print("1. Integrate FlightControllerIntegration into your main FC")
    print("2. Replace simulated gyro_rates with real IMU data")
    print("3. Replace simulated rc_inputs with real RC receiver data")
    print("4. Add hardware-specific ESC output drivers")
    print("5. Tune PID gains for your aircraft")
