#!/usr/bin/env python3
"""
Simple demo - 简单演示
Shows the refactored lib in action
"""

import time
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lib import *

def demo():
    print("=== lib Demo ===")
    print("modular flight control components\n")
    
    # load different size configs
    sizes = ["2in", "3in", "5in", "7in"]
    print("Available sizes:")
    for size in sizes:
        config = get_preset_config(size)
        max_roll = config.rate_mapping.max_rates_dps[0]
        print(f"  {size}: {config.name} - {max_roll:.0f}°/s max roll")
    
    # demo with 5in config
    print(f"\nUsing 5in config...")
    config = get_preset_config("5in")
    
    # init components
    kp, ki, kd = config.get_pid_gains_tuple()
    controller = RateController3D(kp=kp, ki=ki, kd=kd)
    mixer = QuadXMixer()
    esc_output = DshotOutput(motor_count=4, dshot_speed=600)
    rate_mapper = RateMapper(
        max_rates_dps=config.rate_mapping.max_rates_dps,
        expo=config.rate_mapping.expo,
        super_rates=config.rate_mapping.super_rates
    )
    
    # control loop demo
    print("\nControl loop:")
    dt = 0.002
    throttle = 0.3
    stick_roll = 0.4  # 40% right roll
    
    rate_targets = rate_mapper.map_sticks_to_rates(stick_roll, 0.0, 0.0)
    gyro = AxisRates(0.0, 0.0, 0.0)
    
    esc_output.arm()
    print("Time | Commands | Motors | DShot")
    
    for i in range(20):
        # update controller
        u_roll, u_pitch, u_yaw = controller.update(rate_targets, gyro, dt)
        
        # mixing
        motors = mixer.mix(throttle, u_roll, u_pitch, u_yaw)
        motors = desaturate_and_clip(motors)
        
        # output
        dshot_vals = esc_output.process_outputs(motors)
        
        if i % 5 == 0:
            print(f"{i*dt:.3f} | [{u_roll:+.2f},{u_pitch:+.2f},{u_yaw:+.2f}] | "
                  f"{[f'{m:.2f}' for m in motors]} | {dshot_vals[:2]}...")
        
        # simulate response
        alpha = 0.1
        gyro = AxisRates(
            p=gyro.p + alpha*(rate_targets.p - gyro.p),
            q=gyro.q + alpha*(rate_targets.q - gyro.q),
            r=gyro.r + alpha*(rate_targets.r - gyro.r)
        )
        
        time.sleep(dt)
    
    esc_output.disarm()
    print("\nDemo complete - lib is ready")

if __name__ == "__main__":
    demo()
