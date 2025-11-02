#!/usr/bin/env python3
"""
Interactive quad simulator with keyboard control
WASD + Arrow keys for control, real-time visualization
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
import sys
import os

# add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lib import *


class QuadPhysics:
    """Simple quadcopter physics simulator"""
    
    def __init__(self, mass=0.5, arm_length=0.15):
        self.mass = mass
        self.arm_length = arm_length
        
        # inertia (kg*m^2)
        self.Ixx = 0.01
        self.Iyy = 0.01
        self.Izz = 0.02
        
        # state: [roll, pitch, yaw, p, q, r]
        self.state = np.zeros(6)
        self.drag_coeff = 0.1
        
    def step(self, motor_outputs, dt):
        """Update physics"""
        # motor thrust (normalized to force)
        thrust = np.array(motor_outputs) * 8.0
        
        # torques from motor differential
        roll_torque = self.arm_length * (thrust[0] + thrust[3] - thrust[1] - thrust[2])
        pitch_torque = self.arm_length * (thrust[0] + thrust[1] - thrust[2] - thrust[3])
        yaw_torque = 0.05 * (thrust[0] - thrust[1] + thrust[2] - thrust[3])  # flipped sign
        
        # state
        p, q, r = self.state[3:6]
        
        # angular acceleration
        p_dot = (roll_torque / self.Ixx) - self.drag_coeff * p
        q_dot = (pitch_torque / self.Iyy) - self.drag_coeff * q
        r_dot = (yaw_torque / self.Izz) - self.drag_coeff * r
        
        # integrate
        self.state[3:6] += np.array([p_dot, q_dot, r_dot]) * dt
        self.state[0:3] += self.state[3:6] * dt
        
        return self.get_rates()
    
    def get_rates(self):
        """Get angular rates in deg/s"""
        return AxisRates(
            p=np.rad2deg(self.state[3]),
            q=np.rad2deg(self.state[4]),
            r=np.rad2deg(self.state[5])
        )
    
    def get_angles_deg(self):
        """Get angles in degrees"""
        return np.rad2deg(self.state[0:3])


class InteractiveSimulator:
    """Interactive quad simulator with keyboard control"""
    
    def __init__(self, config_name="5in"):
        # load config
        self.config = get_preset_config(config_name)
        
        # init flight control components
        kp, ki, kd = self.config.get_pid_gains_tuple()
        self.controller = RateController3D(kp=kp, ki=ki, kd=kd)
        self.mixer = QuadXMixer()
        self.rate_mapper = RateMapper(
            max_rates_dps=self.config.rate_mapping.max_rates_dps,
            expo=self.config.rate_mapping.expo,
            super_rates=self.config.rate_mapping.super_rates
        )
        
        # physics
        self.physics = QuadPhysics()
        
        # control inputs (stick positions)
        self.stick_roll = 0.0
        self.stick_pitch = 0.0
        self.stick_yaw = 0.0
        self.throttle = 0.5
        
        # input sensitivity
        self.stick_rate = 2.0  # how fast sticks move
        
        # control loop
        self.dt = 0.05  # 20Hz for better performance
        self.armed = True
        
        # data logging for plots
        self.max_history = 100  # 5 seconds at 20Hz
        self.time_data = []
        self.rate_targets = {'roll': [], 'pitch': [], 'yaw': []}
        self.rate_actual = {'roll': [], 'pitch': [], 'yaw': []}
        self.motors = [[], [], [], []]
        self.angles = {'roll': [], 'pitch': [], 'yaw': []}
        
        print(f"=== Interactive Quad Simulator ===")
        print(f"Config: {self.config.name}")
        print(f"Max rates: {self.config.rate_mapping.max_rates_dps}")
        print("\nControls (RC Style):")
        print("  Left Stick (WASD):")
        print("    W/S - Throttle up/down")
        print("    A/D - Yaw left/right")
        print("  Right Stick (Arrows):")
        print("    ↑/↓ - Pitch forward/back")
        print("    ←/→ - Roll left/right")
        print("  R - Reset")
        print("  Q - Quit")
        print("\nStarting simulation...")
        
    def update_sticks(self, keys):
        """Update stick positions based on keyboard input
        Left stick (WASD): throttle/yaw
        Right stick (Arrows): roll/pitch
        """
        # LEFT STICK (WASD)
        # throttle (W=up, S=down) - no spring
        if 'w' in keys:
            self.throttle = min(1.0, self.throttle + 1.0 * self.dt)  # faster response
        elif 's' in keys:
            self.throttle = max(0.0, self.throttle - 1.0 * self.dt)
            
        # yaw (A=left/CCW, D=right/CW)
        if 'a' in keys:
            self.stick_yaw = max(-1.0, self.stick_yaw - self.stick_rate * self.dt)  # yaw left
        elif 'd' in keys:
            self.stick_yaw = min(1.0, self.stick_yaw + self.stick_rate * self.dt)  # yaw right
        else:
            # center spring
            self.stick_yaw *= 0.85
            
        # RIGHT STICK (Arrows)
        # roll (left=roll left, right=roll right)
        if 'left' in keys:
            self.stick_roll = max(-1.0, self.stick_roll - self.stick_rate * self.dt)  # roll left
        elif 'right' in keys:
            self.stick_roll = min(1.0, self.stick_roll + self.stick_rate * self.dt)  # roll right
        else:
            # center spring
            self.stick_roll *= 0.85
            
        # pitch (up=forward, down=back)
        if 'up' in keys:
            self.stick_pitch = min(1.0, self.stick_pitch + self.stick_rate * self.dt)  # pitch forward
        elif 'down' in keys:
            self.stick_pitch = max(-1.0, self.stick_pitch - self.stick_rate * self.dt)  # pitch back
        else:
            # center spring
            self.stick_pitch *= 0.85
    
    def step(self, keys):
        """Single control loop iteration"""
        # update inputs
        self.update_sticks(keys)
        
        # map sticks to rate targets
        rate_targets = self.rate_mapper.map_sticks_to_rates(
            self.stick_roll, self.stick_pitch, self.stick_yaw
        )
        
        # get current rates
        current_rates = self.physics.get_rates()
        
        # PID update
        roll_cmd, pitch_cmd, yaw_cmd = self.controller.update(
            rate_targets, current_rates, self.dt
        )
        
        # mix to motors
        motor_outputs = self.mixer.mix(self.throttle, roll_cmd, pitch_cmd, yaw_cmd)
        motor_outputs = desaturate_and_clip(motor_outputs, idle=0.05)
        
        # update physics
        self.physics.step(motor_outputs, self.dt)
        
        # log data
        self.time_data.append(len(self.time_data) * self.dt)
        self.rate_targets['roll'].append(rate_targets.p)
        self.rate_targets['pitch'].append(rate_targets.q)
        self.rate_targets['yaw'].append(rate_targets.r)
        self.rate_actual['roll'].append(current_rates.p)
        self.rate_actual['pitch'].append(current_rates.q)
        self.rate_actual['yaw'].append(current_rates.r)
        
        angles = self.physics.get_angles_deg()
        self.angles['roll'].append(angles[0])
        self.angles['pitch'].append(angles[1])
        self.angles['yaw'].append(angles[2])
        
        for i in range(4):
            self.motors[i].append(motor_outputs[i])
        
        # keep history size manageable
        if len(self.time_data) > self.max_history:
            self.time_data.pop(0)
            for axis in ['roll', 'pitch', 'yaw']:
                self.rate_targets[axis].pop(0)
                self.rate_actual[axis].pop(0)
                self.angles[axis].pop(0)
            for i in range(4):
                self.motors[i].pop(0)
        
        return motor_outputs, rate_targets, current_rates
    
    def reset(self):
        """Reset simulation"""
        self.physics.state = np.zeros(6)
        self.controller.reset()
        self.stick_roll = 0.0
        self.stick_pitch = 0.0
        self.stick_yaw = 0.0
        self.throttle = 0.5
        print("Reset simulation")


def run_interactive_sim(config_name="5in"):
    """Run interactive simulation with live plots"""
    sim = InteractiveSimulator(config_name)
    
    # setup figure with subplots
    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
    
    # rate plots
    ax_roll = fig.add_subplot(gs[0, 0])
    ax_pitch = fig.add_subplot(gs[1, 0])
    ax_yaw = fig.add_subplot(gs[2, 0])
    
    # motor outputs
    ax_motors = fig.add_subplot(gs[0, 1])
    
    # angles
    ax_angles = fig.add_subplot(gs[1, 1])
    
    # stick positions
    ax_sticks = fig.add_subplot(gs[2, 1])
    
    # 3D attitude visualization
    ax_3d = fig.add_subplot(gs[:, 2], projection='3d')
    
    # keyboard state
    keys_pressed = set()
    
    def on_key_press(event):
        if event.key == 'q':
            plt.close()
            return
        elif event.key == 'r':
            sim.reset()
            return
        keys_pressed.add(event.key)
    
    def on_key_release(event):
        keys_pressed.discard(event.key)
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    fig.canvas.mpl_connect('key_release_event', on_key_release)
    
    # init lines
    lines = {}
    
    def init():
        for ax in [ax_roll, ax_pitch, ax_yaw, ax_motors, ax_angles]:
            ax.clear()
        return []
    
    def update(frame):
        # step simulation
        motors, targets, actual = sim.step(keys_pressed)
        
        if len(sim.time_data) < 2:
            return []
        
        # only update plots every 3 frames for better performance
        if frame % 3 != 0:
            return []
        
        t = np.array(sim.time_data)
        
        # plot rates - simplified for performance
        for ax, axis, color in [(ax_roll, 'roll', 'r'), (ax_pitch, 'pitch', 'g'), (ax_yaw, 'yaw', 'b')]:
            ax.clear()
            ax.plot(t, sim.rate_targets[axis], f'{color}--', linewidth=1.5, alpha=0.6)
            ax.plot(t, sim.rate_actual[axis], f'{color}-', linewidth=1.5)
            ax.set_ylabel(f'{axis.capitalize()}')
            ax.grid(True, alpha=0.2)
            ax.set_xlim([max(0, t[-1]-5), t[-1]])
        
        ax_yaw.set_xlabel('Time (s)')
        
        # plot motors - simplified
        ax_motors.clear()
        colors = ['r', 'g', 'b', 'orange']
        for i in range(4):
            ax_motors.plot(t, sim.motors[i], colors[i], linewidth=1.5)
        ax_motors.set_ylabel('Motors')
        ax_motors.set_ylim([0, 1.1])
        ax_motors.grid(True, alpha=0.2)
        ax_motors.set_xlim([max(0, t[-1]-5), t[-1]])
        
        # plot angles - simplified
        ax_angles.clear()
        for axis, color in [('roll', 'r'), ('pitch', 'g'), ('yaw', 'b')]:
            ax_angles.plot(t, sim.angles[axis], color, linewidth=1.5)
        ax_angles.set_ylabel('Angles')
        ax_angles.grid(True, alpha=0.2)
        ax_angles.set_xlim([max(0, t[-1]-5), t[-1]])
        
        # stick positions
        ax_sticks.clear()
        ax_sticks.set_xlim([-1.5, 1.5])
        ax_sticks.set_ylim([-1.5, 1.5])
        ax_sticks.set_aspect('equal')
        
        # left stick (WASD): throttle/yaw
        ax_sticks.add_patch(plt.Circle((-0.7, 0), 0.5, fill=False, color='gray', linewidth=2))
        ax_sticks.plot(-0.7 + sim.stick_yaw*0.4, (sim.throttle - 0.5)*0.8, 'ro', markersize=15)
        ax_sticks.text(-0.7, -1.2, 'WASD\nThrottle/Yaw', ha='center', fontsize=9)
        
        # right stick (Arrows): roll/pitch  
        ax_sticks.add_patch(plt.Circle((0.7, 0), 0.5, fill=False, color='gray', linewidth=2))
        ax_sticks.plot(0.7 + sim.stick_roll*0.4, sim.stick_pitch*0.4, 'bo', markersize=15)
        ax_sticks.text(0.7, -1.2, 'Arrows\nRoll/Pitch', ha='center', fontsize=9)
        
        ax_sticks.axis('off')
        
        # 3D visualization
        ax_3d.clear()
        angles = sim.physics.get_angles_deg()
        roll, pitch, yaw = np.deg2rad(angles)
        
        # draw quad frame
        arm_len = 1.0
        # rotation matrix
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        
        # quad arms in body frame
        arms = np.array([[arm_len, arm_len, 0], [arm_len, -arm_len, 0], 
                        [-arm_len, arm_len, 0], [-arm_len, -arm_len, 0]]).T
        arms_rot = R @ arms
        
        # draw arms
        for i in range(4):
            ax_3d.plot([0, arms_rot[0,i]], [0, arms_rot[1,i]], [0, arms_rot[2,i]], 
                      'r-' if i < 2 else 'b-', linewidth=3)
        
        ax_3d.set_xlim([-2, 2])
        ax_3d.set_ylim([-2, 2])
        ax_3d.set_zlim([-2, 2])
        ax_3d.set_xlabel('X')
        ax_3d.set_ylabel('Y')
        ax_3d.set_zlabel('Z')
        ax_3d.set_title('Quad Attitude')
        
        # status text
        fig.suptitle(f'{sim.config.name} | Throttle: {sim.throttle:.2f} | Armed: {sim.armed}', 
                    fontsize=12, weight='bold')
        
        return []
    
    # animation - optimized settings
    ani = FuncAnimation(fig, update, init_func=init, interval=int(sim.dt*1000), 
                       blit=True, cache_frame_data=False, save_count=0)
    
    plt.show()


if __name__ == "__main__":
    # can pass config name as argument
    config = "5in" if len(sys.argv) < 2 else sys.argv[1]
    run_interactive_sim(config)