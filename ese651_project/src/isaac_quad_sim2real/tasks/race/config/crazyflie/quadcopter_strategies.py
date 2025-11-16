# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Modular strategy classes for quadcopter environment rewards, observations, and resets."""

from __future__ import annotations

import torch
import numpy as np
from typing import TYPE_CHECKING, Dict, Optional, Tuple

from isaaclab.utils.math import subtract_frame_transforms, quat_from_euler_xyz, euler_xyz_from_quat, wrap_to_pi, matrix_from_quat

if TYPE_CHECKING:
    from .quadcopter_env import QuadcopterEnv

D2R = np.pi / 180.0
R2D = 180.0 / np.pi


class DefaultQuadcopterStrategy:
    """Default strategy implementation for quadcopter environment."""

    def __init__(self, env: QuadcopterEnv):
        """Initialize the default strategy.

        Args:
            env: The quadcopter environment instance.
        """
        self.env = env
        self.device = env.device
        self.num_envs = env.num_envs
        self.cfg = env.cfg

        # Initialize episode sums for logging if in training mode
        if self.cfg.is_train and hasattr(env, 'rew'):
            keys = [key.split("_reward_scale")[0] for key in env.rew.keys() if key != "death_cost"]
            self._episode_sums = {
                key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
                for key in keys
            }

        # Initialize fixed parameters once (no domain randomization)
        # These parameters remain constant throughout the simulation
        # Aerodynamic drag coefficients
        self.env._K_aero[:, :2] = self.env._k_aero_xy_value
        self.env._K_aero[:, 2] = self.env._k_aero_z_value

        # PID controller gains for angular rate control
        # Roll and pitch use the same gains
        self.env._kp_omega[:, :2] = self.env._kp_omega_rp_value
        self.env._ki_omega[:, :2] = self.env._ki_omega_rp_value
        self.env._kd_omega[:, :2] = self.env._kd_omega_rp_value

        # Yaw has different gains
        self.env._kp_omega[:, 2] = self.env._kp_omega_y_value
        self.env._ki_omega[:, 2] = self.env._ki_omega_y_value
        self.env._kd_omega[:, 2] = self.env._kd_omega_y_value

        # Motor time constants (same for all 4 motors)
        self.env._tau_m[:] = self.env._tau_m_value

        # Thrust to weight ratio
        self.env._thrust_to_weight[:] = self.env._twr_value

    def get_rewards(self) -> torch.Tensor:
            
        """Compute rewards for high-speed drone racing."""

        # ==================== GATE PASSING DETECTION ====================
        # Check if drone has passed through current gate
        crossed_gate_plane = self.env._pose_drone_wrt_gate[:, 0] < 0.2
        within_gate_bounds = (
            (torch.abs(self.env._pose_drone_wrt_gate[:, 1]) < 0.6) &
            (torch.abs(self.env._pose_drone_wrt_gate[:, 2]) < 0.6)
        )
        was_behind_gate = self.env._prev_x_drone_wrt_gate > 0
        
        gate_passed = crossed_gate_plane & within_gate_bounds & was_behind_gate
        
        # Update previous x position for next timestep
        self.env._prev_x_drone_wrt_gate = self.env._pose_drone_wrt_gate[:, 0].clone()

        # ==================== SPEED-DEPENDENT GATE PASS BONUS ====================
        # Get drone speed magnitude at gate pass
        drone_speed = torch.linalg.norm(self.env._robot.data.root_com_lin_vel_w, dim=1)
        
        # Speed-scaled gate pass bonus: base reward + speed multiplier
        gate_pass_bonus = torch.zeros(self.num_envs, device=self.device)
        gate_pass_bonus[gate_passed] = 10.0 + 3.0 * drone_speed[gate_passed]
        
        # Update waypoint indices for environments that passed gates
        ids_gate_passed = torch.where(gate_passed)[0]
        if len(ids_gate_passed) > 0:
            self.env._n_gates_passed[ids_gate_passed] += 1
            self.env._idx_wp[ids_gate_passed] = (self.env._idx_wp[ids_gate_passed] + 1) % self.env._waypoints.shape[0]
            
            # Update desired positions to next gate
            self.env._desired_pos_w[ids_gate_passed, :] = self.env._waypoints[self.env._idx_wp[ids_gate_passed], :3].clone()
            
            # Update gate-relative pose for new target gate
            self.env._pose_drone_wrt_gate[ids_gate_passed], _ = subtract_frame_transforms(
                self.env._waypoints[self.env._idx_wp[ids_gate_passed], :3],
                self.env._waypoints_quat[self.env._idx_wp[ids_gate_passed], :],
                self.env._robot.data.root_link_state_w[ids_gate_passed, :3]
            )

        # ==================== PROGRESS AND VELOCITY METRICS ====================
        distance_to_gate = torch.linalg.norm(
            self.env._desired_pos_w[:, :2] - self.env._robot.data.root_link_pos_w[:, :2], dim=1
        )
        prev_distance_to_gate = self.env._last_distance_to_goal
        progress_raw = prev_distance_to_gate - distance_to_gate 
        progress_to_gate = torch.clamp(progress_raw, -1.0, 1.0)
        self.env._last_distance_to_goal = distance_to_gate.detach()
        
        # Velocity towards gate (encourage maximum speed)
        drone_to_gate_vec = self.env._desired_pos_w - self.env._robot.data.root_link_pos_w
        drone_to_gate_vec_normalized = drone_to_gate_vec / (distance_to_gate.unsqueeze(1) + 1e-6)
        
        # Dot product of velocity with direction to gate (includes speed magnitude)
        vel_w = self.env._robot.data.root_com_lin_vel_w
        velocity_towards_gate = torch.sum(vel_w * drone_to_gate_vec_normalized, dim=1)
        velocity_reward = torch.clamp(velocity_towards_gate, -1.0, 6.0)  # Higher max for racing speeds
        

        # Penalize backward movement
        #backward_motion = -torch.clamp(-velocity_towards_gate, 0, 2.0)

        # ==================== OPTIMAL RACING LINE ====================
        # Reward for taking efficient paths between gates
        current_gate_idx = self.env._idx_wp
        next_gate_idx = (current_gate_idx + 1) % self.env._waypoints.shape[0]

        current_gate_pos = self.env._waypoints[current_gate_idx, :3]
        next_gate_pos = self.env._waypoints[next_gate_idx, :3]

        # Optimal path is the midpoint between current and next gate
        gate_to_gate_vec = next_gate_pos - current_gate_pos
        optimal_path_midpoint = current_gate_pos + (gate_to_gate_vec * 0.5)

        # Reward drone for staying close to optimal racing line
        drone_deviation = self.env._robot.data.root_link_pos_w - optimal_path_midpoint
        racing_line_reward = -torch.linalg.norm(drone_deviation[:, :2], dim=1) * 0.1
        
        # ==================== ORIENTATION ALIGNMENT ====================
        drone_forward_w = torch.zeros((self.num_envs, 3), device=self.device)
        drone_forward_w[:, 0] = -1.0  # Forward is +X in body frame
        
        rot_mat = matrix_from_quat(self.env._robot.data.root_quat_w)
        drone_forward_world = torch.bmm(rot_mat, drone_forward_w.unsqueeze(-1)).squeeze(-1)
        
        heading_alignment = torch.sum(drone_forward_world * drone_to_gate_vec_normalized, dim=1)
        heading_reward = torch.clamp(heading_alignment, -1.5, 1.0)
        
        # ==================== STABILITY AND CONTROL ====================
        euler_tuple = euler_xyz_from_quat(self.env._robot.data.root_quat_w)
        roll = euler_tuple[0]
        pitch = euler_tuple[1]
        
        tilt_penalty = torch.clamp(torch.abs(roll) + torch.abs(pitch) - 0.3, 0.15, 2.0)
        ang_vel_penalty = torch.linalg.norm(self.env._robot.data.root_ang_vel_b, dim=1) * 0.1


        # ==================== CONTROL SMOOTHNESS ====================
        # Penalize jerky control actions
        action_smoothness_penalty = torch.linalg.norm(
            self.env._actions - self.env._previous_actions, dim=1
        ) * 2.0
        
        # ==================== CRASH DETECTION ====================
        contact_forces = self.env._contact_sensor.data.net_forces_w
        crashed = (torch.norm(contact_forces, dim=-1) > 1e-8).squeeze(1).int()
        mask = (self.env.episode_length_buf > 100).int()
        self.env._crashed = self.env._crashed + crashed * mask
        crash_penalty = (self.env._crashed > 0).float()

        # ==================== HEIGHT MAINTENANCE ====================
        # Encourage staying near gate height
        target_height = self.env._desired_pos_w[:, 2]
        current_height = self.env._robot.data.root_link_pos_w[:, 2]
        height_error = torch.abs(current_height - target_height)
        #height_penalty = torch.clamp(height_error, 0.0, 2.0)
        height_penalty = torch.clamp(height_error - 0.5, 0.0, 1.0)  # Allow 0.5m variation

        # ==================== CRASH PREDICTION ====================
        # Add this after height maintenance section
        collision_risk = torch.zeros(self.num_envs, device=self.device)

        # Risk 1: High speed + excessive tilt
        high_speed = torch.linalg.norm(self.env._robot.data.root_com_lin_vel_w, dim=1) > 3.0
        excessive_tilt = (torch.abs(roll) > 0.5) | (torch.abs(pitch) > 0.5)
        collision_risk += (high_speed & excessive_tilt).float() * 2.0

        # Risk 2: Spinning out of control  
        excessive_ang_vel = torch.linalg.norm(self.env._robot.data.root_ang_vel_b, dim=1) > 2.0
        collision_risk += excessive_ang_vel.float() * 1.5

        # Risk 3: Too close to ground
        too_low = current_height < 0.3
        collision_risk += too_low.float() * 1.0

        ###
        # REPLACE your current velocity_reward line with:
        is_stable = (torch.abs(roll) < 0.4) & (torch.abs(pitch) < 0.4) & \
                    (torch.linalg.norm(self.env._robot.data.root_ang_vel_b, dim=1) < 1.5)

        velocity_reward = torch.where(
            is_stable,
            torch.clamp(velocity_towards_gate, -1.0, 6.0),  # Full speed when stable
            torch.clamp(velocity_towards_gate, -1.0, 3.0)   # Cap speed when unstable
        )


        # ==================== COMPUTE FINAL REWARD ====================
        if self.cfg.is_train:
            rewards = {
            "progress_gate": progress_to_gate * self.env.rew['progress_gate_reward_scale'],
            "velocity_forward": velocity_reward * self.env.rew['velocity_forward_reward_scale'],
            "gate_pass": gate_pass_bonus * self.env.rew['gate_pass_reward_scale'],
            "heading_alignment": heading_reward * self.env.rew['heading_alignment_reward_scale'],
            "tilt": -tilt_penalty * self.env.rew['tilt_reward_scale'],
            "ang_vel": -ang_vel_penalty * self.env.rew['ang_vel_reward_scale'],
            "crash": -crash_penalty * self.env.rew['crash_reward_scale'],
            "height": -height_penalty * self.env.rew['height_reward_scale'],
            "smoothness": -action_smoothness_penalty * self.env.rew['smoothness_reward_scale'],
            "racing_line": racing_line_reward * self.env.rew['racing_line_reward_scale'],
            "collision_risk": -collision_risk * self.env.rew.get('collision_risk_reward_scale', 1.5)
            }
            
            reward = torch.sum(torch.stack(list(rewards.values())), dim=0)
            reward = torch.where(
                self.env.reset_terminated,
                torch.ones_like(reward) * self.env.rew['death_cost'],
                reward
            )

            # Logging
            for key, value in rewards.items():
                self._episode_sums[key] += value
        else:
            reward = torch.zeros(self.num_envs, device=self.device)

        return reward

    def get_observations(self) -> Dict[str, torch.Tensor]:
        """Get observations for high-speed racing policy."""

        # ==================== DRONE STATE ====================
        drone_pos_w = self.env._robot.data.root_link_pos_w
        drone_lin_vel_b = self.env._robot.data.root_com_lin_vel_b
        drone_ang_vel_b = self.env._robot.data.root_ang_vel_b
        drone_quat_w = self.env._robot.data.root_quat_w
        
        euler_tuple = euler_xyz_from_quat(drone_quat_w)
        euler_angles = torch.stack(euler_tuple, dim=-1)

        # ==================== CURRENT GATE INFORMATION ====================
        current_gate_idx = self.env._idx_wp
        current_gate_pos_w = self.env._waypoints[current_gate_idx, :3]
        
        drone_pos_gate_frame = self.env._pose_drone_wrt_gate
        
        gate_pos_b, gate_quat_b = subtract_frame_transforms(
            self.env._robot.data.root_link_pos_w,
            self.env._robot.data.root_quat_w,
            current_gate_pos_w
        )
        
        dist_to_gate = torch.linalg.norm(gate_pos_b, dim=1, keepdim=True)
        gate_direction_b = gate_pos_b / (dist_to_gate + 1e-6)
        
        # ==================== NEXT GATE INFORMATION ====================
        next_gate_idx = (current_gate_idx + 1) % self.env._waypoints.shape[0]
        next_gate_pos_w = self.env._waypoints[next_gate_idx, :3]
        
        next_gate_pos_b, _ = subtract_frame_transforms(
            self.env._robot.data.root_link_pos_w,
            self.env._robot.data.root_quat_w,
            next_gate_pos_w
        )
        
        # ==================== PROGRESS INFORMATION ====================
        gates_passed_normalized = self.env._n_gates_passed.unsqueeze(1).float() / self.env._waypoints.shape[0]
        
        # ==================== PREVIOUS ACTIONS ====================
        prev_actions = self.env._previous_actions

        # ==================== ASSEMBLE OBSERVATION VECTOR ====================
        obs = torch.cat(
            [
                # Drone state (13 dims)
                drone_pos_w,                    # Position in world (3)
                drone_lin_vel_b,                # Linear velocity in body (3)
                drone_ang_vel_b,                # Angular velocity in body (3)
                euler_angles,                   # Roll, pitch, yaw (3)
                drone_quat_w[:, 3:4],          # Quaternion w component (1)
                
                # Current gate relative information (10 dims)
                gate_pos_b,                     # Gate position in body frame (3)
                gate_direction_b,               # Normalized direction to gate (3)
                dist_to_gate,                   # Distance to gate (1)
                drone_pos_gate_frame,           # Position in gate frame (3)
                
                # Next gate information (3 dims)
                next_gate_pos_b,                # Next gate position in body frame (3)
                
                # Progress and history (5 dims)
                gates_passed_normalized,        # Progress through course (1)
                prev_actions,                   # Previous actions (4)
            ],
            dim=-1,
        )
        
        observations = {"policy": obs}

        return observations

    def reset_idx(self, env_ids: Optional[torch.Tensor]):
        """Reset specific environments to initial states with randomization."""
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.env._robot._ALL_INDICES

        # Logging for training mode
        if self.cfg.is_train and hasattr(self, '_episode_sums'):
            extras = dict()
            for key in self._episode_sums.keys():
                episodic_sum_avg = torch.mean(self._episode_sums[key][env_ids])
                extras["Episode_Reward/" + key] = episodic_sum_avg / self.env.max_episode_length_s
                self._episode_sums[key][env_ids] = 0.0
            self.env.extras["log"] = dict()
            self.env.extras["log"].update(extras)
            extras = dict()
            extras["Episode_Termination/died"] = torch.count_nonzero(self.env.reset_terminated[env_ids]).item()
            extras["Episode_Termination/time_out"] = torch.count_nonzero(self.env.reset_time_outs[env_ids]).item()
            self.env.extras["log"].update(extras)

        # Call robot reset first
        self.env._robot.reset(env_ids)

        # Initialize model paths if needed
        if not self.env._models_paths_initialized:
            num_models_per_env = self.env._waypoints.size(0)
            model_prim_names_in_env = [f"{self.env.target_models_prim_base_name}_{i}" for i in range(num_models_per_env)]

            self.env._all_target_models_paths = []
            for env_path in self.env.scene.env_prim_paths:
                paths_for_this_env = [f"{env_path}/{name}" for name in model_prim_names_in_env]
                self.env._all_target_models_paths.append(paths_for_this_env)

            self.env._models_paths_initialized = True

        n_reset = len(env_ids)
        if n_reset == self.num_envs and self.num_envs > 1:
            self.env.episode_length_buf = torch.randint_like(
                self.env.episode_length_buf,
                high=int(self.env.max_episode_length)
            )

        # Reset action buffers
        self.env._actions[env_ids] = 0.0
        self.env._previous_actions[env_ids] = 0.0
        self.env._previous_yaw[env_ids] = 0.0
        self.env._motor_speeds[env_ids] = 0.0
        self.env._previous_omega_meas[env_ids] = 0.0
        self.env._previous_omega_err[env_ids] = 0.0
        self.env._omega_err_integral[env_ids] = 0.0

        # Reset joints state
        joint_pos = self.env._robot.data.default_joint_pos[env_ids]
        joint_vel = self.env._robot.data.default_joint_vel[env_ids]
        self.env._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        default_root_state = self.env._robot.data.default_root_state[env_ids]

        # ==================== TRAINING MODE RESET ====================
        if self.cfg.is_train:
            # Start from random waypoints for curriculum learning
            # Early in training, start from earlier gates; later, randomize more
            waypoint_indices = torch.zeros(n_reset, device=self.device, dtype=self.env._idx_wp.dtype)
            
            # Get starting gate information
            x0_wp = self.env._waypoints[waypoint_indices][:, 0]
            y0_wp = self.env._waypoints[waypoint_indices][:, 1]
            z_wp = self.env._waypoints[waypoint_indices][:, 2]
            theta = self.env._waypoints[waypoint_indices][:, -1]
            
            # Randomize starting position relative to gate
            # Position behind the gate with some variation
            x_local = torch.empty(n_reset, device=self.device).uniform_(-3.0, -1.0)  # 1-3m behind
            y_local = torch.empty(n_reset, device=self.device).uniform_(-0.8, 0.8)   # Lateral variation
            z_local = torch.empty(n_reset, device=self.device).uniform_(-0.3, 0.3)   # Vertical variation
            
            # Rotate local position to global frame
            cos_theta = torch.cos(theta)
            sin_theta = torch.sin(theta)
            x_rot = cos_theta * x_local - sin_theta * y_local
            y_rot = sin_theta * x_local + cos_theta * y_local
            
            initial_x = x0_wp - x_rot
            initial_y = y0_wp - y_rot
            initial_z = z_local + z_wp
            
            default_root_state[:, 0] = initial_x
            default_root_state[:, 1] = initial_y
            default_root_state[:, 2] = initial_z
            
            # Point drone towards the gate with some random yaw offset
            initial_yaw = torch.atan2(y0_wp - initial_y, x0_wp - initial_x)
            yaw_noise = torch.empty(n_reset, device=self.device).uniform_(-0.3, 0.3)  # ±17 degrees
            
            quat = quat_from_euler_xyz(
                torch.empty(n_reset, device=self.device).uniform_(-0.1, 0.1),  # Small roll variation
                torch.empty(n_reset, device=self.device).uniform_(-0.1, 0.1),  # Small pitch variation
                initial_yaw + yaw_noise
            )
            default_root_state[:, 3:7] = quat
            
            # Add small initial velocity towards gate for more dynamic starts
            initial_speed = torch.empty(n_reset, device=self.device).uniform_(0.0, 0.5)
            vel_direction = torch.stack([
                torch.cos(initial_yaw + yaw_noise),
                torch.sin(initial_yaw + yaw_noise),
                torch.zeros(n_reset, device=self.device)
            ], dim=1)
            default_root_state[:, 7:10] = vel_direction * initial_speed.unsqueeze(1)
            
        # ==================== PLAY MODE RESET ====================
        else:
            # Play mode: random position relative to initial waypoint
            x_local = torch.empty(1, device=self.device).uniform_(-3.0, -0.5)
            y_local = torch.empty(1, device=self.device).uniform_(-1.0, 1.0)

            x0_wp = self.env._waypoints[self.env._initial_wp, 0]
            y0_wp = self.env._waypoints[self.env._initial_wp, 1]
            theta = self.env._waypoints[self.env._initial_wp, -1]

            # Rotate local pos to global frame
            cos_theta, sin_theta = torch.cos(theta), torch.sin(theta)
            x_rot = cos_theta * x_local - sin_theta * y_local
            y_rot = sin_theta * x_local + cos_theta * y_local
            x0 = x0_wp - x_rot
            y0 = y0_wp - y_rot
            z0 = 0.05

            # Point drone towards the gate
            yaw0 = torch.atan2(y0_wp - y0, x0_wp - x0)

            default_root_state = self.env._robot.data.default_root_state[0].unsqueeze(0)
            default_root_state[:, 0] = x0
            default_root_state[:, 1] = y0
            default_root_state[:, 2] = z0

            quat = quat_from_euler_xyz(
                torch.zeros(1, device=self.device),
                torch.zeros(1, device=self.device),
                yaw0
            )
            default_root_state[:, 3:7] = quat
            waypoint_indices = self.env._initial_wp

        # Set waypoint indices and desired positions
        self.env._idx_wp[env_ids] = waypoint_indices

        self.env._desired_pos_w[env_ids, :2] = self.env._waypoints[waypoint_indices, :2].clone()
        self.env._desired_pos_w[env_ids, 2] = self.env._waypoints[waypoint_indices, 2].clone()

        self.env._last_distance_to_goal[env_ids] = torch.linalg.norm(
            self.env._desired_pos_w[env_ids, :2] - self.env._robot.data.root_link_pos_w[env_ids, :2], dim=1
        )
        self.env._n_gates_passed[env_ids] = 0

        # Write state to simulation
        self.env._robot.write_root_link_pose_to_sim(default_root_state[:, :7], env_ids)
        self.env._robot.write_root_com_velocity_to_sim(default_root_state[:, 7:], env_ids)

        # Reset variables
        self.env._yaw_n_laps[env_ids] = 0

        self.env._pose_drone_wrt_gate[env_ids], _ = subtract_frame_transforms(
            self.env._waypoints[self.env._idx_wp[env_ids], :3],
            self.env._waypoints_quat[self.env._idx_wp[env_ids], :],
            self.env._robot.data.root_link_state_w[env_ids, :3]
        )

        self.env._prev_x_drone_wrt_gate[env_ids] = -1.0  # Initialize behind gate

        self.env._crashed[env_ids] = 0