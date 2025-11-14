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

            # === ADD THESE TWO LINES ===
            self._episode_sums.setdefault("forward_progress", torch.zeros(self.num_envs, dtype=torch.float, device=self.device))
            self._episode_sums.setdefault("track_aligned", torch.zeros(self.num_envs, dtype=torch.float, device=self.device))
            # =============================

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

        """Compute rewards for drone racing through gates with directional enforcement."""

        """Compute rewards for drone racing through gates with minimal lap time."""

        """Compute rewards for drone racing through gates with minimal lap time."""
        # ==================== GATE PASSING DETECTION ====================
        # Distance to gate centre in gate frame
        dist_to_gate_center = torch.linalg.norm(self.env._pose_drone_wrt_gate, dim=1)

        # ---- 1. Forward-only progress (prevents reward-hacking by oscillation) ----
        # Velocity in gate frame (already computed later – reuse here)
        gate_rot_mat = matrix_from_quat(self.env._waypoints_quat[self.env._idx_wp, :])
        vel_w = self.env._robot.data.root_com_lin_vel_w.unsqueeze(-1)          # (N,3,1)
        vel_in_gate_frame = torch.bmm(gate_rot_mat.transpose(1, 2), vel_w).squeeze(-1)  # (N,3)

        # Positive X-velocity in gate frame → forward progress
        forward_vel_gate = torch.clamp(vel_in_gate_frame[:, 0], min=0.0)   # only count forward

        # ---- 2. Track-aligned velocity (encourage racing-line optimisation) ----
        # Desired heading = direction from drone to next gate in world frame
        drone_to_gate_vec = self.env._desired_pos_w - self.env._robot.data.root_link_pos_w
        dist_to_gate = torch.linalg.norm(drone_to_gate_vec[:, :2], dim=1, keepdim=True) + 1e-6
        track_dir = drone_to_gate_vec[:, :2] / dist_to_gate

        # Project world velocity onto track direction (ignore Z)
        vel_w_xy = self.env._robot.data.root_com_lin_vel_w[:, :2]
        track_aligned_vel = torch.sum(vel_w_xy * track_dir, dim=1)       # scalar per env

        # ------------------- RELAXED GATE PASS -------------------
        crossed_gate_plane = self.env._pose_drone_wrt_gate[:, 0] > -0.3   # allow a little “behind”
        within_gate_bounds = (
            (torch.abs(self.env._pose_drone_wrt_gate[:, 1]) < 0.8) &     # wider tolerance
            (torch.abs(self.env._pose_drone_wrt_gate[:, 2]) < 0.8)
        )
        was_behind_gate = self.env._prev_x_drone_wrt_gate < 0.0
        moving_forward_towards_gate = vel_in_gate_frame[:, 0] > -0.5   # accept slight backward slip

        gate_passed = crossed_gate_plane & within_gate_bounds & was_behind_gate & moving_forward_towards_gate

        # Update previous x-position
        self.env._prev_x_drone_wrt_gate = self.env._pose_drone_wrt_gate[:, 0].clone()

        # ------------------- UPDATE WAYPOINT -------------------
        ids_gate_passed = torch.where(gate_passed)[0]
        if len(ids_gate_passed) > 0:
            self.env._n_gates_passed[ids_gate_passed] += 1
            self.env._idx_wp[ids_gate_passed] = (self.env._idx_wp[ids_gate_passed] + 1) % self.env._waypoints.shape[0]

            self.env._desired_pos_w[ids_gate_passed, :2] = self.env._waypoints[self.env._idx_wp[ids_gate_passed], :2]
            self.env._desired_pos_w[ids_gate_passed, 2] = self.env._waypoints[self.env._idx_wp[ids_gate_passed], 2]

            self.env._pose_drone_wrt_gate[ids_gate_passed], _ = subtract_frame_transforms(
                self.env._waypoints[self.env._idx_wp[ids_gate_passed], :3],
                self.env._waypoints_quat[self.env._idx_wp[ids_gate_passed], :],
                self.env._robot.data.root_link_state_w[ids_gate_passed, :3]
            )

        # ==================== PROGRESS METRICS ====================
        distance_to_gate = torch.linalg.norm(
            self.env._desired_pos_w[:, :2] - self.env._robot.data.root_link_pos_w[:, :2], dim=1
        )
        prev_distance_to_gate = self.env._last_distance_to_goal
        progress_raw = prev_distance_to_gate - distance_to_gate
        self.env._last_distance_to_goal = distance_to_gate.detach()
        progress_to_gate = torch.clamp(progress_raw, -1.0, 1.0)

        # Velocity towards gate
        drone_to_gate_vec_normalized = drone_to_gate_vec / (distance_to_gate.unsqueeze(1) + 1e-6)
        velocity_towards_gate = torch.sum(self.env._robot.data.root_com_lin_vel_w * drone_to_gate_vec_normalized, dim=1)
        velocity_reward = torch.clamp(velocity_towards_gate, -1.0, 6.0)

        # Extra penalty for moving backwards relative to the current gate
        backward_motion = -torch.clamp(-velocity_towards_gate, 0, 2.0)

        # Gate passing bonus
        gate_pass_bonus = gate_passed.float() * 10.0

        # ---- NEW REWARDS -------------------------------------------------
        # 1. Forward-only progress (scaled by forward velocity in gate frame)
        forward_progress_reward = forward_vel_gate * self.env.rew.get('forward_progress_reward_scale', 0.5)

        # 2. Track-aligned velocity (encourage staying on the racing line)
        track_aligned_reward = torch.clamp(track_aligned_vel, 0.0, 8.0) * self.env.rew.get('track_aligned_reward_scale', 0.3)

        # ==================== ORIENTATION ALIGNMENT ====================
        drone_forward_w = torch.zeros((self.num_envs, 3), device=self.device)
        drone_forward_w[:, 0] = 1.0
        rot_mat = matrix_from_quat(self.env._robot.data.root_quat_w)
        drone_forward_world = torch.bmm(rot_mat, drone_forward_w.unsqueeze(-1)).squeeze(-1)
        heading_alignment = torch.sum(drone_forward_world * drone_to_gate_vec_normalized, dim=1)
        heading_reward = torch.clamp(heading_alignment, -1.5, 1.0)

        # ==================== STABILITY AND CONTROL ====================
        euler_tuple = euler_xyz_from_quat(self.env._robot.data.root_quat_w)
        roll, pitch = euler_tuple[0], euler_tuple[1]
        tilt_penalty = torch.clamp(torch.abs(roll) + torch.abs(pitch) - 0.3, 0.0, 2.0)
        ang_vel_penalty = torch.linalg.norm(self.env._robot.data.root_ang_vel_b, dim=1) * 0.1

        # ==================== CRASH DETECTION ====================
        contact_forces = self.env._contact_sensor.data.net_forces_w
        crashed = (torch.norm(contact_forces, dim=-1) > 1e-8).squeeze(1).int()
        mask = (self.env.episode_length_buf > 100).int()
        self.env._crashed = self.env._crashed + crashed * mask
        crash_penalty = (self.env._crashed > 0).float()

        # ==================== HEIGHT MAINTENANCE ====================
        target_height = self.env._desired_pos_w[:, 2]
        current_height = self.env._robot.data.root_link_pos_w[:, 2]
        height_error = torch.abs(current_height - target_height)
        height_penalty = torch.clamp(height_error, 0.0, 2.0)

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
                "backward": backward_motion * self.env.rew['backward_reward_scale'],
                # ---- NEW TERMS -------------------------------------------------
                "forward_progress": forward_progress_reward * self.env.rew['forward_progress_reward_scale'], 
                "track_aligned": track_aligned_reward * self.env.rew['track_aligned_reward_scale'],
            }

            reward = torch.sum(torch.stack(list(rewards.values())), dim=0)

            # Death cost for terminated episodes
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
        """Rich observation: drone state + gate-relative pose + history."""

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
        
        # Relative position to current gate in gate frame
        drone_pos_gate_frame = self.env._pose_drone_wrt_gate
        
        # Relative position to current gate in body frame
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
        
        # ==================== GATE-TO-GATE SEGMENT INFORMATION ====================
        gate_to_gate_vec = next_gate_pos_w - current_gate_pos_w
        gate_to_gate_vec_normalized = gate_to_gate_vec / (torch.linalg.norm(gate_to_gate_vec, dim=1, keepdim=True) + 1e-6)
        
        # Drone position relative to current gate
        drone_from_gate_vec = self.env._robot.data.root_link_pos_w - current_gate_pos_w
        
        # Projection onto gate-to-gate segment
        dot_product = torch.sum(drone_from_gate_vec * gate_to_gate_vec_normalized, dim=1, keepdim=True)
        segment_progress = dot_product / (torch.linalg.norm(gate_to_gate_vec, dim=1, keepdim=True) + 1e-6)
        
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
                
                # Gate-to-gate segment information (4 dims)
                gate_to_gate_vec_normalized,    # Normalized segment direction (3)
                segment_progress,               # Progress along segment (1)
                
                # Progress and history (5 dims)
                gates_passed_normalized,        # Progress through course (1)
                prev_actions,                   # Previous actions (4)
            ],
            dim=-1,
        )
        
        observations = {"policy": obs}

        return observations

    def reset_idx(self, env_ids: Optional[torch.Tensor]):
        """Reset specific environments to initial states with curriculum."""
        """Reset specific environments to a fixed starting position behind gate 0."""
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

        # === FIXED START: 2.0m behind gate 0, no noise ===
        idx_dtype = self.env._idx_wp.dtype
        waypoint_indices = torch.zeros(n_reset, dtype=idx_dtype, device=self.device)

        # Local offset in gate frame - BEHIND GATE 0
        x_local = torch.full((n_reset,), -2.0, device=self.device)  # 2m behind gate 0
        y_local = torch.zeros(n_reset, device=self.device)
        z_local = torch.full((n_reset,), 0.05, device=self.device)  # Slightly above ground

        # Gate 0 position and yaw (EXPLICITLY USE GATE 0)
        wp_pos = self.env._waypoints[0, :3]  # Explicit gate 0 position
        theta = self.env._waypoints[0, -1]   # Explicit gate 0 yaw

        # Rotate local pos to global frame
        cos_theta, sin_theta = torch.cos(theta), torch.sin(theta)
        x_rot = cos_theta * x_local - sin_theta * y_local
        y_rot = sin_theta * x_local + cos_theta * y_local
        x0 = wp_pos[0] - x_rot  # CORRECTED: Use gate 0 position directly
        y0 = wp_pos[1] - y_rot  # CORRECTED: Use gate 0 position directly
        z0 = z_local  # CORRECTED: Use tensor instead of float

        # Point drone towards gate 0
        yaw0 = torch.atan2(wp_pos[1] - y0, wp_pos[0] - x0)  # CORRECTED: Point to gate 0

        # Apply to root state
        default_root_state[:, 0] = x0
        default_root_state[:, 1] = y0
        default_root_state[:, 2] = z0

        quat = quat_from_euler_xyz(
            torch.zeros(n_reset, device=self.device),
            torch.zeros(n_reset, device=self.device),
            yaw0
        )
        default_root_state[:, 3:7] = quat

        # Small forward velocity (0.5 m/s) towards gate 0
        initial_speed = torch.ones(n_reset, device=self.device) * 0.5
        vel_dir = torch.stack([
            torch.cos(yaw0),
            torch.sin(yaw0),
            torch.zeros(n_reset, device=self.device)
        ], dim=1)
        default_root_state[:, 7:10] = vel_dir * initial_speed.unsqueeze(1)

        # === FINALIZE RESET ===
        self.env._idx_wp[env_ids] = waypoint_indices

        self.env._desired_pos_w[env_ids, :2] = self.env._waypoints[waypoint_indices, :2].clone()
        self.env._desired_pos_w[env_ids, 2] = self.env._waypoints[waypoint_indices, 2].clone()

        # Initialize tracking buffers - CORRECTED
        initial_pos = torch.stack([x0, y0, z0], dim=1)
        self.env._last_distance_to_goal[env_ids] = torch.linalg.norm(
            self.env._desired_pos_w[env_ids] - initial_pos, dim=1
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
            initial_pos  # Use the actual starting position
        )

        self.env._prev_x_drone_wrt_gate[env_ids] = -1.0  # Initialize behind gate

        self.env._crashed[env_ids] = 0

        # Reset tracking variables for the new reward strategy
        if hasattr(self, '_prev_track_progress'):
            self._prev_track_progress[env_ids] = 0.0
        if hasattr(self, '_gate_lockout_timer'):
            self._gate_lockout_timer[env_ids] = 0