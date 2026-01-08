#!/usr/bin/env python3
import os
import glob
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped

def quat_wxyz_to_rotmat(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Return 3x3 rotation matrix from quaternion (w, x, y, z)."""
    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) + 1e-12
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n #normalize the quaternions
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = np.array([
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)],
    ], dtype=np.float64)
    return R

def yaw_to_quat_wxyz(yaw: float)-> Tuple[float,float,float,float]:
    """Pure yaw quaternion"""
    cy=np.cos(yaw*0.5)
    sy=np.sin(yaw*0.5)
    return(cy,0.0,0.0,sy)

def world_to_body_points(
    p_w: np.ndarray,
    body_pos_w: np.ndarray,
    body_rot_w: np.ndarray,
) -> np.ndarray:
    """
    Transform world points (N,3) into body frame using:
      p_b = R_wb^T * (p_w - p_body)
    Here we use body_rot_w as R_bw (body->world), so inverse is transpose.
    """
    return (body_rot_w.T @ (p_w - body_pos_w).T).T # If body_rot_w is body->world, then world->body is R^T.

def clamp(x: float, lo: float, hi: float) -> float:
    return float(np.minimum(np.maximum(x, lo), hi))

@dataclass
class Waypoint:
    pos_w: np.ndarray #(3,)
    yaw_w: float #radians

def generate_square_track(center_xy: Tuple[float, float], z: float, side_m: float) -> List[Waypoint]:
    """
    4-corner square loop (clockwise).
    """
    cx, cy = center_xy
    s = side_m / 2.0
    corners = [
        (cx - s, cy - s),
        (cx + s, cy - s),
        (cx + s, cy + s),
        (cx - s, cy + s),
    ]
    wps: List[Waypoint] = []
    for i in range(len(corners)):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % len(corners)]
        yaw = np.arctan2((y1 - y0), (x1 - x0))
        wps.append(Waypoint(pos_w=np.array([x0, y0, z], dtype=np.float64), yaw_w=float(yaw)))
    return wps

def get_local_square_corners(square_size_m: float) -> np.ndarray:
    """
    4 corners in the waypoint's local frame.
    IMPORTANT: The ordering must match training.
    """
    h = square_size_m / 2.0
    # Typical "square in XY plane, z=0"
    return np.array([
        [ h,  h, 0.0],
        [ h, -h, 0.0],
        [-h, -h, 0.0],
        [-h,  h, 0.0],
    ], dtype=np.float64)

def find_latest_checkpoint_folder(root_dir: str) -> Optional[str]:
    """
    root_dir .../ese651_project/checkpoints/quadcopter/
    returns newest subdir containing best_model.pt
    """
    if not os.path.isdir(root_dir):
        return None
    candidates = []
    for d in glob.glob(os.path.join(root_dir, "*")):
        if os.path.isdir(d) and os.path.isfile(os.path.join(d, "best_model.pt")):
            candidates.append(d)
    if not candidates:
        return None
    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates[0]

class RLAgentNode(Node):
    def __init__(self):
        super().__init__('rl_agent_node')
        self.get_logger().info('rl_agent_node.py up and running')

         # ---- Params: file paths
        self.declare_parameter("checkpoint_dir", "")  # full path to timestamp folder containing best_model.pt
        self.declare_parameter("checkpoints_root", "")  # optional: root folder to auto-pick latest
        self.declare_parameter("device", "cpu")

        # ---- Params: topics
        self.declare_parameter("vicon_pose_topic", "/vicon/uav/uav/pose")
        self.declare_parameter("vicon_twist_topic", "/vicon/uav/uav/twist")
        self.declare_parameter("rc_out_topic", "/rc_aetr_aux")  # String expected by uros_bridge_node.cpp

        # ---- Params: timing
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("vicon_timeout_s", 0.2)
        self.declare_parameter("publish_when_disarmed", True)

        # ---- Params: action->RC mapping (µs)
        self.declare_parameter("roll_scale_us", 200.0)
        self.declare_parameter("pitch_scale_us", 200.0)
        self.declare_parameter("yaw_scale_us", 150.0)
        self.declare_parameter("throttle_min_us", 1000.0)
        self.declare_parameter("throttle_max_us", 1700.0)
        self.declare_parameter("center_us", 1500.0)

        # per-axis sign flips (set to -1 if direction is inverted)
        self.declare_parameter("roll_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)
        self.declare_parameter("yaw_sign", 1.0)

        # ---- Params: AUX channels (µs)
        # AUX1 arming range is 1050-1150. Pick one arming value in that window.
        self.declare_parameter("aux1_arm_us", 1100.0)
        self.declare_parameter("aux1_disarm_us", 900.0)
        self.declare_parameter("aux_defaults_us", [1500, 1500, 1500])  # AUX2,AUX3,AUX4 defaults

        # ---- Params: policy smoothing
        self.declare_parameter("action_smoothing_alpha", 0.0)  # 0 = off, else [0..1], higher=more smoothing

        # ---- Params: waypoint track (virtual)
        self.declare_parameter("track_mode", "square")  # square|hover
        self.declare_parameter("square_center_xy", [0.0, 0.0])
        self.declare_parameter("square_side_m", 5.0)
        self.declare_parameter("track_z_m", 1.2)

        # This is the “gate square” size used to create corners. MUST match training.
        self.declare_parameter("target_square_size_m", 2.0)

        # gate/waypoint progression
        self.declare_parameter("wp_reach_radius_m", 0.4)  # when within this radius, advance to next waypoint
        self.declare_parameter("wp_use_yaw_heading", True)

        # ---- Internal state
        self._pose_w: Optional[np.ndarray] = None
        self._quat_wxyz: Optional[np.ndarray] = None
        self._twist_w: Optional[np.ndarray] = None  # linear vel in world
        self._last_vicon_time = 0.0

        self._last_action = np.zeros(4, dtype=np.float64)

        self._armed = True  # software gate;

        self._kill = False

         # ---- Load policy
        ckpt_dir = self.get_parameter("checkpoint_dir").get_parameter_value().string_value
        ckpt_root = self.get_parameter("checkpoints_root").get_parameter_value().string_value

        if (not ckpt_dir) and ckpt_root:
            latest = find_latest_checkpoint_folder(ckpt_root)
            if latest is None:
                raise RuntimeError(f"No checkpoint folder with best_model.pt found under checkpoints_root={ckpt_root}")
            ckpt_dir = latest
            self.get_logger().info(f"Auto-selected latest checkpoint_dir: {ckpt_dir}")

        if not ckpt_dir:
            raise RuntimeError("You must set parameter checkpoint_dir (timestamp folder containing best_model.pt) "
                                "or provide checkpoints_root to auto-select latest.")

        policy_path = os.path.join(ckpt_dir, "best_model.pt")
        if not os.path.isfile(policy_path):
            raise RuntimeError(f"Policy file not found: {policy_path}")
        
        device_str = self.get_parameter("device").get_parameter_value().string_value
        self._device = torch.device(device_str)
        self.get_logger().info(f"Loading policy: {policy_path} on {self._device}")

        self._policy = torch.jit.load(policy_path, map_location=self._device)
        self._policy.eval()

        # ---- Waypoints
        track_mode = self.get_parameter("track_mode").value
        z = float(self.get_parameter("track_z_m").value)

        if track_mode == "hover":
            # a single waypoint at the origin (or center_xy) for “hover around here”
            center = self.get_parameter("square_center_xy").value
            wps = [Waypoint(pos_w=np.array([center[0], center[1], z], dtype=np.float64), yaw_w=0.0)]
        else:
            center = self.get_parameter("square_center_xy").value
            side = float(self.get_parameter("square_side_m").value)
            wps = generate_square_track((center[0], center[1]), z=z, side_m=side)

        self._waypoints = wps
        self._wp_idx = 0

        # local square corners (4,3) in waypoint frame
        sq_size = float(self.get_parameter("target_square_size_m").value)
        self._local_square = get_local_square_corners(sq_size)

        # ---- ROS I/O
        pose_topic = self.get_parameter("vicon_pose_topic").value
        twist_topic = self.get_parameter("vicon_twist_topic").value
        out_topic = self.get_parameter("rc_out_topic").value

        self._sub_pose = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self._sub_twist = self.create_subscription(TwistStamped, twist_topic, self._on_twist, 10)
        self._sub_kill = self.create_subscription(Bool, "/rl/kill", self._on_kill, 10)
        self._pub_rc = self.create_publisher(String, out_topic, 10)

        # ---- Timer loop
        rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"rl_agent_node running. Vicon pose={pose_topic}, twist={twist_topic}, out={out_topic}, rate={rate_hz} Hz"
        )

        def _on_pose(self, msg: PoseStamped):
            p=msg.pose.position
            q=msg.pose.orientation
            self._pose_w=np.array([p.x, p.y, p.z], dtype=np.float64)
            self._quat_wxyz = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)
            self._last_vicon_time = time.time()

        def _on_twist(self, msg: TwistStamped):
            v = msg.twist.linear
            self._twist_w = np.array([v.x, v.y, v.z], dtype=np.float64)
            self._last_vicon_time = time.time()

        def _vicon_ok(self) -> bool:
            timeout_s = float(self.get_parameter("vicon_timeout_s").value)
            return (time.time() - self._last_vicon_time) <= timeout_s and (self._pose_w is not None) and (self._quat_wxyz is not None)
        
        def _on_kill(self, msg: Bool):
            if msg.data:
                self.get_logger().warn("KILL received: publishing DISARM and shutting down.")
                self._kill = True

        def _advance_waypoint_if_needed(self, pos_w: np.ndarray):
            if len(self._waypoints) <= 1:
                return
            r = float(self.get_parameter("wp_reach_radius_m").value)
            wp = self._waypoints[self._wp_idx].pos_w
            if np.linalg.norm((wp - pos_w)[:2]) < r:
                self._wp_idx = (self._wp_idx + 1) % len(self._waypoints)

        def _compute_waypoint_corners_body(self, body_pos_w: np.ndarray, body_R_bw: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
            """
            Returns:
            corners_curr_b_flat (12,)
            corners_next_b_flat (12,)
            """
            curr = self._waypoints[self._wp_idx]
            nxt = self._waypoints[(self._wp_idx + 1) % len(self._waypoints)]

            def corners_world(wp: Waypoint) -> np.ndarray:
                # waypoint orientation (yaw-only or fixed)
                if bool(self.get_parameter("wp_use_yaw_heading").value):
                    qw, qx, qy, qz = yaw_to_quat_wxyz(wp.yaw_w)
                else:
                    qw, qx, qy, qz = (1.0, 0.0, 0.0, 0.0)
                R = quat_wxyz_to_rotmat(qw, qx, qy, qz)  # waypoint body->world
                # local_square (4,3) * R^T + pos  (matches your sim: bmm(local_square, rot.T) + pos)
                return (self._local_square @ R.T) + wp.pos_w.reshape(1, 3)

            verts_curr_w = corners_world(curr)  # (4,3)
            verts_next_w = corners_world(nxt)

            verts_curr_b = world_to_body_points(verts_curr_w, body_pos_w, body_R_bw)  # (4,3)
            verts_next_b = world_to_body_points(verts_next_w, body_pos_w, body_R_bw)

            return verts_curr_b.reshape(-1), verts_next_b.reshape(-1)
        
        def _build_obs(self) -> Optional[np.ndarray]:
            if not self._vicon_ok():
                return None

            pos_w = self._pose_w
            qw, qx, qy, qz = self._quat_wxyz.tolist()

            # body rotation matrix (body->world)
            R_bw = quat_wxyz_to_rotmat(qw, qx, qy, qz)

            # velocity in body frame (need v in world)
            if self._twist_w is None:
                # If you don’t have Twist from Vicon, you can add finite-diff here.
                v_w = np.zeros(3, dtype=np.float64)
            else:
                v_w = self._twist_w

            v_b = (R_bw.T @ v_w.reshape(3, 1)).reshape(3,)  # world->body

            # waypoint progression
            self._advance_waypoint_if_needed(pos_w)

            # corners in body frame
            corners_curr_b, corners_next_b = self._compute_waypoint_corners_body(pos_w, R_bw)

            # attitude rotation matrix flattened
            attitude_flat = R_bw.reshape(-1)  # row-major flatten

            obs = np.concatenate([v_b, attitude_flat, corners_curr_b, corners_next_b], axis=0)
            if obs.shape[0] != 36:
                self.get_logger().error(f"Obs dim mismatch: got {obs.shape[0]}, expected 36")
                return None
            return obs.astype(np.float32)
        
        def _actions_to_aetr_us(self, a: np.ndarray) -> Tuple[int, int, int, int]:
            """
            Map policy actions [-1,1] to AETR in microseconds.
            a = [thrust_norm, roll_rate_norm, pitch_rate_norm, yaw_rate_norm]
            """
            center = float(self.get_parameter("center_us").value)

            roll_s = float(self.get_parameter("roll_scale_us").value) * float(self.get_parameter("roll_sign").value)
            pitch_s = float(self.get_parameter("pitch_scale_us").value) * float(self.get_parameter("pitch_sign").value)
            yaw_s = float(self.get_parameter("yaw_scale_us").value) * float(self.get_parameter("yaw_sign").value)

            tmin = float(self.get_parameter("throttle_min_us").value)
            tmax = float(self.get_parameter("throttle_max_us").value)

            # A/E/R centered
            A = center + roll_s * float(a[1])
            E = center + pitch_s * float(a[2])
            R = center + yaw_s * float(a[3])

            # Throttle non-centered
            # Map [-1,1] -> [tmin, tmax]
            T = tmin + ((float(a[0]) + 1.0) * 0.5) * (tmax - tmin)

            A = int(round(clamp(A, 1000.0, 2000.0)))
            E = int(round(clamp(E, 1000.0, 2000.0)))
            T = int(round(clamp(T, 1000.0, 2000.0)))
            R = int(round(clamp(R, 1000.0, 2000.0)))
            return A, E, T, R
        
        def _publish_rc(self, A: int, E: int, T: int, R: int, aux1: int, aux234: List[int]):
            """
            Your uros_bridge_node.cpp expects a list-string; we’ll publish as:
            "A E T R AUX1 AUX2 AUX3 AUX4"
            """
            msg = String()
            aux2, aux3, aux4 = aux234
            msg.data = f"{A} {E} {T} {R} {aux1} {aux2} {aux3} {aux4}"
            self._pub_rc.publish(msg)

        def _tick(self):
            publish_when_disarmed = bool(self.get_parameter("publish_when_disarmed").value)
            aux_defaults = [int(x) for x in self.get_parameter("aux_defaults_us").value]

            if self._kill:
                A = E = R = int(self.get_parameter("center_us").value)
                T = int(self.get_parameter("throttle_min_us").value)
                aux1 = int(self.get_parameter("aux1_disarm_us").value)
                aux_defaults = [int(x) for x in self.get_parameter("aux_defaults_us").value]
                self._publish_rc(A, E, T, R, aux1, aux_defaults)
                # stop this node
                rclpy.shutdown()
                return
            
            if not self._vicon_ok():
                # Fail-safe: disarm + neutral
                A = E = R = int(self.get_parameter("center_us").value)
                T = int(self.get_parameter("throttle_min_us").value)
                aux1 = int(self.get_parameter("aux1_disarm_us").value)
                self._publish_rc(A, E, T, R, aux1, aux_defaults)
                return

            obs = self._build_obs()
            if obs is None:
                # same failsafe
                A = E = R = int(self.get_parameter("center_us").value)
                T = int(self.get_parameter("throttle_min_us").value)
                aux1 = int(self.get_parameter("aux1_disarm_us").value)
                self._publish_rc(A, E, T, R, aux1, aux_defaults)
                return

            # Run policy
            obs_t = torch.from_numpy(obs).to(self._device).unsqueeze(0)
            with torch.no_grad():
                act_t = self._policy(obs_t)  # (1,4)
            act = act_t.squeeze(0).detach().cpu().numpy().astype(np.float64)
            act = np.clip(act, -1.0, 1.0)
            # Optional smoothing
            alpha = float(self.get_parameter("action_smoothing_alpha").value)
            if alpha > 0.0:
                act = alpha * self._last_action + (1.0 - alpha) * act
            self._last_action = act.copy()

            # Arm logic:
            if self._armed:
                aux1 = int(self.get_parameter("aux1_arm_us").value)
            else:
                aux1 = int(self.get_parameter("aux1_disarm_us").value)

            # If disarmed and you don’t want to spam, optionally stop publishing.
            if (not self._armed) and (not publish_when_disarmed):
                return

            A, E, T, R = self._actions_to_aetr_us(act)
            self._publish_rc(A, E, T, R, aux1, aux_defaults)

        def arm(self):
            self.get_logger().warn("ARMED (software gate). AUX1 will be set to arm_us in outgoing messages.")
            self._armed = True

        def disarm(self):
            self.get_logger().warn("DISARMED (software gate). AUX1 will be set to disarm_us in outgoing messages.")
            self._armed = False

def main():
    rclpy.init()
    node = RLAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
