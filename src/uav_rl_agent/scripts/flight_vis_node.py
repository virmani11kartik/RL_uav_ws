#!/usr/bin/env python3
"""
flight_vis_node.py - 3D Simulation Style Visualization

Real-time 3D drone simulator visualization with smooth rendering.

Subscribes to:
  /uav_0/pose   (geometry_msgs/msg/PoseStamped)

Features:
  - 3D drone model with proper orientation
  - Smooth camera following
  - Trail visualization
  - Ground plane with grid
  - Real-time telemetry overlay

Works on ROS 2 Jazzy (Python 3.12).
"""

import threading
import time
from dataclasses import dataclass
from typing import Optional, Deque
from collections import deque

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped


def quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to rotation matrix."""
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=float)


def quat_to_euler(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1, 1))

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class FlightVisNode(Node):
    def __init__(self):
        super().__init__("flight_vis_node")

        self.pose_topic = self.declare_parameter("pose_topic", "/uav_0/pose").value
        self.history_len = int(self.declare_parameter("history_len", 1000).value)
        self.plot_hz = float(self.declare_parameter("plot_hz", 60.0).value)
        self.trail_length = int(self.declare_parameter("trail_length", 500).value)

        self._lock = threading.Lock()
        self._t0_wall = time.time()
        self._latest: Optional[PoseSample] = None
        self._hist: Deque[PoseSample] = deque(maxlen=self.history_len)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_pose = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_cb,
            qos,
        )

        self.get_logger().info(f"3D Flight Visualizer - Subscribing to {self.pose_topic}")

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation

        t = time.time() - self._t0_wall
        sample = PoseSample(
            t=t,
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            qx=float(q.x),
            qy=float(q.y),
            qz=float(q.z),
            qw=float(q.w),
        )

        with self._lock:
            self._latest = sample
            self._hist.append(sample)

    def get_snapshot(self):
        with self._lock:
            return self._latest, list(self._hist)


class DroneModel:
    """Creates a 3D drone mesh."""
    
    def __init__(self, arm_length=0.3, body_size=0.1):
        self.arm_length = arm_length
        self.body_size = body_size
        
    def create_mesh(self):
        """Create mesh data for the drone."""
        verts = []
        faces = []
        colors = []
        
        L = self.arm_length
        B = self.body_size
        
        # Central body (cube)
        body_verts = np.array([
            [-B, -B, -B/2], [B, -B, -B/2], [B, B, -B/2], [-B, B, -B/2],
            [-B, -B, B/2], [B, -B, B/2], [B, B, B/2], [-B, B, B/2]
        ])
        
        body_faces = np.array([
            [0,1,2], [0,2,3], [4,5,6], [4,6,7],
            [0,1,5], [0,5,4], [2,3,7], [2,7,6],
            [0,3,7], [0,7,4], [1,2,6], [1,6,5]
        ])
        
        verts.append(body_verts)
        faces.append(body_faces)
        colors.extend([[0.2, 0.2, 0.2, 1]] * len(body_faces))
        
        # Four arms with motors
        arm_positions = [
            (L/np.sqrt(2), L/np.sqrt(2), 0),    # Front right
            (-L/np.sqrt(2), L/np.sqrt(2), 0),   # Front left
            (-L/np.sqrt(2), -L/np.sqrt(2), 0),  # Back left
            (L/np.sqrt(2), -L/np.sqrt(2), 0),   # Back right
        ]
        
        arm_colors = [
            [1, 0, 0, 1],  # Red
            [0, 1, 0, 1],  # Green
            [0, 0, 1, 1],  # Blue
            [1, 1, 0, 1],  # Yellow
        ]
        
        base_idx = len(body_verts)
        
        for i, (ax, ay, az) in enumerate(arm_positions):
            # Motor (cylinder approximation)
            motor_r = 0.04
            motor_h = 0.03
            n_sides = 8
            
            motor_verts = []
            for j in range(n_sides):
                angle = 2 * np.pi * j / n_sides
                x = motor_r * np.cos(angle)
                y = motor_r * np.sin(angle)
                motor_verts.append([ax + x, ay + y, az - motor_h/2])
                motor_verts.append([ax + x, ay + y, az + motor_h/2])
            
            motor_verts = np.array(motor_verts)
            start_idx = base_idx + i * len(motor_verts)
            
            motor_faces = []
            for j in range(n_sides):
                j1 = (j + 1) % n_sides
                # Side faces
                motor_faces.append([
                    start_idx + j*2,
                    start_idx + j*2 + 1,
                    start_idx + j1*2 + 1
                ])
                motor_faces.append([
                    start_idx + j*2,
                    start_idx + j1*2 + 1,
                    start_idx + j1*2
                ])
            
            verts.append(motor_verts)
            faces.append(np.array(motor_faces))
            colors.extend([arm_colors[i]] * len(motor_faces))
        
        # Concatenate all vertices and faces
        verts = np.vstack(verts)
        faces = np.vstack(faces)
        colors = np.array(colors)
        
        return verts, faces, colors


class Simulator3DWindow(gl.GLViewWidget):
    def __init__(self, node: FlightVisNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('3D Flight Simulator')
        
        # Camera settings
        self.opts['distance'] = 3
        self.opts['fov'] = 60
        self.opts['elevation'] = 30
        self.opts['azimuth'] = 45
        
        # Ground grid
        grid = gl.GLGridItem()
        grid.setSize(20, 20)
        grid.setSpacing(0.5, 0.5)
        self.addItem(grid)
        
        # Axes
        axis = gl.GLAxisItem()
        axis.setSize(1, 1, 1)
        self.addItem(axis)
        
        # Create drone model
        drone_model = DroneModel(arm_length=0.3, body_size=0.08)
        verts, faces, colors = drone_model.create_mesh()
        
        self.drone_mesh = gl.GLMeshItem(
            vertexes=verts,
            faces=faces,
            faceColors=colors,
            smooth=True,
            drawEdges=False,
            shader='shaded'
        )
        self.addItem(self.drone_mesh)
        
        # Trail line
        self.trail = gl.GLLinePlotItem(
            pos=np.zeros((1, 3)),
            color=(0, 1, 1, 0.8),
            width=2,
            antialias=True
        )
        self.addItem(self.trail)
        
        # Current position marker
        self.marker = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3)),
            color=(1, 0, 0, 1),
            size=8
        )
        self.addItem(self.marker)
        
        # Telemetry text overlay
        self.telemetry_label = QtWidgets.QLabel(self)
        self.telemetry_label.setStyleSheet("""
            QLabel {
                background-color: rgba(0, 0, 0, 180);
                color: #00ff00;
                font-family: 'Courier New', monospace;
                font-size: 12px;
                padding: 10px;
                border-radius: 5px;
            }
        """)
        self.telemetry_label.move(10, 10)
        
        # Setup update timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_visualization)
        interval_ms = int(1000.0 / max(1.0, self.node.plot_hz))
        self.timer.start(interval_ms)
        
        # Camera follow mode
        self.follow_camera = True
        self.camera_offset = np.array([2, 2, 1.5])
        
        self.resize(1280, 720)
    
    def update_visualization(self):
        latest, hist = self.node.get_snapshot()
        
        if latest is None or len(hist) < 2:
            return
        
        # Get drone position and orientation
        pos = np.array([latest.x, latest.y, latest.z])
        R = quat_to_rotation_matrix(latest.qx, latest.qy, latest.qz, latest.qw)
        roll, pitch, yaw = quat_to_euler(latest.qx, latest.qy, latest.qz, latest.qw)
        
        # Create transform matrix
        transform = np.eye(4)
        transform[:3, :3] = R
        transform[:3, 3] = pos
        
        # Update drone mesh
        self.drone_mesh.setTransform(transform)
        
        # Update trail (limit length for performance)
        trail_data = hist[-self.node.trail_length:]
        if len(trail_data) > 1:
            trail_pts = np.array([[s.x, s.y, s.z] for s in trail_data])
            self.trail.setData(pos=trail_pts)
        
        # Update position marker
        self.marker.setData(pos=pos.reshape(1, 3))
        
        # Update camera to follow drone
        if self.follow_camera:
            self.opts['center'] = pg.Vector(pos[0], pos[1], pos[2])
        
        # Update telemetry
        velocity = 0.0
        if len(hist) > 10:
            dt = hist[-1].t - hist[-10].t
            dx = hist[-1].x - hist[-10].x
            dy = hist[-1].y - hist[-10].y
            dz = hist[-1].z - hist[-10].z
            if dt > 0:
                velocity = np.sqrt(dx*dx + dy*dy + dz*dz) / dt
        
        telemetry_text = f"""
TELEMETRY
─────────────────
Position:
  X: {pos[0]:7.3f} m
  Y: {pos[1]:7.3f} m
  Z: {pos[2]:7.3f} m

Attitude:
  Roll:  {np.degrees(roll):6.1f}°
  Pitch: {np.degrees(pitch):6.1f}°
  Yaw:   {np.degrees(yaw):6.1f}°

Velocity: {velocity:.2f} m/s
Time:     {latest.t:.1f} s
        """.strip()
        
        self.telemetry_label.setText(telemetry_text)
        self.telemetry_label.adjustSize()
    
    def keyPressEvent(self, event):
        """Handle keyboard input."""
        if event.key() == QtCore.Qt.Key.Key_F:
            self.follow_camera = not self.follow_camera
        elif event.key() == QtCore.Qt.Key.Key_R:
            self.opts['elevation'] = 30
            self.opts['azimuth'] = 45
            self.opts['distance'] = 3
        super().keyPressEvent(event)


def main():
    rclpy.init()

    node = FlightVisNode()

    # Spin ROS in background
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication([])
    
    window = Simulator3DWindow(node)
    window.show()
    
    node.get_logger().info("Controls: F=Toggle camera follow, R=Reset view, Mouse=Rotate/Pan/Zoom")
    
    try:
        app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()