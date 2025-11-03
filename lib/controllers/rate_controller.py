"""3-axis rate controller"""

from .rate_pid import RatePID
from ..utils.data_structures import AxisRates

class RateController3D:
    
    def __init__(self, kp=(0.08, 0.08, 0.12), ki=(0.05, 0.05, 0.05), 
                 kd=(0.0015, 0.0015, 0.002), d_cut_hz=90.0, dt_init=0.002):
        self.pid_roll = RatePID(kp[0], ki[0], kd[0], d_cut_hz, dt_init)
        self.pid_pitch = RatePID(kp[1], ki[1], kd[1], d_cut_hz, dt_init)
        self.pid_yaw = RatePID(kp[2], ki[2], kd[2], d_cut_hz, dt_init)
    
    def set_gains(self, roll_gains=None, pitch_gains=None, yaw_gains=None, **kwargs):
        if roll_gains:
            self.pid_roll.set_gains(**roll_gains)
        if pitch_gains:
            self.pid_pitch.set_gains(**pitch_gains)
        if yaw_gains:
            self.pid_yaw.set_gains(**yaw_gains)
    
    def set_dt(self, dt, d_cut_hz=None):
        self.pid_roll.set_dt(dt, d_cut_hz)
        self.pid_pitch.set_dt(dt, d_cut_hz)
        self.pid_yaw.set_dt(dt, d_cut_hz)
    
    def reset(self):
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()
    
    def update(self, rate_target_dps, gyro_dps, dt):
        # calc errors
        e_roll = rate_target_dps.p - gyro_dps.p
        e_pitch = rate_target_dps.q - gyro_dps.q  
        e_yaw = rate_target_dps.r - gyro_dps.r
        
        # update PIDs
        u_roll = self.pid_roll.update(e_roll, dt)
        u_pitch = self.pid_pitch.update(e_pitch, dt)
        u_yaw = self.pid_yaw.update(e_yaw, dt)
        
        return u_roll, u_pitch, u_yaw