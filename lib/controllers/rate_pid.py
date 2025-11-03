"""Single axis PID"""

from ..utils.math_utils import clamp, lowpass, alpha_from_cutoff

class RatePID:
    
    def __init__(self, kp=0.08, ki=0.05, kd=0.0015, d_cut_hz=90.0, dt_init=0.002):
        self.kp = kp
        self.ki = ki 
        self.kd = kd
        
        # state
        self.integrator = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        
        # filter
        self.d_alpha = alpha_from_cutoff(d_cut_hz, dt_init)
        
        # output limits
        self.out_min = -1.0
        self.out_max = 1.0
    
    def set_gains(self, kp=None, ki=None, kd=None):
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd
    
    def set_dt(self, dt, d_cut_hz=None):
        if d_cut_hz is not None:
            self.d_alpha = alpha_from_cutoff(d_cut_hz, dt)
    
    def reset(self):
        self.integrator = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
    
    def update(self, error, dt):
        # P term
        p_term = self.kp * error
        
        # I term
        i_term = self.integrator
        
        # D term with filter
        if dt > 0:
            d_raw = (error - self.prev_err) / dt
        else:
            d_raw = 0.0
            
        self.d_filt = lowpass(self.d_filt, d_raw, self.d_alpha)
        d_term = self.kd * self.d_filt
        
        # sum
        output = p_term + i_term + d_term
        
        # saturation & anti-windup
        if output > self.out_max:
            output = self.out_max
            if error > 0:
                self.integrator -= self.ki * error * dt
        elif output < self.out_min:
            output = self.out_min
            if error < 0:
                self.integrator -= self.ki * error * dt
        else:
            self.integrator += self.ki * error * dt
        
        self.prev_err = error
        return output