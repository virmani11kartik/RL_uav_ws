"""Math utils"""

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def lowpass(prev, x, alpha):
    return prev + alpha * (x - prev)

def alpha_from_cutoff(f_cut, dt):
    """calc alpha from cutoff freq"""
    import math
    if f_cut <= 0:
        return 1.0
    rc = 1.0 / (2.0 * math.pi * f_cut)
    return dt / (dt + rc)