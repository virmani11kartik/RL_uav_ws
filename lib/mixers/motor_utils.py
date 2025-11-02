"""Motor utilities"""

from ..utils.math_utils import clamp

def desaturate_and_clip(motors, idle=0.05):
    """desaturate and clip motors to [idle..1.0]"""
    if not motors:
        return []
    
    lo, hi = min(motors), max(motors)
    out = motors[:]
    
    # all equal, just clamp
    if hi - lo < 1e-6:
        return [clamp(x, idle, 1.0) for x in out]
    
    # shift to above idle
    if lo < idle:
        shift = idle - lo
        out = [x + shift for x in out]
        hi = hi + shift
    
    # scale if above 1.0
    if hi > 1.0:
        span = hi - idle
        if span > 1e-6:
            scale = (1.0 - idle) / span
            out = [idle + (x - idle) * scale for x in out]
    
    # final clamp
    return [clamp(x, idle, 1.0) for x in out]