"""Mixers"""

from .quad_mixer import QuadXMixer, QuadPlusMixer
from .motor_utils import desaturate_and_clip
try:
    from .base_mixer import BaseMixer
except ImportError:
    BaseMixer = None

__all__ = ['QuadXMixer', 'QuadPlusMixer', 'desaturate_and_clip']
if BaseMixer:
    __all__.append('BaseMixer')