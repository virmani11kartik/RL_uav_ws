"""
lib - flight control components
Modular quadcopter control
"""

from .controllers import RatePID, RateController3D
from .mixers import QuadXMixer, BaseMixer, desaturate_and_clip
from .outputs import ESCProtocol, DshotOutput, PWMOutput
from .utils import AxisRates, clamp, lowpass, sticks_to_rates, RateMapper
from .configs import QuadConfig, load_config, get_preset_config

__version__ = "1.0.0"

__all__ = [
    'RatePID', 'RateController3D',
    'QuadXMixer', 'BaseMixer', 'desaturate_and_clip',
    'ESCProtocol', 'DshotOutput', 'PWMOutput',
    'AxisRates', 'clamp', 'lowpass', 'sticks_to_rates', 'RateMapper',
    'QuadConfig', 'load_config', 'get_preset_config'
]