"""Configs"""

from .quad_config import QuadConfig
from .presets import PRESET_CONFIGS, get_preset_config
try:
    from .config_loader import load_config, save_config
    __all__ = ['QuadConfig', 'PRESET_CONFIGS', 'get_preset_config', 'load_config', 'save_config']
except ImportError:
    __all__ = ['QuadConfig', 'PRESET_CONFIGS', 'get_preset_config']