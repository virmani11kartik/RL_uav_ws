"""Base controller interface"""

from abc import ABC, abstractmethod
from typing import Tuple
from ..utils.data_structures import AxisRates


class BaseController(ABC):
    """Abstract base class for flight controllers"""
    
    @abstractmethod
    def update(self, target: AxisRates, current: AxisRates, dt: float) -> Tuple[float, float, float]:
        """
        Update controller with target and current values
        
        Args:
            target: Target rates/angles
            current: Current measured rates/angles  
            dt: Time step in seconds
            
        Returns:
            Tuple of (roll_cmd, pitch_cmd, yaw_cmd) control outputs
        """
        pass
    
    @abstractmethod
    def reset(self):
        """Reset controller state (integrators, filters, etc.)"""
        pass
    
    @abstractmethod
    def set_gains(self, **kwargs):
        """Set controller gains"""
        pass
