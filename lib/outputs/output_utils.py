"""Utility functions for ESC output validation and processing"""

from typing import List, Union, Dict, Any


def validate_motor_outputs(outputs: List[Union[int, float]], 
                          min_val: Union[int, float], 
                          max_val: Union[int, float]) -> bool:
    """
    Validate that all motor outputs are within specified range
    
    Args:
        outputs: List of motor output values
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        True if all outputs are valid
    """
    return all(min_val <= output <= max_val for output in outputs)


def clamp_motor_outputs(outputs: List[Union[int, float]], 
                       min_val: Union[int, float], 
                       max_val: Union[int, float]) -> List[Union[int, float]]:
    """
    Clamp motor outputs to specified range
    
    Args:
        outputs: List of motor output values
        min_val: Minimum allowed value  
        max_val: Maximum allowed value
        
    Returns:
        List of clamped motor outputs
    """
    return [max(min_val, min(max_val, output)) for output in outputs]


def detect_motor_failures(outputs: List[Union[int, float]], 
                         expected_outputs: List[Union[int, float]], 
                         tolerance: float = 0.1) -> List[int]:
    """
    Detect potential motor failures by comparing expected vs actual outputs
    
    Args:
        outputs: Actual motor outputs
        expected_outputs: Expected motor outputs
        tolerance: Tolerance for difference (as fraction)
        
    Returns:
        List of motor indices that may have failed
    """
    if len(outputs) != len(expected_outputs):
        raise ValueError("Output lists must have same length")
    
    failed_motors = []
    for i, (actual, expected) in enumerate(zip(outputs, expected_outputs)):
        if expected == 0:
            continue  # Skip if expected output is zero
            
        relative_error = abs(actual - expected) / abs(expected)
        if relative_error > tolerance:
            failed_motors.append(i)
    
    return failed_motors


def calculate_motor_statistics(outputs: List[Union[int, float]]) -> Dict[str, float]:
    """
    Calculate statistics for motor outputs
    
    Args:
        outputs: List of motor output values
        
    Returns:
        Dictionary with min, max, mean, std statistics
    """
    if not outputs:
        return {'min': 0, 'max': 0, 'mean': 0, 'std': 0}
    
    min_val = min(outputs)
    max_val = max(outputs)
    mean_val = sum(outputs) / len(outputs)
    
    # Calculate standard deviation
    variance = sum((x - mean_val) ** 2 for x in outputs) / len(outputs)
    std_val = variance ** 0.5
    
    return {
        'min': min_val,
        'max': max_val, 
        'mean': mean_val,
        'std': std_val,
        'range': max_val - min_val
    }


def format_motor_outputs(outputs: List[Union[int, float]], 
                        protocol: str = "PWM", 
                        precision: int = 2) -> str:
    """
    Format motor outputs for display/logging
    
    Args:
        outputs: List of motor output values
        protocol: Protocol name for display
        precision: Decimal precision for float values
        
    Returns:
        Formatted string representation
    """
    if not outputs:
        return f"{protocol}: []"
    
    if isinstance(outputs[0], int):
        formatted = [str(x) for x in outputs]
    else:
        formatted = [f"{x:.{precision}f}" for x in outputs]
    
    return f"{protocol}: [{', '.join(formatted)}]"


def create_output_log_entry(outputs: List[Union[int, float]], 
                           protocol_info: Dict[str, Any], 
                           timestamp: float = None) -> Dict[str, Any]:
    """
    Create structured log entry for motor outputs
    
    Args:
        outputs: Motor output values
        protocol_info: Protocol information dictionary
        timestamp: Optional timestamp
        
    Returns:
        Structured log entry
    """
    import time
    
    if timestamp is None:
        timestamp = time.time()
    
    stats = calculate_motor_statistics(outputs)
    
    return {
        'timestamp': timestamp,
        'protocol': protocol_info.get('protocol', 'Unknown'),
        'motor_count': len(outputs),
        'outputs': outputs,
        'statistics': stats,
        'protocol_info': protocol_info,
        'armed': protocol_info.get('armed', False)
    }


def check_output_symmetry(outputs: List[Union[int, float]], 
                         tolerance: float = 0.05) -> Dict[str, Any]:
    """
    Check motor output symmetry for quad configurations
    
    Args:
        outputs: Motor outputs (assumes 4-motor quad)
        tolerance: Tolerance for symmetry check
        
    Returns:
        Dictionary with symmetry analysis
    """
    if len(outputs) != 4:
        return {'symmetric': False, 'reason': 'Not a 4-motor configuration'}
    
    m1, m2, m3, m4 = outputs
    
    # Check diagonal symmetry (for X-config)
    diag1_diff = abs(m1 - m3)  # Front-left vs rear-right
    diag2_diff = abs(m2 - m4)  # Front-right vs rear-left
    
    # Check side symmetry
    front_diff = abs(m1 - m2)  # Front motors
    rear_diff = abs(m3 - m4)   # Rear motors
    left_diff = abs(m1 - m4)   # Left motors  
    right_diff = abs(m2 - m3)  # Right motors
    
    max_output = max(outputs)
    if max_output == 0:
        return {'symmetric': True, 'reason': 'All outputs zero'}
    
    # Normalize differences by max output
    diag1_rel = diag1_diff / max_output
    diag2_rel = diag2_diff / max_output
    front_rel = front_diff / max_output
    rear_rel = rear_diff / max_output
    left_rel = left_diff / max_output
    right_rel = right_diff / max_output
    
    symmetric = all(diff <= tolerance for diff in [diag1_rel, diag2_rel, front_rel, rear_rel, left_rel, right_rel])
    
    return {
        'symmetric': symmetric,
        'diagonal_symmetry': {'m1_m3': diag1_rel, 'm2_m4': diag2_rel},
        'side_symmetry': {'front': front_rel, 'rear': rear_rel, 'left': left_rel, 'right': right_rel},
        'tolerance': tolerance
    }
