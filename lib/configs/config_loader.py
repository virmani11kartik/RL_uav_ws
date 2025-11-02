"""Configuration loading and saving utilities"""

import json
import os
from pathlib import Path
from typing import Dict, Any, Optional
from .quad_config import QuadConfig
from .presets import PRESET_CONFIGS, get_preset_config


def load_config(file_path: str) -> QuadConfig:
    """
    Load configuration from JSON file
    
    Args:
        file_path: Path to configuration file
        
    Returns:
        QuadConfig instance
        
    Raises:
        FileNotFoundError: If file doesn't exist
        json.JSONDecodeError: If file contains invalid JSON
        ValueError: If configuration is invalid
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Configuration file not found: {file_path}")
    
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    config = QuadConfig.from_dict(data)
    
    # Validate configuration
    issues = config.validate()
    if issues:
        raise ValueError(f"Invalid configuration: {'; '.join(issues)}")
    
    return config


def save_config(config: QuadConfig, file_path: str, create_dirs: bool = True):
    """
    Save configuration to JSON file
    
    Args:
        config: QuadConfig instance to save
        file_path: Path to save configuration
        create_dirs: Create parent directories if they don't exist
        
    Raises:
        ValueError: If configuration is invalid
        OSError: If file cannot be written
    """
    # Validate configuration before saving
    issues = config.validate()
    if issues:
        raise ValueError(f"Cannot save invalid configuration: {'; '.join(issues)}")
    
    # Create parent directories if needed
    if create_dirs:
        Path(file_path).parent.mkdir(parents=True, exist_ok=True)
    
    # Save configuration
    with open(file_path, 'w') as f:
        f.write(config.to_json(indent=2))


def load_config_or_preset(identifier: str) -> QuadConfig:
    """
    Load configuration from file or preset name
    
    Args:
        identifier: File path or preset name
        
    Returns:
        QuadConfig instance
    """
    # Check if it's a file path
    if os.path.exists(identifier) or '/' in identifier or '\\' in identifier:
        return load_config(identifier)
    
    # Try as preset name
    try:
        return get_preset_config(identifier)
    except KeyError:
        raise ValueError(f"'{identifier}' is not a valid file path or preset name")


def get_preset_configs() -> Dict[str, QuadConfig]:
    """Get all preset configurations"""
    return PRESET_CONFIGS.copy()


def list_config_files(directory: str) -> list:
    """
    List all JSON configuration files in a directory
    
    Args:
        directory: Directory to search
        
    Returns:
        List of configuration file paths
    """
    if not os.path.exists(directory):
        return []
    
    config_files = []
    for file in os.listdir(directory):
        if file.endswith('.json'):
            file_path = os.path.join(directory, file)
            try:
                # Try to load to verify it's a valid config
                load_config(file_path)
                config_files.append(file_path)
            except (json.JSONDecodeError, ValueError, KeyError):
                # Skip invalid configuration files
                continue
    
    return sorted(config_files)


def create_config_backup(config: QuadConfig, backup_dir: str = "config_backups") -> str:
    """
    Create a timestamped backup of configuration
    
    Args:
        config: Configuration to backup
        backup_dir: Directory to store backups
        
    Returns:
        Path to backup file
    """
    import datetime
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_filename = f"{config.name.replace(' ', '_')}_{timestamp}.json"
    backup_path = os.path.join(backup_dir, backup_filename)
    
    save_config(config, backup_path)
    return backup_path


def merge_configs(base_config: QuadConfig, override_config: QuadConfig) -> QuadConfig:
    """
    Merge two configurations, with override taking precedence
    
    Args:
        base_config: Base configuration
        override_config: Configuration with overrides
        
    Returns:
        Merged QuadConfig instance
    """
    base_dict = base_config.to_dict()
    override_dict = override_config.to_dict()
    
    # Deep merge dictionaries
    def deep_merge(base: dict, override: dict) -> dict:
        result = base.copy()
        for key, value in override.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = deep_merge(result[key], value)
            else:
                result[key] = value
        return result
    
    merged_dict = deep_merge(base_dict, override_dict)
    return QuadConfig.from_dict(merged_dict)


def export_config_summary(config: QuadConfig) -> str:
    """
    Export configuration as human-readable summary
    
    Args:
        config: Configuration to summarize
        
    Returns:
        Formatted summary string
    """
    summary = f"""
Configuration: {config.name}
Description: {config.description}

=== Rate Control ===
Roll  PID: P={config.rates.roll.kp:.3f}, I={config.rates.roll.ki:.3f}, D={config.rates.roll.kd:.4f}
Pitch PID: P={config.rates.pitch.kp:.3f}, I={config.rates.pitch.ki:.3f}, D={config.rates.pitch.kd:.4f}
Yaw   PID: P={config.rates.yaw.kp:.3f}, I={config.rates.yaw.ki:.3f}, D={config.rates.yaw.kd:.4f}
D-term Filter: {config.rates.d_cut_hz:.1f} Hz

=== Rate Mapping ===
Max Rates: Roll={config.rate_mapping.max_rates_dps[0]:.0f}°/s, Pitch={config.rate_mapping.max_rates_dps[1]:.0f}°/s, Yaw={config.rate_mapping.max_rates_dps[2]:.0f}°/s
Expo: Roll={config.rate_mapping.expo[0]:.2f}, Pitch={config.rate_mapping.expo[1]:.2f}, Yaw={config.rate_mapping.expo[2]:.2f}
Super-Rate: Roll={config.rate_mapping.super_rates[0]:.2f}, Pitch={config.rate_mapping.super_rates[1]:.2f}, Yaw={config.rate_mapping.super_rates[2]:.2f}

=== Hardware ===
Mixer: {config.mixer.type.upper()} (idle: {config.mixer.motor_idle:.2f})
ESC Protocol: {config.esc.protocol.upper()} ({config.esc.motor_count} motors)
Loop Frequency: {config.loop_frequency:.0f} Hz
Battery: {config.battery_cells}S, Motor Poles: {config.motor_poles}

=== Validation ===
Status: {"✓ Valid" if config.is_valid() else "✗ Invalid"}
"""
    
    issues = config.validate()
    if issues:
        summary += f"Issues: {'; '.join(issues)}\n"
    
    return summary.strip()
