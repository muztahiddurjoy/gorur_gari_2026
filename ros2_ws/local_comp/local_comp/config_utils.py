"""Shared configuration loader for local_comp nodes."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_default_params(node, yaml_filename: str, node_key: str) -> dict:
    """Load YAML defaults from local_comp's share/config so bare `ros2 run` uses them."""
    try:
        cfg_path = os.path.join(
            get_package_share_directory('local_comp'), 'config', yaml_filename)
        with open(cfg_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        params = data.get(node_key, {}).get('ros__parameters', {})
        node.get_logger().info(f'Loaded default config from {cfg_path}')
        return params if isinstance(params, dict) else {}
    except Exception as exc:
        node.get_logger().warn(f'Using code fallback defaults ({exc}).')
        return {}
