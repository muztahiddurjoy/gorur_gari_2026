"""
Round driver the start button starts, run at boot by gorur-round.service.

It sits on top of gorur_base.launch.py (mcu_bridge, vector_odom, LiDAR).
Which driver runs, and how long after the start button it moves, is set in
config/round_select.yaml:

    round: open      local_comp open_round      tuning: local_comp/config/open_round.yaml
    round: obstacle  local_comp obstacle_round  tuning: local_comp/config/obstacle_round.yaml

The driver boots into STANDBY and drives exactly start_delay_sec after the
start button on the car (the ESP32 BOOT button, /button_status) is pressed,
while the status LED blinks that delay out. Only one round runs at a time -
both own /cmd_vel.

    ros2 launch launch/gorur_round.launch.py                  # as round_select.yaml says
    ros2 launch launch/gorur_round.launch.py round:=obstacle  # override it for one launch
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# This file sits in <workspace>/launch, the yamls in <workspace>/config.
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ROUND_SELECT = os.path.join(WORKSPACE_DIR, 'config', 'round_select.yaml')

ROUNDS = {
    'open': ('open_round', 'open_round.yaml'),
    'obstacle': ('obstacle_round', 'obstacle_round.yaml'),
}


def load_round_select(path):
    """Return (round, start_delay_sec) from round_select.yaml, or raise if it is wrong."""
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f'{path}: expected "round:" and "start_delay_sec:" keys')
    round_name = str(data.get('round', '')).strip().lower()
    if round_name not in ROUNDS:
        raise RuntimeError(
            f"{path}: round must be one of {sorted(ROUNDS)}, got '{data.get('round')}'")
    delay = data.get('start_delay_sec')
    if isinstance(delay, bool) or not isinstance(delay, (int, float)) or not delay > 0:
        raise RuntimeError(f'{path}: start_delay_sec must be a number > 0, got {delay!r}')
    return round_name, float(delay)


def _driver(context):
    select_path = LaunchConfiguration('round_select').perform(context)
    round_name, start_delay_sec = load_round_select(select_path)

    override = LaunchConfiguration('round').perform(context).strip().lower()
    if override:
        if override not in ROUNDS:
            raise RuntimeError(
                f"round must be one of {sorted(ROUNDS)}, got '{override}'")
        round_name = override

    executable, config = ROUNDS[round_name]
    config_path = LaunchConfiguration('params_file').perform(context) or (
        FindPackageShare('local_comp').perform(context) + '/config/' + config)
    return [
        LogInfo(msg=f'Start button -> local_comp {executable}, '
                    f'drives {start_delay_sec:g} s after the press '
                    f'({"round:= override" if override else select_path}).'),
        Node(
            package='local_comp',
            executable=executable,
            name=executable,  # matches the top-level key in its yaml
            output='screen',
            emulate_tty=True,
            # the delay comes last so round_select.yaml wins over the tuning yaml
            parameters=[config_path, {'start_delay_sec': start_delay_sec}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'round_select', default_value=DEFAULT_ROUND_SELECT,
            description='Which round the start button starts, and its start delay.'),
        DeclareLaunchArgument(
            'round', default_value='',
            description='open or obstacle: overrides round_select.yaml for this launch.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Override the round tuning yaml (default: the local_comp config).'),
        OpaqueFunction(function=_driver),
    ])
