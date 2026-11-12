"""
Round driver, started at boot by gorur-round.service on top of
gorur_base.launch.py (mcu_bridge, vector_odom, LiDAR).

    round:=open      local_comp open_round      config: local_comp/config/open_round.yaml
    round:=obstacle  local_comp obstacle_round  config: local_comp/config/obstacle_round.yaml

Both nodes boot into STANDBY and only drive after the start button on the
car (the ESP32 BOOT button, /button_status) is pressed. Only one round runs
at a time - both own /cmd_vel.

    ros2 launch launch/gorur_round.launch.py round:=obstacle
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ROUNDS = {
    'open': ('open_round', 'open_round.yaml'),
    'obstacle': ('obstacle_round', 'obstacle_round.yaml'),
}


def _driver(context):
    round_name = LaunchConfiguration('round').perform(context).strip().lower()
    if round_name not in ROUNDS:
        raise RuntimeError(
            f"round must be one of {sorted(ROUNDS)}, got '{round_name}'")
    executable, config = ROUNDS[round_name]
    config_path = LaunchConfiguration('params_file').perform(context) or (
        FindPackageShare('local_comp').perform(context) + '/config/' + config)
    return [Node(
        package='local_comp',
        executable=executable,
        name=executable,  # matches the top-level key in its yaml
        output='screen',
        emulate_tty=True,
        parameters=[config_path],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'round', default_value='open',
            description='Which round to drive: open or obstacle.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Override the round yaml (default: the local_comp config).'),
        OpaqueFunction(function=_driver),
    ])
