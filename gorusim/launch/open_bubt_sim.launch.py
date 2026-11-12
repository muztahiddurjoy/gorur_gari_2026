"""
BUBT FastRobox / Autonomous Navigation Challenge 2026: Open Round Simulator.

Square circuit: 3.0 m x 3.0 m outer boundary, 1.0 m track width enclosed by 10 cm black walls.
3 full laps, stop on starting side.

Launches Ignition Gazebo with goruWorld, spawns the robot at the start line,
initializes the virtual ESP32 bridge (sim_mcu), runs vector odometry for RViz,
and lays out the open round track from track_open.yaml.

Usage:
    ros2 launch gorusim open_bubt_sim.launch.py
    ros2 launch gorusim open_bubt_sim.launch.py direction:=cw
    ros2 launch gorusim open_bubt_sim.launch.py direction:=random
"""
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'gorusim'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    config = os.path.join(share, 'config')

    arguments = [
        DeclareLaunchArgument(
            'direction',
            default_value='ccw',
            description="Circuit direction: 'ccw' (0 deg), 'cw' (180 deg), or 'random'."
        ),
        DeclareLaunchArgument('gui', default_value='true', description='Run Ignition GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('wheel_diameter_m', default_value='0.057'),
        DeclareLaunchArgument('encoder_counts_per_rev', default_value='400'),
        DeclareLaunchArgument('distance_scale', default_value='1.0'),
        DeclareLaunchArgument('output_units', default_value='cm'),
        DeclareLaunchArgument('heading_clockwise', default_value='true'),
    ]

    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    def launch_sim(context):
        resolved_config = os.path.join(config, 'track_open.yaml')
        direction = LaunchConfiguration('direction').perform(context).strip().lower()
        if direction == 'random':
            direction = random.choice(['ccw', 'cw'])
        if direction not in ('ccw', 'cw'):
            raise ValueError(f"Invalid direction '{direction}': expected 'ccw', 'cw', or 'random'.")
        start_angle_deg = '0.0' if direction == 'ccw' else '180.0'

        return [
            LogInfo(msg=f'[open_bubt_sim] Launching Open Round in {direction.upper()} direction (start_angle_deg={start_angle_deg}).'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
                launch_arguments={
                    'world': 'goruWorld',
                    'gui': LaunchConfiguration('gui'),
                    'rviz': LaunchConfiguration('rviz'),
                    'track_config': resolved_config,
                    'start_angle_deg': start_angle_deg,
                }.items(),
            )
        ]

    vector_odom = Node(
        package='controls',
        executable='vector_odom',
        name='vector_odometry',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'wheel_diameter_m': typed('wheel_diameter_m', float),
            'encoder_counts_per_rev': typed('encoder_counts_per_rev', int),
            'distance_scale': typed('distance_scale', float),
            'output_units': typed('output_units', str),
            'heading_clockwise': typed('heading_clockwise', bool),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=launch_sim),
        vector_odom,
    ])
