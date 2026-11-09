"""
BUBT FastRobox / Autonomous Navigation Challenge 2026: Obstacle Avoidance Round Simulator.

Corridor: 3.0 m x 1.0 m straight path enclosed by 10 cm black walls.
Finish line: 30 cm before the end boundary wall (x = 2.70 m).
Obstacles: 6 plain 5x5x10 cm blocks.

Launches Ignition Gazebo with obstacle_bubt world, spawns the robot at the start line,
initializes the virtual ESP32 bridge (sim_mcu), runs vector odometry for RViz,
and builds the track layout from track_obstacle_bubt.yaml.

Usage:
    ros2 launch gorusim obstacle_bubt_sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
            'track',
            default_value='default',
            description='Track difficulty preset: default, level1_gentle, level2_slalom, level3_chicanes, level4_bottleneck, level5_nightmare.'
        ),
        DeclareLaunchArgument(
            'track_config',
            default_value='',
            description='Explicit obstacle YAML path. If empty, uses track=<preset>.'
        ),
        DeclareLaunchArgument('gui', default_value='true', description='Run Ignition GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('wheel_diameter_m', default_value='0.065'),
        DeclareLaunchArgument('encoder_counts_per_rev', default_value='363'),
        DeclareLaunchArgument('distance_scale', default_value='0.976'),
        DeclareLaunchArgument('output_units', default_value='cm'),
        DeclareLaunchArgument('heading_clockwise', default_value='true'),
    ]

    def resolve_track_config(context):
        explicit = LaunchConfiguration('track_config').perform(context).strip()
        if explicit:
            return explicit
        preset = LaunchConfiguration('track').perform(context).strip()
        filename_map = {
            'default': 'track_obstacle_bubt.yaml',
            'level1': 'track_obstacle_bubt_level1_gentle.yaml',
            'level1_gentle': 'track_obstacle_bubt_level1_gentle.yaml',
            'level2': 'track_obstacle_bubt_level2_slalom.yaml',
            'level2_slalom': 'track_obstacle_bubt_level2_slalom.yaml',
            'level3': 'track_obstacle_bubt_level3_chicanes.yaml',
            'level3_chicanes': 'track_obstacle_bubt_level3_chicanes.yaml',
            'level4': 'track_obstacle_bubt_level4_bottleneck.yaml',
            'level4_bottleneck': 'track_obstacle_bubt_level4_bottleneck.yaml',
            'level5': 'track_obstacle_bubt_level5_nightmare.yaml',
            'level5_nightmare': 'track_obstacle_bubt_level5_nightmare.yaml',
            'level6': 'track_obstacle_bubt_level6_funnel.yaml',
            'level6_funnel': 'track_obstacle_bubt_level6_funnel.yaml',
            'level7': 'track_obstacle_bubt_level7_blind_cascade.yaml',
            'level7_blind_cascade': 'track_obstacle_bubt_level7_blind_cascade.yaml',
            'level8': 'track_obstacle_bubt_level8_wall_squeeze.yaml',
            'level8_wall_squeeze': 'track_obstacle_bubt_level8_wall_squeeze.yaml',
            'level9': 'track_obstacle_bubt_level9_cluster.yaml',
            'level9_cluster': 'track_obstacle_bubt_level9_cluster.yaml',
            'level10': 'track_obstacle_bubt_level10_chaos.yaml',
            'level10_chaos': 'track_obstacle_bubt_level10_chaos.yaml',
        }
        filename = filename_map.get(preset, f'track_obstacle_bubt_{preset}.yaml')
        path = os.path.join(config, filename)
        if not os.path.isfile(path):
            path = os.path.join(config, 'track_obstacle_bubt.yaml')
        return path

    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    def launch_sim(context):
        resolved_config = resolve_track_config(context)
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
                launch_arguments={
                    'world': 'obstacle_bubt',
                    'gui': LaunchConfiguration('gui'),
                    'rviz': LaunchConfiguration('rviz'),
                    'track_config': resolved_config,
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
