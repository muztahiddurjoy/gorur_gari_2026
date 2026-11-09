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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
            'track_config',
            default_value=os.path.join(config, 'track_obstacle_bubt.yaml'),
            description='Obstacle and start layout YAML for BUBT obstacle round.'
        ),
        DeclareLaunchArgument('gui', default_value='true', description='Run Ignition GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('wheel_diameter_m', default_value='0.065'),
        DeclareLaunchArgument('encoder_counts_per_rev', default_value='363'),
        DeclareLaunchArgument('distance_scale', default_value='0.976'),
        DeclareLaunchArgument('output_units', default_value='cm'),
        DeclareLaunchArgument('heading_clockwise', default_value='true'),
    ]

    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': 'obstacle_bubt',
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'track_config': LaunchConfiguration('track_config'),
        }.items(),
    )

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
        simulator,
        vector_odom,
    ])
