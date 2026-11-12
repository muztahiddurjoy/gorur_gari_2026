"""
Open round, in simulation: three laps of the bare mat, then park on the start point.

The same nodes, with the same parameters, as ros2_ws/launch/gorurgari_autonomy_boot.launch.py
plus the hardware services it leaves to systemd. Only what physically needs the car is
swapped, and mcu_bridge is not on that list:

    ESP32-S3     -> sim_mcu        (real mcu_bridge, over a virtual serial port)
    rplidar_c1   -> ros_gz_bridge  (/scan from gpu_lidar)

    ros2 launch gorusim open_round_sim.launch.py

Watch it think without letting it move:

    ros2 launch gorusim open_round_sim.launch.py enable_auto_steering:=false

Rehearse the real start sequence (standby until the button):

    ros2 launch gorusim open_round_sim.launch.py require_button_start:=true
    ros2 service call /gorusim/press_start std_srvs/srv/Trigger

Track tooling adapted from Team LazyGo's simulator:
https://github.com/A-N-M-Noor/LazyGo_WRO2025/
For testing purposes only: none of this runs on the competition car.
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
    bot_config = os.path.join(config, 'bot_config.yaml')
    overlay = os.path.join(config, 'sim_overlay.yaml')

    arguments = [
        DeclareLaunchArgument('track_config',
                              default_value=os.path.join(config, 'track_open.yaml'),
                              description='Bare mat by default: this is the open round.'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('enable_auto_steering', default_value='true',
                              description='false to watch it plan without moving.'),
        DeclareLaunchArgument('require_button_start', default_value='false',
                              description='true holds the car in standby until '
                                          '/gorusim/press_start is called.'),
        DeclareLaunchArgument('start_delay_sec', default_value='3.0'),
        # gorurgari_autonomy_boot.launch.py's defaults, verbatim.
        DeclareLaunchArgument('wheel_diameter_m', default_value='0.057'),
        DeclareLaunchArgument('encoder_counts_per_rev', default_value='400'),
        DeclareLaunchArgument('distance_scale', default_value='1.0'),
        DeclareLaunchArgument('output_units', default_value='cm'),
        DeclareLaunchArgument('heading_clockwise', default_value='true'),
        DeclareLaunchArgument('turn_angle_deg', default_value='90.0'),
        DeclareLaunchArgument('turns_per_lap', default_value='4'),
    ]

    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={
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

    lap_counter = Node(
        package='autonomy',
        executable='lap_counter',
        name='lap_counter',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'turn_angle_deg': typed('turn_angle_deg', float),
            'turns_per_lap': typed('turns_per_lap', int),
            'heading_clockwise': typed('heading_clockwise', bool),
            'odom_distance_units': typed('output_units', str),
            'use_sim_time': True,
        }],
    )

    open_round_run = Node(
        package='autonomy',
        executable='open_round_run',
        name='open_round_run',
        output='screen',
        emulate_tty=True,
        parameters=[
            bot_config,
            overlay,
            {
                'enable_auto_steering': typed('enable_auto_steering', bool),
                'require_button_start': typed('require_button_start', bool),
                'start_delay_sec': typed('start_delay_sec', float),
                'use_sim_time': True,
            },
        ],
    )

    run_timer = Node(
        package='autonomy',
        executable='run_timer',
        name='run_timer',
        output='screen',
        emulate_tty=True,
        parameters=[bot_config, {'use_sim_time': True}],
    )

    return LaunchDescription(arguments + [
        simulator,
        vector_odom,
        lap_counter,
        open_round_run,
        run_timer,
    ])
