"""
Open round, in simulation — three laps of the mat, then park on the
start point.

The same node graph as ros2_ws/launch/gorurgari_open_round.launch.py,
with the two nodes that need hardware swapped out:

    mcu_bridge   -> lazybridge   (no ESP32, no serial port)
    rplidar_c1   -> ros_gz_bridge (no LiDAR, /scan comes from gpu_lidar)

vector_odom, lap_counter and open_round_run are the real nodes, running
their real code, against a /scan and a /heading that look like the
hardware's. That is what makes a result here mean something.

    ros2 launch lazysim open_round_sim.launch.py

Watch it think without letting it move:

    ros2 launch lazysim open_round_sim.launch.py enable_auto_steering:=false

Rehearse the real start sequence (standby until the button):

    ros2 launch lazysim open_round_sim.launch.py require_button_start:=true
    ros2 service call /lazybot/press_start std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'lazysim'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    default_params = os.path.join(share, 'config', 'bot_config_sim.yaml')

    arguments = [
        DeclareLaunchArgument('bot_config', default_value=default_params,
                              description='Tuning for the whole stack.'),
        DeclareLaunchArgument('track_config',
                              default_value=os.path.join(share, 'config', 'track.yaml'),
                              description='Pillar / parking / start layout.'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('enable_auto_steering', default_value='true',
                              description='false to watch it plan without moving.'),
        DeclareLaunchArgument('require_button_start', default_value='false',
                              description='true holds the car in standby until '
                                          '/lazybot/press_start is called.'),
    ]

    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'bot_config': LaunchConfiguration('bot_config'),
            'track_config': LaunchConfiguration('track_config'),
        }.items(),
    )

    # encoder/count + heading -> /odom_vector and the odom -> base_link TF.
    vector_odom = Node(
        package='controls',
        executable='vector_odom',
        name='vector_odometry',
        parameters=[LaunchConfiguration('bot_config'), {'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
    )

    lap_counter = Node(
        package='autonomy',
        executable='lap_counter',
        name='lap_counter',
        parameters=[LaunchConfiguration('bot_config'), {'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
    )

    open_round_run = Node(
        package='autonomy',
        executable='open_round_run',
        name='open_round_run',
        parameters=[
            LaunchConfiguration('bot_config'),
            {
                'use_sim_time': True,
                'enable_auto_steering': typed('enable_auto_steering', bool),
                'require_button_start': typed('require_button_start', bool),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(arguments + [
        simulator,
        vector_odom,
        lap_counter,
        open_round_run,
    ])
