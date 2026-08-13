"""
Obstacle round, in simulation — laps with red and green traffic pillars,
passing red on the right and green on the left.

Mirrors ros2_ws/launch/gorurgari_obstacle_round.launch.py with the
hardware nodes swapped for the simulator:

    mcu_bridge   -> lazybridge
    rplidar_c1   -> ros_gz_bridge  (/scan from gpu_lidar)
    usb_cam      -> ros_gz_bridge  (/camera/image_raw from the sim camera)

The pillar colours in the world are chosen so vision_node's real HSV
thresholds work unchanged — see the note in config/object_template.sdf
about why the pillars have a black specular term.

    ros2 launch lazysim obstacle_round_sim.launch.py

Check what the camera actually sees:

    ros2 run rqt_image_view rqt_image_view /camera/image_raw
    ros2 topic echo /closest_obj

Adapted from Team LazyGo's simulator:
https://github.com/A-N-M-Noor/LazyGo_WRO2025/
For testing purposes only — none of this runs on the competition car.
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
        DeclareLaunchArgument('enable_auto_steering', default_value='true'),
        DeclareLaunchArgument('require_button_start', default_value='false'),
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

    # /camera/image_raw -> /closest_obj ("R", "G" or nothing in range)
    vision_node = Node(
        package='autonomy',
        executable='vision_node',
        name='vision_node',
        parameters=[LaunchConfiguration('bot_config'), {'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
    )

    # Fuses /scan with the pillar colour and drives /cmd_vel.
    disparity_extender = Node(
        package='autonomy',
        executable='disparity_extender',
        name='disparity_extender_node',
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
        vision_node,
        disparity_extender,
    ])
