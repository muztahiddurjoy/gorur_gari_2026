"""
Obstacle round, in simulation: laps with red and green traffic pillars, passing red
on the right and green on the left.

The same nodes, with the same parameter files, as
ros2_ws/launch/gorurgari_obstacle_round.launch.py, with the hardware swapped:

    ESP32-S3     -> sim_mcu        (real mcu_bridge, over a virtual serial port)
    rplidar_c1   -> ros_gz_bridge  (/scan from gpu_lidar)
    usb_cam      -> ros_gz_bridge  (/camera/image_raw from the C920 model)

    ros2 launch gorusim obstacle_round_sim.launch.py

Check what the camera sees:

    ros2 run rqt_image_view rqt_image_view /camera/image_raw
    ros2 topic echo /closest_obj

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
    overlay = os.path.join(config, 'sim_overlay.yaml')

    arguments = [
        DeclareLaunchArgument('track_config',
                              default_value=os.path.join(config, 'track.yaml'),
                              description='Pillar / parking / start layout.'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('enable_auto_steering', default_value='true'),
        DeclareLaunchArgument('require_button_start', default_value='false'),
        DeclareLaunchArgument('start_delay_sec', default_value='5.0'),
        # gorurgari_obstacle_round.launch.py's defaults, verbatim.
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

    vision_node = Node(
        package='autonomy',
        executable='vision_node',
        name='vision_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(config, 'vision_params.yaml'), {'use_sim_time': True}],
        # ROS Humble's cv_bridge is built against NumPy 1.x. A newer NumPy in
        # ~/.local makes this node segfault, so it runs on the system's packages.
        additional_env={'PYTHONNOUSERSITE': '1'},
    )

    disparity_extender = Node(
        package='autonomy',
        executable='disparity_extender',
        name='disparity_extender_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(config, 'disparity_extender_params.yaml'),
            overlay,
            {
                'enable_auto_steering': typed('enable_auto_steering', bool),
                'require_button_start': typed('require_button_start', bool),
                'start_delay_sec': typed('start_delay_sec', float),
                'use_sim_time': True,
            },
        ],
    )

    return LaunchDescription(arguments + [
        simulator,
        vector_odom,
        lap_counter,
        vision_node,
        disparity_extender,
    ])
