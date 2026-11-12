"""
Always-on hardware layer of the car, started at boot by gorur-base.service.

    mcu_bridge      controls     USB link to the ESP32-S3 - publishes
                                 encoder/count, encoder/speed, heading,
                                 button_status, ...; drives /cmd_vel out
    vector_odom     controls     encoder/count + heading -> /odom_vector
                                 (and the odom -> base_link TF)
    sllidar_node    sllidar_ros2 RPLIDAR C1 on /dev/rplidar -> /scan
                                 (Slamtec's driver, built in ~/lidar_ws by
                                 systemd/install.sh - the apt rplidar_ros has
                                 no C1 support)

The round drivers (local_comp open_round / obstacle_round) are deliberately
NOT here: they run in gorur-round.service, so switching rounds or restarting
a driver never drops the MCU or LiDAR link. See ros2_ws/systemd/README.md.

Run it by hand from the workspace root:

    ros2 launch launch/gorur_base.launch.py
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# This file sits in <workspace>/launch, the tuning yamls in <workspace>/config.
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BOT_CONFIG = os.path.join(WORKSPACE_DIR, 'config', 'bot_config.yaml')

# Same odometry calibration as the gorurgari_* round launch files.
DEFAULT_WHEEL_DIAMETER_M = '0.057'
DEFAULT_COUNTS_PER_REV = '400'
DEFAULT_DISTANCE_SCALE = '1.0'  # re-tape-measure on the 57 mm wheel


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'bot_config', default_value=DEFAULT_BOT_CONFIG,
            description='bot_config.yaml with the mcu_bridge section.'),
        DeclareLaunchArgument(
            'wheel_diameter_m', default_value=DEFAULT_WHEEL_DIAMETER_M,
            description='Drive wheel diameter in metres.'),
        DeclareLaunchArgument(
            'encoder_counts_per_rev', default_value=DEFAULT_COUNTS_PER_REV,
            description='Encoder ticks per wheel revolution.'),
        DeclareLaunchArgument(
            'distance_scale', default_value=DEFAULT_DISTANCE_SCALE,
            description='Odometry calibration: tape distance / reported distance.'),
        DeclareLaunchArgument(
            'output_units', default_value='cm',
            description='Units of /odom_vector, cm or m.'),
        DeclareLaunchArgument(
            'heading_clockwise', default_value='false',
            description='True if /heading grows clockwise (ROS grows counter clockwise).'),
        DeclareLaunchArgument(
            'lidar_serial_port', default_value='/dev/rplidar',
            description='RPLIDAR C1 serial port (/dev/rplidar from config/99-gorur.rules).'),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value='laser',
            description='Frame ID of /scan.'),
        DeclareLaunchArgument(
            'lidar_inverted', default_value='true',
            description='True when the C1 is mounted upside down (it is on this car). '
                        'Check: a box 1 m to the LEFT must read at a POSITIVE angle.'),
    ]

    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    mcu_bridge = Node(
        package='controls',
        executable='mcu_bridge',
        name='mcu_bridge',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        parameters=[LaunchConfiguration('bot_config')],
    )

    vector_odom = Node(
        package='controls',
        executable='vector_odom',
        name='vector_odometry',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'wheel_diameter_m': typed('wheel_diameter_m', float),
            'encoder_counts_per_rev': typed('encoder_counts_per_rev', int),
            'distance_scale': typed('distance_scale', float),
            'output_units': typed('output_units', str),
            'heading_clockwise': typed('heading_clockwise', bool),
        }],
    )

    # sllidar_node exits if the port is missing or the C1 does not answer;
    # respawn keeps retrying so the LiDAR can be plugged in after boot.
    rplidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': 460800,  # RPLIDAR C1
            'frame_id': LaunchConfiguration('lidar_frame_id'),
            'inverted': typed('lidar_inverted', bool),
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    # Without the driver the car cannot run a round, but keep the MCU link
    # up anyway (status LEDs, OLED, debugging) instead of failing the launch.
    try:
        get_package_share_directory('sllidar_ros2')
    except PackageNotFoundError:
        rplidar = LogInfo(msg='[ERROR] sllidar_ros2 not found - no /scan. '
                              'Build it with ros2_ws/systemd/install.sh.')

    return LaunchDescription(arguments + [mcu_bridge, vector_odom, rplidar])
