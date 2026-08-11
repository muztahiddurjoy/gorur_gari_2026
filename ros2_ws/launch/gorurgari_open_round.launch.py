"""
Open round: the full driving stack, no obstacles on the mat.

    mcu_bridge          controls    serial link to the car - publishes
                                    encoder/count, encoder/speed, heading,
                                    steering_angle, sonar; drives /cmd_vel out
    vector_odom         controls    encoder/count + heading -> /odom_vector
                                    (and the odom -> base_link TF)
    lap_counter         autonomy    /heading corners -> /lap_count, /turn_count
    rplidar_c1          sensors     serial link to LiDAR - drives /scan out
    open_round_run      autonomy    /scan + /odom_vector + /heading -> /cmd_vel,
                                    laps -> return to start position (homing).
                                    Starts in standby: it only drives 3 s after
                                    the start button on the car is pressed
                                    (/button_status), and the MCU blinks its
                                    status LED once a second over that delay

Everything talks on relative topic names in the root namespace, so the nodes
wire up to each other with no remaps. 

Run it from the workspace root:

    ros2 launch launch/gorurgari_open_round.launch.py

Tuning for open_round_run lives in config/bot_config.yaml
and is loaded by default; point the disparity_params argument somewhere else to
try an alternative set without editing the file.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# This file sits in <workspace>/launch, the tuning yamls in <workspace>/config.
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BOT_CONFIG = os.path.join(
    WORKSPACE_DIR, 'config', 'bot_config.yaml')

# Measure the wheel before a run - it scales every distance odometry reports.
DEFAULT_WHEEL_DIAMETER_M = '0.065'
# Ticks per wheel revolution as measured at the tick stream. Deliberately NOT
# the firmware's ENCODER_COUNTS_PER_REV (1320) - see the note on the parameter
# in controls/controls/vector_odom.py.
DEFAULT_COUNTS_PER_REV = '363'
# Residual error after the geometry is right: (tape measure / reported).
DEFAULT_DISTANCE_SCALE = '0.976'

SONARS = ['front', 'left', 'right', 'rear']


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'wheel_diameter_m', default_value=DEFAULT_WHEEL_DIAMETER_M,
            description='Drive wheel diameter in metres. Measure this, the default is a guess.'),
        DeclareLaunchArgument(
            'encoder_counts_per_rev', default_value=DEFAULT_COUNTS_PER_REV,
            description='Encoder ticks per wheel revolution, measured at the tick stream.'),
        DeclareLaunchArgument(
            'distance_scale', default_value=DEFAULT_DISTANCE_SCALE,
            description='Odometry calibration factor: tape measured distance / reported distance.'),
        DeclareLaunchArgument(
            'output_units', default_value='cm',
            description='Units of /odom_vector, cm or m. The TF is always metres.'),
        DeclareLaunchArgument(
            'heading_clockwise', default_value='true',
            description='BNO055 yaw grows clockwise, ROS grows counter clockwise. '
                        'Every /heading consumer here is given the same value.'),

        DeclareLaunchArgument(
            'turn_angle_deg', default_value='90.0',
            description='Degrees per track corner. 90 for a box mat.'),
        DeclareLaunchArgument(
            'turns_per_lap', default_value='4',
            description='Corners in one lap. 4 * 90 = one full loop of the mat.'),

        DeclareLaunchArgument(
            'disparity_params', default_value=DEFAULT_BOT_CONFIG,
            description='YAML of open_round_run and bot configuration (config/bot_config.yaml).'),
        DeclareLaunchArgument(
            'enable_auto_steering', default_value='true',
            description='Let open_round_run drive /cmd_vel. False to watch it '
                        'plan without the car moving.'),
        DeclareLaunchArgument(
            'require_button_start', default_value='true',
            description='Hold open_round_run in standby until the start button on '
                        'the car is pressed. False to drive as soon as scans arrive.'),
        DeclareLaunchArgument(
            'start_delay_sec', default_value='3.0',
            description='Seconds between the start button press and the first movement. '
                        'The MCU blinks its status LED once a second through it.'),
                        
        # LiDAR Arguments
        DeclareLaunchArgument(
            'lidar_serial_port', default_value='/dev/ttyUSB0',
            description='Serial port for the RPLidar C1.'),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value='laser',
            description='Frame ID for the LiDAR scans.'),
    ]

    # one launch argument per sonar, mirroring the firmware SONAR_*_ENABLED flags
    arguments += [
        DeclareLaunchArgument(
            f'sonar_{name}_enabled', default_value='false',
            description=f'Publish the {name} sonar. Only enable it if one is wired up.')
        for name in SONARS
    ]

    # launch arguments arrive as strings, so each one is coerced to the type the
    # node declared it with. Without this the nodes reject them on startup.
    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    mcu_bridge = Node(
        package='controls',
        executable='mcu_bridge',
        name='mcu_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            # serial port and /cmd_vel drive limits (throttle/steering
            # scaling and clamps) come from the mcu_bridge section here
            DEFAULT_BOT_CONFIG,
            {
                f'sonar_{name}_enabled': typed(f'sonar_{name}_enabled', bool)
                for name in SONARS
            },
        ],
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
            # label only, but it has to agree with what vector_odom publishes
            'odom_distance_units': typed('output_units', str),
        }],
    )

    # RPLidar C1 Node using rplidar_ros
    rplidar_c1 = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': 460800,  # Specific baudrate required for RPLidar C1
            'frame_id': LaunchConfiguration('lidar_frame_id'),
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    open_round_run = Node(
        package='autonomy',
        executable='open_round_run',
        name='open_round_run',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('disparity_params'),
            {
                'enable_auto_steering': typed('enable_auto_steering', bool),
                'require_button_start': typed('require_button_start', bool),
                'start_delay_sec': typed('start_delay_sec', float),
            },
        ],
    )

    return LaunchDescription(arguments + [
        mcu_bridge,
        vector_odom,
        lap_counter,
        rplidar_c1,
        open_round_run,
    ])