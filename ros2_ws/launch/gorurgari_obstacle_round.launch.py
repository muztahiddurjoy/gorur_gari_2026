"""
Obstacle round (Round 2 MVP): Full driving stack with vision-based Red & Green pillar avoidance.

    mcu_bridge          controls    serial link to the car - publishes
                                    encoder/count, encoder/speed, heading,
                                    steering_angle, sonar; drives /cmd_vel out
    vector_odom         controls    encoder/count + heading -> /odom_vector
    lap_counter         autonomy    /heading corners -> /lap_count, /turn_count
    rplidar_c1          sensors     serial link to LiDAR -> /scan
    vision_node         autonomy    /camera/image_raw -> /closest_obj ("R", "G", "N")
    disparity_extender  autonomy    fuses /scan + /closest_obj -> /cmd_vel
                                    Red pillar -> pass RIGHT | Green pillar -> pass LEFT

Not here yet: run_timer, the run stopwatch the open round launch file starts.
It follows a run state topic, and disparity_extender does not publish one - it
has no FINISHED state to stop the clock on, only "3 laps done, hold zero". Give
it a state topic and adding the node here is a copy of the open round block
plus a state_topic parameter. See ros2_ws/autonomy/docs/run_timer.md.

Run it from the workspace root:

    ros2 launch launch/gorurgari_obstacle_round.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BOT_CONFIG = os.path.join(WORKSPACE_DIR, 'config', 'bot_config.yaml')
DEFAULT_DISPARITY_PARAMS = os.path.join(WORKSPACE_DIR, 'config', 'disparity_extender_params.yaml')
DEFAULT_VISION_PARAMS = os.path.join(WORKSPACE_DIR, 'config', 'vision_params.yaml')

DEFAULT_WHEEL_DIAMETER_M = '0.065'
DEFAULT_COUNTS_PER_REV = '363'
DEFAULT_DISTANCE_SCALE = '0.976'

SONARS = ['front', 'left', 'right', 'rear']


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'wheel_diameter_m', default_value=DEFAULT_WHEEL_DIAMETER_M,
            description='Drive wheel diameter in metres.'),
        DeclareLaunchArgument(
            'encoder_counts_per_rev', default_value=DEFAULT_COUNTS_PER_REV,
            description='Encoder ticks per wheel revolution.'),
        DeclareLaunchArgument(
            'distance_scale', default_value=DEFAULT_DISTANCE_SCALE,
            description='Odometry calibration factor.'),
        DeclareLaunchArgument(
            'output_units', default_value='cm',
            description='Units of /odom_vector.'),
        DeclareLaunchArgument(
            'heading_clockwise', default_value='true',
            description='BNO055 yaw grows clockwise.'),

        DeclareLaunchArgument(
            'turn_angle_deg', default_value='90.0',
            description='Degrees per track corner.'),
        DeclareLaunchArgument(
            'turns_per_lap', default_value='4',
            description='Corners in one lap.'),

        DeclareLaunchArgument(
            'disparity_params', default_value=DEFAULT_DISPARITY_PARAMS,
            description='YAML of disparity_extender tuning parameters.'),
        DeclareLaunchArgument(
            'vision_params', default_value=DEFAULT_VISION_PARAMS,
            description='YAML of vision_node tuning parameters.'),

        DeclareLaunchArgument(
            'enable_auto_steering', default_value='true',
            description='Let disparity_extender drive /cmd_vel.'),
        DeclareLaunchArgument(
            'require_button_start', default_value='true',
            description='Hold disparity_extender in standby until the start button is pressed.'),
        DeclareLaunchArgument(
            'start_delay_sec', default_value='3.0',
            description='Seconds between start button press and first movement.'),

        # LiDAR Arguments
        DeclareLaunchArgument(
            'lidar_serial_port', default_value='/dev/ttyUSB0',
            description='Serial port for the RPLidar C1.'),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value='laser',
            description='Frame ID for LiDAR scans.'),
    ]

    arguments += [
        DeclareLaunchArgument(
            f'sonar_{name}_enabled', default_value='false',
            description=f'Publish the {name} sonar.')
        for name in SONARS
    ]

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
            'odom_distance_units': typed('output_units', str),
        }],
    )

    rplidar_c1 = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': 460800,
            'frame_id': LaunchConfiguration('lidar_frame_id'),
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    vision_node = Node(
        package='autonomy',
        executable='vision_node',
        name='vision_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('vision_params'),
        ],
    )

    disparity_extender = Node(
        package='autonomy',
        executable='disparity_extender',
        name='disparity_extender_node',
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
        vision_node,
        disparity_extender,
    ])
