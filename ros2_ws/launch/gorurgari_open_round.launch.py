"""
Open round: the full driving stack, no obstacles on the mat.

    mcu_bridge          controls    serial link to the car - publishes
                                    encoder/count, encoder/speed, heading,
                                    steering_angle, sonar; drives /cmd_vel out
    vector_odom         controls    encoder/count + heading -> /odom_vector
                                    (and the odom -> base_link TF)
    lap_counter         autonomy    /heading corners -> /lap_count, /turn_count
    disparity_extender  autonomy    /scan -> /cmd_vel, and stops the car once
                                    /lap_count reaches the round's lap target

Everything talks on relative topic names in the root namespace, so the nodes
wire up to each other with no remaps. The LiDAR driver itself is NOT started
here - bring up whatever publishes /scan separately, this file only consumes it.

Run it from the workspace root:

    ros2 launch launch/gorurgari_open_round.launch.py

Tuning for disparity_extender lives in config/disparity_extender_params.yaml
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
DEFAULT_DISPARITY_PARAMS = os.path.join(
    WORKSPACE_DIR, 'config', 'disparity_extender_params.yaml')

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
            'disparity_params', default_value=DEFAULT_DISPARITY_PARAMS,
            description='YAML of disparity_extender tuning parameters.'),
        DeclareLaunchArgument(
            'enable_auto_steering', default_value='true',
            description='Let disparity_extender drive /cmd_vel. False to watch it '
                        'plan without the car moving.'),
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
        parameters=[{
            f'sonar_{name}_enabled': typed(f'sonar_{name}_enabled', bool)
            for name in SONARS
        }],
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

    # The node name has to stay disparity_extender_node - that is the key the
    # tuning yaml is written under, and a rename silently drops every value.
    disparity_extender = Node(
        package='autonomy',
        executable='disparity_extender',
        name='disparity_extender_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('disparity_params'),
            {'enable_auto_steering': typed('enable_auto_steering', bool)},
        ],
    )

    return LaunchDescription(arguments + [
        mcu_bridge,
        vector_odom,
        lap_counter,
        disparity_extender,
    ])
