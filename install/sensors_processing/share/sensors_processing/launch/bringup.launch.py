"""
Bring up the whole robot sensing stack.

Starts the MCU serial bridge (raw encoder/heading/sonar topics + /cmd_vel out
to the car) and the odometry node that turns those into /odom and the
odom -> base_link transform, which is what RViz needs to draw the robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The wheel size scales every distance odometry reports, and nothing in the
# repo records it, so it stays an argument you are expected to set.
DEFAULT_WHEEL_DIAMETER_M = '0.065'
# matches ENCODER_COUNTS_PER_REV in firmware/include/config.h
DEFAULT_COUNTS_PER_REV = '1320'

SONARS = ['front', 'left', 'right', 'rear']


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'wheel_diameter_m', default_value=DEFAULT_WHEEL_DIAMETER_M,
            description='Drive wheel diameter in metres. Measure this, the default is a guess.'),
        DeclareLaunchArgument(
            'encoder_counts_per_rev', default_value=DEFAULT_COUNTS_PER_REV,
            description='Encoder ticks per wheel revolution, must match the firmware.'),
        DeclareLaunchArgument(
            'publish_tf', default_value='true',
            description='Broadcast odom -> base_link. Turn off if something else owns that TF.'),
        DeclareLaunchArgument(
            'heading_clockwise', default_value='true',
            description='BNO055 yaw grows clockwise, ROS grows counter clockwise. '
                        'Flip if the robot turns the wrong way in RViz.'),
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
        package='contols',
        executable='mcu_bridge',
        name='mcu_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            f'sonar_{name}_enabled': typed(f'sonar_{name}_enabled', bool)
            for name in SONARS
        }],
    )

    encoder_odometry = Node(
        package='sensors_processing',
        executable='encoder_odometry',
        name='encoder_odometry',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'wheel_diameter_m': typed('wheel_diameter_m', float),
            'encoder_counts_per_rev': typed('encoder_counts_per_rev', int),
            'publish_tf': typed('publish_tf', bool),
            'heading_clockwise': typed('heading_clockwise', bool),
        }],
    )

    return LaunchDescription(arguments + [mcu_bridge, encoder_odometry])
