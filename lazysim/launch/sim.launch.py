"""
The simulator on its own — Ignition, the car, the bridge and the track.

Brings up everything that stands in for hardware, and nothing that
drives. Use it to poke at the car by hand:

    ros2 launch lazysim sim.launch.py
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\
        '{linear: {x: 0.3}, angular: {z: 0.4}}'

For a full round, launch open_round_sim.launch.py or
obstacle_round_sim.launch.py instead — both include this file.

    ign gazebo ─┬─ gpu_lidar ─┐
                ├─ imu        ├─ ros_gz_bridge ─┬─ /scan /imu /odom
                ├─ camera ────┘                 ├─ /camera/image_raw
                └─ joint controllers ◀──────────┘  /lazybot/joint_states
                                                          │
                     /cmd_vel ──▶ lazybridge ◀────────────┘
                                      │
                                      └──▶ encoder/count, heading,
                                           steering_angle, /joint_states

Adapted from Team LazyGo's simulator:
https://github.com/A-N-M-Noor/LazyGo_WRO2025/
For testing purposes only — none of this runs on the competition car.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'lazysim'
# Must match <world name="..."> in worlds/lazyWorld.sdf: the Ignition
# services track_maker calls are scoped by it.
WORLD_NAME = 'lazyWorld'
# Must match the -name given to `create` below, because the Ignition
# controller plugin topics in control.xacro are scoped by model name.
MODEL_NAME = 'lazyBot'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    xacro_file = os.path.join(share, 'description', 'lazyBot.xacro')
    world_file = os.path.join(share, 'worlds', 'lazyWorld.sdf')
    rviz_file = os.path.join(share, 'config', 'lazySim.rviz')
    default_track = os.path.join(share, 'config', 'track.yaml')
    default_params = os.path.join(share, 'config', 'bot_config_sim.yaml')

    arguments = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Ignition GUI. false is headless '
                                          '(server only), which is much faster.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz on the sim.'),
        DeclareLaunchArgument('verbosity', default_value='2',
                              description='Ignition log level, 0 quiet .. 4 debug.'),
        DeclareLaunchArgument('track_config', default_value=default_track,
                              description='Pillar / parking / start layout YAML.'),
        DeclareLaunchArgument('bot_config', default_value=default_params,
                              description='Parameters for lazybridge and the autonomy '
                                          'stack. Defaults to this package\'s copy; '
                                          'point it at ros2_ws/config/bot_config.yaml '
                                          'to run the real tuning.'),
        DeclareLaunchArgument('build_track', default_value='true',
                              description='Spawn the pillars and place the car from '
                                          'track_config. false leaves a bare mat.'),
    ]

    # Ignition resolves any relative mesh/texture path against this.
    resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(share, 'worlds'),
            share,
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ]).rstrip(os.pathsep))

    # -r starts the world unpaused; -s runs the server with no GUI.
    # Two separate actions rather than one with a conditional flag,
    # because an empty argv element is not the same as no argument and
    # `ign gazebo` treats it as an unparseable world name.
    gazebo_args = ['ign', 'gazebo', '-r', '-v', LaunchConfiguration('verbosity')]
    gazebo_gui = ExecuteProcess(
        cmd=gazebo_args + [world_file],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen',
    )
    gazebo_headless = ExecuteProcess(
        cmd=gazebo_args + ['-s', world_file],
        condition=UnlessCondition(LaunchConfiguration('gui')),
        output='screen',
    )

    # robot_state_publisher owns TF for everything on the car. It is
    # fed by lazybridge's rate-limited /joint_states, not by Ignition
    # directly, which publishes one joint state per physics step.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-topic', 'robot_description',
            '-name', MODEL_NAME,
            # Just clear of the mat so it settles onto its wheels rather
            # than starting interpenetrated with the floor.
            '-z', '0.02',
        ],
        output='screen',
    )

    # ros_gz_bridge maps one topic to one topic and cannot rename, so
    # every name here is identical on both sides.
    #   [  Ignition -> ROS
    #   ]  ROS -> Ignition
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Sim time, so every node below agrees with the physics clock.
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

            # Sensors
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

            # Joint feedback (1 kHz — lazybridge decimates it) and the
            # simulator's ground truth pose.
            '/lazybot/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/lazybot/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

            # Actuators
            '/lazybot/steer_left@std_msgs/msg/Float64]ignition.msgs.Double',
            '/lazybot/steer_right@std_msgs/msg/Float64]ignition.msgs.Double',
            '/lazybot/drive_left@std_msgs/msg/Float64]ignition.msgs.Double',
            '/lazybot/drive_right@std_msgs/msg/Float64]ignition.msgs.Double',
            '/lazybot/cam_servo@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        remappings=[
            # disparity_extender subscribes to /odom.
            ('/lazybot/odom', '/odom'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    lazybridge = Node(
        package=PACKAGE,
        executable='lazybridge',
        name='lazybridge',
        parameters=[LaunchConfiguration('bot_config'), {'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
    )

    track_maker = Node(
        package=PACKAGE,
        executable='track_maker',
        name='track_maker',
        parameters=[{
            'world_name': WORLD_NAME,
            'tower_template_path': os.path.join(share, 'config', 'object_template.sdf'),
            'wall_template_path': os.path.join(share, 'config', 'wall_template.sdf'),
            'settings_path': LaunchConfiguration('track_config'),
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('build_track')),
        output='screen',
        emulate_tty=True,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )

    return LaunchDescription(arguments + [
        resource_path,
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        bridge,
        lazybridge,
        # Ignition needs a moment to stand the world up before it will
        # answer a spawn request, and the track can only be laid out
        # once the car it repositions actually exists.
        TimerAction(period=4.0, actions=[spawn]),
        TimerAction(period=7.0, actions=[track_maker]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
