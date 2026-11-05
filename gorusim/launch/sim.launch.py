"""
The simulator on its own: Ignition, the car, the virtual ESP32, the real mcu_bridge
and the track. Everything that stands in for hardware, and nothing that drives.

    ros2 launch gorusim sim.launch.py
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\
        '{linear: {x: 0.6}, angular: {z: 0.4}}'

For a full round, launch open_round_sim.launch.py or obstacle_round_sim.launch.py
instead; both include this file.

    ign gazebo --- gpu_lidar, camera ---> ros_gz_bridge ---> /scan, /camera/image_raw
        |  ^------ motor, kingpins <----- ros_gz_bridge <---+
        +--------- imu, joint states ---> ros_gz_bridge --> sim_mcu (firmware emulation)
                                                               ^
                                               MAVLink over a pty, as on the USB cable
                                                               v
              /cmd_vel --> mcu_bridge (real) --> encoder/*, heading, steering_angle,
                                                 /button_status

Track tooling adapted from Team LazyGo's simulator:
https://github.com/A-N-M-Noor/LazyGo_WRO2025/
For testing purposes only: none of this runs on the competition car.
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'gorusim'
MODEL_NAME = 'goruBot'


def spawn_on_start_pose(context):
    """Spawn the car where track.yaml says, rather than teleporting it there later."""
    with open(LaunchConfiguration('track_config').perform(context), 'r') as handle:
        start = (yaml.safe_load(handle) or {}).get('start') or {}
    yaw = math.radians(float(start.get('angle', 0.0)))
    offset = float(start.get('nose_offset_m', 0.1205))
    position = start.get('pos', {'x': 0.0, 'y': 0.0})
    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', LaunchConfiguration('world').perform(context),
            '-topic', 'robot_description',
            '-name', MODEL_NAME,
            '-x', str(float(position['x']) - offset * math.cos(yaw)),
            '-y', str(float(position['y']) - offset * math.sin(yaw)),
            # Just clear of the mat, so it settles onto its tyres.
            '-z', '0.002',
            '-Y', str(yaw),
        ],
        output='screen',
    )]


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    config = os.path.join(share, 'config')
    xacro_file = os.path.join(share, 'description', 'goruBot.xacro')
    # File and <world name="..."> share a name, which the spawn and the Ignition
    # services are scoped by.
    world_file = PathJoinSubstitution(
        [share, 'worlds', [LaunchConfiguration('world'), '.sdf']])

    arguments = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Ignition GUI. false is headless.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('verbosity', default_value='2',
                              description='Ignition log level, 0 quiet .. 4 debug.'),
        DeclareLaunchArgument('world', default_value='goruWorld',
                              description='goruWorld is the mat, flatWorld an open floor '
                                          'for tools/verify_dynamics.py.'),
        DeclareLaunchArgument('track_config',
                              default_value=os.path.join(config, 'track.yaml'),
                              description='Pillar / parking / start layout YAML.'),
        DeclareLaunchArgument('build_track', default_value='true',
                              description='Spawn what track_config describes. '
                                          'false leaves a bare mat.'),
    ]

    # Meshes resolve as model://gorusim/meshes/..., against this path.
    resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(share, 'worlds'),
            os.path.dirname(share),
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ]).rstrip(os.pathsep))

    # -r starts unpaused. --headless-rendering lets the server render the LiDAR and
    # camera with no display attached. Two actions rather than a conditional flag,
    # because `ign gazebo` reads an empty argv element as a world name.
    gazebo_args = ['ign', 'gazebo', '-r', '-v', LaunchConfiguration('verbosity')]
    gazebo_gui = ExecuteProcess(
        cmd=gazebo_args + [world_file],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen',
    )
    gazebo_headless = ExecuteProcess(
        cmd=gazebo_args + ['-s', '--headless-rendering', world_file],
        condition=UnlessCondition(LaunchConfiguration('gui')),
        output='screen',
    )

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

    #   [  Ignition -> ROS     ]  ROS -> Ignition
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/gorubot/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/gorubot/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/gorubot/motor_target@std_msgs/msg/Float64]ignition.msgs.Double',
            '/gorubot/steer_left@std_msgs/msg/Float64]ignition.msgs.Double',
            '/gorubot/steer_right@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    sim_mcu = Node(
        package=PACKAGE,
        executable='sim_mcu',
        name='sim_mcu',
        parameters=[
            os.path.join(config, 'sim_hardware.yaml'),
            {
                'use_sim_time': True,
                'cad_geometry_path': os.path.join(config, 'cad_geometry.yaml'),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    # The real thing. It opens the port once, at startup, so the virtual ESP32 has to
    # exist first. Same mistake on the bench: bridge up before the USB cable is in.
    mcu_bridge = Node(
        package='controls',
        executable='mcu_bridge',
        name='mcu_bridge',
        parameters=[
            os.path.join(config, 'bot_config.yaml'),
            os.path.join(config, 'sim_overlay.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
        emulate_tty=True,
    )
    bridge_after_mcu = RegisterEventHandler(OnProcessStart(
        target_action=sim_mcu,
        on_start=[TimerAction(period=1.5, actions=[mcu_bridge])],
    ))

    track_maker = Node(
        package=PACKAGE,
        executable='track_maker',
        name='track_maker',
        parameters=[{
            'world_name': LaunchConfiguration('world'),
            'tower_template_path': os.path.join(config, 'object_template.sdf'),
            'wall_template_path': os.path.join(config, 'wall_template.sdf'),
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
        arguments=['-d', os.path.join(config, 'goruSim.rviz')],
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
        sim_mcu,
        bridge_after_mcu,
        # Ignition needs a moment to stand the world up before it answers a spawn.
        TimerAction(period=4.0, actions=[OpaqueFunction(function=spawn_on_start_pose)]),
        TimerAction(period=7.0, actions=[track_maker]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
