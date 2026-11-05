"""
The whole challenge: arena, robot, random boxes, then the robot drives itself.

    ros2 launch fastrobox_sim competition.launch.py
    ros2 launch fastrobox_sim competition.launch.py seed:=3 box_count:=8
    ros2 launch fastrobox_sim competition.launch.py gui:=false
    ros2 launch fastrobox_sim competition.launch.py drive:=false    # teleop it yourself

The robot starts centred in the start zone, facing the finish. The node waits until
the boxes are in place, so it never sees a half-built track.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'fastrobox_sim'
WORLD = 'competition_arena'
# Base centre at the middle of the 0.2 m start zone; the 0.19 m robot just fits.
START_X = 0.1
# ROS Humble's cv_bridge is built against NumPy 1.x and crashes under a NumPy 2 in
# ~/.local, so anything importing it runs on the system packages only.
SYSTEM_PYTHON = {'PYTHONNOUSERSITE': '1'}


def nodes_after_spawn(context):
    """Start the boxes, then what depends on them, once the launch arguments are known."""
    layout_file = LaunchConfiguration('layout_file').perform(context)
    spawn_arguments = ['--layout-file', layout_file]
    for option, argument in (('--seed', 'seed'), ('--count', 'box_count')):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            spawn_arguments += [option, value]

    spawn_boxes = Node(package=PACKAGE, executable='spawn_boxes',
                       arguments=spawn_arguments, output='screen')
    followers = []
    if LaunchConfiguration('drive').perform(context) == 'true':
        followers.append(Node(
            package=PACKAGE, executable='smart_navigation', name='smart_navigation',
            parameters=[{'use_sim_time': True}], additional_env=SYSTEM_PYTHON,
            output='screen', emulate_tty=True))
        referee_arguments = ['--layout-file', layout_file]
        result_file = LaunchConfiguration('result_file').perform(context)
        if result_file:
            referee_arguments += ['--result-file', result_file]
        followers.append(Node(
            package=PACKAGE, executable='referee', name='referee',
            arguments=referee_arguments, parameters=[{'use_sim_time': True}],
            additional_env=SYSTEM_PYTHON, output='screen', emulate_tty=True))
    if not followers:
        return [spawn_boxes]
    return [spawn_boxes, RegisterEventHandler(OnProcessExit(
        target_action=spawn_boxes, on_exit=followers))]


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    world_file = os.path.join(share, 'worlds', f'{WORLD}.world')
    xacro_file = os.path.join(share, 'description', 'robot.urdf.xacro')

    arguments = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Ignition GUI. false is headless.'),
        DeclareLaunchArgument('seed', default_value='',
                              description='Box layout seed. Empty: a new layout each run.'),
        DeclareLaunchArgument('box_count', default_value='',
                              description='1 to 8 boxes. Empty: 6 to 8 at random.'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='Start smart_navigation.'),
        DeclareLaunchArgument('layout_file', default_value='/tmp/fastrobox_layout.yaml',
                              description='Where spawn_boxes records the layout.'),
        DeclareLaunchArgument('result_file', default_value='',
                              description="Where the referee writes its verdict as JSON."),
    ]

    gazebo_args = ['ign', 'gazebo', '-r', '-v', '2']
    gazebo_gui = ExecuteProcess(cmd=gazebo_args + [world_file],
                                condition=IfCondition(LaunchConfiguration('gui')),
                                output='screen')
    gazebo_headless = ExecuteProcess(cmd=gazebo_args + ['-s', '--headless-rendering', world_file],
                                     condition=UnlessCondition(LaunchConfiguration('gui')),
                                     output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': True,
        }],
        output='screen')

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', WORLD, '-topic', 'robot_description', '-name', 'fastrobox',
                   '-x', str(START_X), '-y', '0.0', '-z', '0.002'],
        output='screen')

    #   [  Ignition -> ROS     ]  ROS -> Ignition
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription(arguments + [
        gazebo_gui, gazebo_headless, robot_state_publisher, bridge, spawn_robot,
        OpaqueFunction(function=nodes_after_spawn),
    ])
