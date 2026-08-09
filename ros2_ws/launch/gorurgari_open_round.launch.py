import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('gorurgari_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gorurgari_bringup.launch.py')),
        ),
        Node(
            package='con',
            executable='gorurgari_bringup_node',
            name='gorurgari_bringup_node',
            output='screen',
        ),
    ])