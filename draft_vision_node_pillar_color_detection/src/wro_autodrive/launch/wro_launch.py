#!/usr/bin/env python3
"""
wro_launch.py - Master ROS 2 Launch File for WRO 2026 Future Engineers Autonomous Vehicle

Brings up all core stack nodes simultaneously:
1. vision_node
2. fusion_node
3. state_machine_node
4. drive_controller_node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate LaunchDescription object containing all node executions.
    """

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/Webots) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Vision Processing Node
    vision_node = Node(
        package='wro_autodrive',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_topic': '/camera/image_raw',
            'obstacles_topic': '/vision/obstacles'
        }]
    )

    # 2. Sensor Fusion Node
    fusion_node = Node(
        package='wro_autodrive',
        executable='fusion_node.py',
        name='fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vision_topic': '/vision/obstacles',
            'lidar_topic': '/lidar/scan',
            'fused_topic': '/fused/obstacles',
            'camera_fov_deg': 90.0,
            'camera_resolution_x': 640
        }]
    )

    # 3. Finite State Machine Node
    state_machine_node = Node(
        package='wro_autodrive',
        executable='state_machine_node.py',
        name='state_machine_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fused_obstacles_topic': '/fused/obstacles',
            'goal_topic': '/planner/drive_goal',
            'state_topic': '/robot/state',
            'fsm_loop_rate_hz': 20.0
        }]
    )

    # 4. Ackermann Drive Controller Node
    drive_controller_node = Node(
        package='wro_autodrive',
        executable='drive_controller_node.py',
        name='drive_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheelbase_m': 0.20,
            'max_steering_angle_rad': 0.523,
            'max_speed_mps': 1.5,
            'goal_topic': '/planner/drive_goal',
            'ackermann_cmd_topic': '/cmd_ackermann'
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        vision_node,
        fusion_node,
        state_machine_node,
        drive_controller_node
    ])
