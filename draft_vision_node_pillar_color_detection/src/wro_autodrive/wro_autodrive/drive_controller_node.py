#!/usr/bin/env python3
"""
drive_controller_node.py - Ackermann Drive Controller Node for WRO 2026 Future Engineers

Translates high-level path planning goals (/planner/drive_goal) into low-level Ackermann steering
and propulsion commands (/cmd_ackermann) tailored for 1-driving-axle, 1-steering-actuator vehicles.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
try:
    from ackermann_msgs.msg import AckermannDriveStamped
    ACKERMANN_MSGS_AVAILABLE = True
except ImportError:
    ACKERMANN_MSGS_AVAILABLE = False


class DriveControllerNode(Node):
    """
    ROS 2 Controller Node translating velocity/heading goals into Ackermann drive geometry outputs.
    """

    def __init__(self) -> None:
        super().__init__('drive_controller_node')

        # --- Vehicle Geometry & Kinematic Parameters ---
        self.declare_parameter('wheelbase_m', 0.20)         # Vehicle wheelbase (meters)
        self.declare_parameter('max_steering_angle_rad', 0.523) # Max steering lock (~30 deg)
        self.declare_parameter('max_speed_mps', 1.5)        # Max linear speed limit
        self.declare_parameter('goal_topic', '/planner/drive_goal')
        self.declare_parameter('ackermann_cmd_topic', '/cmd_ackermann')

        self.wheelbase = self.get_parameter('wheelbase_m').get_parameter_value().double_value
        self.max_steer = self.get_parameter('max_steering_angle_rad').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed_mps').get_parameter_value().double_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        ackermann_topic = self.get_parameter('ackermann_cmd_topic').get_parameter_value().string_value

        # --- Callback Group Setup ---
        self.cb_group = MutuallyExclusiveCallbackGroup()

        # --- Publishers & Subscribers ---
        if ACKERMANN_MSGS_AVAILABLE:
            self.ackermann_pub = self.create_publisher(
                AckermannDriveStamped,
                ackermann_topic,
                qos_profile=10
            )
        else:
            # Fallback to Twist message if ackermann_msgs is not installed
            self.ackermann_pub = self.create_publisher(
                Twist,
                ackermann_topic,
                qos_profile=10
            )

        self.goal_sub = self.create_subscription(
            Twist,
            goal_topic,
            self._goal_callback,
            qos_profile=10,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            f'DriveControllerNode initialized (Wheelbase={self.wheelbase}m, MaxSteer={math.degrees(self.max_steer):.1f}°).'
        )

    def _goal_callback(self, msg: Twist) -> None:
        """
        Calculates Ackermann steering angle from requested linear velocity (v) and angular velocity (w).

        Ackermann Kinematics:
        steering_angle delta = arctan(w * L / v)
        """
        v = msg.linear.x
        w = msg.angular.z

        # Clamp linear velocity to vehicle maximums
        target_speed = max(min(v, self.max_speed), -self.max_speed)

        # Compute Ackermann steering angle
        if abs(v) > 1e-4 and abs(w) > 1e-4:
            steering_angle = math.atan2(w * self.wheelbase, v)
        else:
            steering_angle = w  # Direct angular assignment if stationary/slow

        # Clamp steering angle to mechanical limits
        clamped_steering_angle = max(min(steering_angle, self.max_steer), -self.max_steer)

        self._publish_drive_command(target_speed, clamped_steering_angle)

    def _publish_drive_command(self, speed: float, steering_angle: float) -> None:
        """
        Formats and publishes drive command to motor controller / hardware bridge.
        """
        if ACKERMANN_MSGS_AVAILABLE:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.drive.speed = speed
            cmd.drive.steering_angle = steering_angle
            self.ackermann_pub.publish(cmd)
        else:
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = steering_angle
            self.ackermann_pub.publish(cmd)

        self.get_logger().debug(f'Published Ackermann drive: speed={speed:.2f} m/s, steer={math.degrees(steering_angle):.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = DriveControllerNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('DriveControllerNode stopping via KeyboardInterrupt.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
