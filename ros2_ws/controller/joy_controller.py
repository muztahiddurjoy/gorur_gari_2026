#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmdvel')

        # ======  CONFIGURE YOUR AXES HERE  ======
        self.linear_axis  = 1      # index of forward/backward axis
        self.angular_axis = 3      # index of rotation axis
        self.linear_scale  = 3   # max speed (m/s)
        self.angular_scale = 40   # max rotation (rad/s)
        # =======================================

        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info(
            f'Mapping axis {self.linear_axis} -> linear.x, '
            f'axis {self.angular_axis} -> angular.z')

    def joy_callback(self, msg: Joy):
        # Safety check – ignore if axes are too short
        if len(msg.axes) <= max(self.linear_axis, self.angular_axis):
            self.get_logger().warn('Joy message has too few axes', throttle_duration_sec=2)
            return

        twist = Twist()
        twist.linear.x  = msg.axes[self.linear_axis]  * self.linear_scale
        twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()