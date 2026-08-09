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
        self.linear_scale  = 3   # mcu_bridge maps linear.x*100 to throttle -128..127
        # angular.z is NOT rad/s on this robot: mcu_bridge maps it as a
        # normalized steering fraction (90 + z*45 servo degrees), so 1.0 is
        # full lock. A scale of 3 makes the servo saturate at 1/3 stick.
        self.angular_scale = 3
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