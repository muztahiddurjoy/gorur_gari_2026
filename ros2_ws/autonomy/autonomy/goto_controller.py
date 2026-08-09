import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float32
from math import radians, degrees, atan2, atan, hypot


class GoToController(Node):
    def __init__(self):
        super().__init__('goto_controller')
        self.current_heading = None
        self.current_position = None
        self.target_position = None
        self.expected_heading = None   # will be computed when both odom and target are known
        self.base_speed = 1.0
        self.heading_tolerance = 2.0
        self.distance_tolerance = 2.0
        self.overshoot_margin = 1.0    # stop if we get this much farther than the closest approach
        self.vel_data = Twist()
        self.timer = None              # keep track of the driving timer
        self.log_timer = None          # periodic distance logging timer
        self.min_distance = None       # closest distance reached so far (for overshoot detection)

        self.subscription = self.create_subscription(
            Vector3,
            'target_coord',
            self.target_coord_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Vector3,
            'odom_vector',
            self.odom_callback,
            10
        )
        self.heading_subscription = self.create_subscription(
            Float32,
            'heading',
            self.heading_callback,
            10
        )

        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('GoToController node has been started.')

    def odom_callback(self, msg: Vector3):
        self.current_position = msg

    def target_coord_callback(self, msg: Vector3):
        self.get_logger().info(f'Received target coordinates: x={msg.x}, y={msg.y}, z={msg.z}')
        self.target_position = msg

        # Only compute heading if we already have odometry
        if self.current_position is not None:
            self.expected_heading = degrees(atan2(msg.y - self.current_position.y,
                                                  msg.x - self.current_position.x))
            self.get_logger().info(f'Computed expected heading: {self.expected_heading:.2f} degrees')
            
        else:
            self.get_logger().warn('Target received before odometry. Heading will be computed when odom arrives.')

        # New target: reset overshoot tracking
        self.min_distance = None

        while True:
            # Compute heading if we now have both position and target
                    if self.expected_heading is None and self.current_position is not None and self.target_position is not None:
                        self.expected_heading = degrees(atan2(self.target_position.y - self.current_position.y,
                                                              self.target_position.x - self.current_position.x))
            
                    if self.current_position is None or self.current_heading is None or self.target_position is None:
                        self.get_logger().warn('Still waiting for position, heading or target.')
                        return
            
                    distance = self.distance_to_target()
            
                    if self.min_distance is None or distance < self.min_distance:
                        self.min_distance = distance
            
                    if distance < self.distance_tolerance:
                        self.stop_robot('Reached target coordinates.')
                    elif distance > self.min_distance + self.overshoot_margin:
                        self.stop_robot(
                            f'Crossed the target: distance grew to {distance:.2f} m '
                            f'(closest approach was {self.min_distance:.2f} m). Stopping.'
                        )
                    else:
                        self.vel_data.linear.x = self.base_speed
                        self.stabilize_steer()
                        self.vel_publisher.publish(self.vel_data)
            
        # Create timers if not already running (avoid multiple timers)
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.drive_toward_target)
        if self.log_timer is None:
            self.log_timer = self.create_timer(1.0, self.log_distance)

    def heading_callback(self, msg: Float32):
        self.current_heading = msg.data

    def distance_to_target(self):
        if self.current_position is None or self.target_position is None:
            return None
        return hypot(
            self.target_position.x - self.current_position.x,
            self.target_position.y - self.current_position.y
        )

    def log_distance(self):
        distance = self.distance_to_target()
        if distance is None:
            self.get_logger().info('Distance to goal: unknown (waiting for odom/target).')
        else:
            self.get_logger().info(f'Distance to goal: {distance:.2f} m')

    def stop_robot(self, reason: str):
        self.vel_data.linear.x = 0.0
        self.vel_data.angular.z = 0.0
        self.vel_publisher.publish(self.vel_data)
        self.get_logger().info(reason)
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.log_timer is not None:
            self.log_timer.cancel()
            self.log_timer = None
        self.min_distance = None

    def stabilize_steer(self):
        if self.expected_heading is None or self.current_heading is None:
            self.vel_data.angular.z = 0.0
            return
        error = self.expected_heading - self.current_heading
        error = (error + 180) % 360 - 180

        if abs(error) < self.heading_tolerance:
            self.vel_data.angular.z = 0.0
            return

        kp = 0.03
        angular_z = kp * error
        max_angular_z = 0.3
        angular_z = max(-max_angular_z, min(max_angular_z, angular_z))
        self.vel_data.angular.z = -angular_z

    def drive_toward_target(self):
        # Compute heading if we now have both position and target
        if self.expected_heading is None and self.current_position is not None and self.target_position is not None:
            self.expected_heading = degrees(atan2(self.target_position.y - self.current_position.y,
                                                  self.target_position.x - self.current_position.x))

        if self.current_position is None or self.current_heading is None or self.target_position is None:
            self.get_logger().warn('Still waiting for position, heading or target.')
            return

        distance = self.distance_to_target()

        if self.min_distance is None or distance < self.min_distance:
            self.min_distance = distance

        if distance < self.distance_tolerance:
            self.stop_robot('Reached target coordinates.')
        elif distance > self.min_distance + self.overshoot_margin:
            self.stop_robot(
                f'Crossed the target: distance grew to {distance:.2f} m '
                f'(closest approach was {self.min_distance:.2f} m). Stopping.'
            )
        else:
            self.vel_data.linear.x = self.base_speed
            self.stabilize_steer()
            self.vel_publisher.publish(self.vel_data)


def main(args=None):
    rclpy.init(args=args)
    goto_controller = GoToController()
    rclpy.spin(goto_controller)
    goto_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()