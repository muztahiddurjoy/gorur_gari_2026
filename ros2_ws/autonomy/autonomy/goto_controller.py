"""
Drive to a target coordinate published on /target_coord.

Frames and units, so the steering math is consistent end to end:
- /odom_vector comes from vector_odom in that node's output_units (cm by
  default). x grows along yaw 0, y grows counter clockwise (REP-103).
- /heading is the RAW MCU compass heading in DEGREES, growing CLOCKWISE
  (the same number the MCU's OLED shows). It is negated on arrival so this
  node and vector_odom agree on one yaw convention. vector_odom must run
  with zero_heading_on_start:=false (now the default) or the odom frame
  and this yaw would disagree by the startup heading.
- The bearing to the target is recomputed EVERY control tick from the
  latest odometry, so the steering keeps correcting while the car moves.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float32
from math import degrees, atan2, hypot


def normalize_angle_deg(angle):
    """Wrap to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


class GoToController(Node):
    def __init__(self):
        super().__init__('goto_controller')

        # Same convention parameter as vector_odom: the BNO055 heading grows
        # clockwise, ROS yaw grows counter clockwise.
        self.declare_parameter('heading_clockwise', True)
        # The BNO055 is mounted at the rear of the car facing BACKWARD, so it
        # reports the direction the tail points. Added to the raw heading to
        # get the direction the car actually drives. Must match vector_odom.
        self.declare_parameter('heading_offset_deg', 180.0)
        # Flip if the car steers AWAY from the target: the servo mapping in
        # mcu_bridge (90 + angular.z * 45) decides which sign turns left.
        self.declare_parameter('invert_steering', False)
        # All distances below are in /odom_vector units (cm unless
        # vector_odom's output_units was changed). Label only.
        self.declare_parameter('odom_units', 'cm')
        self.declare_parameter('distance_tolerance', 5.0)
        # Stop if the car gets this much farther than its closest approach:
        # it crossed the target. Must be above odometry noise.
        self.declare_parameter('overshoot_margin', 10.0)

        self.heading_clockwise = self.get_parameter('heading_clockwise').value
        self.heading_offset_deg = float(self.get_parameter('heading_offset_deg').value)
        self.steer_sign = -1.0 if self.get_parameter('invert_steering').value else 1.0
        self.odom_units = self.get_parameter('odom_units').value
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.overshoot_margin = float(self.get_parameter('overshoot_margin').value)

        self.current_yaw_deg = None    # ROS convention, from /heading
        self.current_position = None
        self.target_position = None
        self.expected_heading = None   # bearing to target, recomputed every tick
        self.base_speed = 1.0
        self.heading_tolerance = 2.0
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

    def heading_callback(self, msg: Float32):
        """Convert the MCU compass heading (deg, clockwise) into a ROS yaw in degrees."""
        heading = msg.data + self.heading_offset_deg  # undo the backward IMU mounting
        yaw = -heading if self.heading_clockwise else heading
        self.current_yaw_deg = normalize_angle_deg(yaw)

    def target_coord_callback(self, msg: Vector3):
        self.get_logger().info(f'Received target coordinates: x={msg.x}, y={msg.y}, z={msg.z}')
        self.target_position = msg

        # New target: reset overshoot tracking
        self.min_distance = None

        # Create timers if not already running (avoid multiple timers)
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.drive_toward_target)
        if self.log_timer is None:
            self.log_timer = self.create_timer(1.0, self.log_distance)

    def distance_to_target(self):
        if self.current_position is None or self.target_position is None:
            return None
        return hypot(
            self.target_position.x - self.current_position.x,
            self.target_position.y - self.current_position.y
        )

    def bearing_to_target(self):
        """Direction from the car to the target in the odom frame, degrees CCW."""
        if self.current_position is None or self.target_position is None:
            return None
        return degrees(atan2(self.target_position.y - self.current_position.y,
                             self.target_position.x - self.current_position.x))

    def log_distance(self):
        distance = self.distance_to_target()
        if distance is None or self.current_yaw_deg is None:
            self.get_logger().info('Distance to goal: unknown (waiting for odom/heading/target).')
            return
        error = normalize_angle_deg(self.expected_heading - self.current_yaw_deg) \
            if self.expected_heading is not None else 0.0
        self.get_logger().info(
            f'Distance to goal: {distance:.1f} {self.odom_units} | '
            f'bearing {self.expected_heading:.1f} deg, yaw {self.current_yaw_deg:.1f} deg, '
            f'error {error:+.1f} deg, angular.z {self.vel_data.angular.z:+.2f}')

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
        if self.expected_heading is None or self.current_yaw_deg is None:
            self.vel_data.angular.z = 0.0
            return
        error = normalize_angle_deg(self.expected_heading - self.current_yaw_deg)

        if abs(error) < self.heading_tolerance:
            self.vel_data.angular.z = 0.0
            return

        kp = 0.03
        angular_z = kp * error
        max_angular_z = 0.3
        angular_z = max(-max_angular_z, min(max_angular_z, angular_z))
        # positive error = target is to the left (CCW); positive angular.z
        # turns left per REP-103, invert_steering flips it for the servo
        self.vel_data.angular.z = self.steer_sign * angular_z

    def drive_toward_target(self):
        if self.current_position is None or self.current_yaw_deg is None or self.target_position is None:
            self.get_logger().warn('Still waiting for position, heading or target.')
            return

        # Recompute the bearing from the CURRENT position every tick, so the
        # steering target moves with the car instead of freezing at the value
        # it had when the target arrived.
        self.expected_heading = self.bearing_to_target()
        distance = self.distance_to_target()

        if self.min_distance is None or distance < self.min_distance:
            self.min_distance = distance

        if distance < self.distance_tolerance:
            self.stop_robot('Reached target coordinates.')
        elif distance > self.min_distance + self.overshoot_margin:
            self.stop_robot(
                f'Crossed the target: distance grew to {distance:.1f} {self.odom_units} '
                f'(closest approach was {self.min_distance:.1f} {self.odom_units}). Stopping.'
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
