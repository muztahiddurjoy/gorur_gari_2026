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
from collections import deque


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
        self.declare_parameter('heading_offset_deg', -180)
        # Flip if the car steers AWAY from the target: the servo mapping in
        # mcu_bridge (90 + angular.z * 45) decides which sign turns left.
        self.declare_parameter('invert_steering', False)
        # All distances below are in /odom_vector units (cm unless
        # vector_odom's output_units was changed). Label only.
        self.declare_parameter('odom_units', 'cm')
        self.declare_parameter('distance_tolerance', 10.0)
        # Stop if the car gets this much farther than its closest approach:
        # it crossed the target. Must be above odometry noise.
        self.declare_parameter('overshoot_margin', 10.0)
        # Adaptive throttle: full speed far away, ramping down to min_speed
        # once closer than slowdown_distance (odom units).
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('min_speed', 0.2)
        self.declare_parameter('slowdown_distance', 100.0)
        # Drive backward when the goal is behind the car (mcu_bridge maps
        # negative linear.x to negative throttle). Also lets the car back up
        # onto the exact point after coasting past it instead of giving up.
        self.declare_parameter('allow_reverse', True)
        # Anti-hunting failsafe: if the car keeps flipping forward/reverse
        # around one goal this many times, accept the position and move on
        # instead of shuttling over the point forever.
        self.declare_parameter('max_gear_flips', 6)

        self.heading_clockwise = self.get_parameter('heading_clockwise').value
        self.heading_offset_deg = float(self.get_parameter('heading_offset_deg').value)
        self.steer_sign = -1.0 if self.get_parameter('invert_steering').value else 1.0
        self.odom_units = self.get_parameter('odom_units').value
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.overshoot_margin = float(self.get_parameter('overshoot_margin').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.max_gear_flips = int(self.get_parameter('max_gear_flips').value)

        self.current_yaw_deg = None    # ROS convention, from /heading
        self.current_position = None
        self.target_position = None
        self.expected_heading = None   # bearing to target, recomputed every tick
        self.heading_tolerance = 2.0
        self.vel_data = Twist()
        self.timer = None              # keep track of the driving timer
        self.log_timer = None          # periodic distance logging timer
        self.min_distance = None       # closest distance reached so far (for overshoot detection)
        self.driving_reverse = False   # current drive direction (hysteresis state)
        self.gear_flips = 0            # direction changes on the current goal

        # FIFO goal queue with set semantics: a coordinate already pending
        # (or currently being driven to) is not enqueued again.
        self.goal_queue = deque()
        self.queued_keys = set()
        self.active_key = None         # key of the goal currently being driven to

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

    @staticmethod
    def goal_key(msg: Vector3):
        """Hashable identity of a goal; rounded so float noise doesn't defeat dedup."""
        return (round(msg.x, 2), round(msg.y, 2), round(msg.z, 2))

    def target_coord_callback(self, msg: Vector3):
        key = self.goal_key(msg)
        if key == self.active_key or key in self.queued_keys:
            self.get_logger().info(
                f'Ignoring duplicate goal x={msg.x}, y={msg.y}, z={msg.z}')
            return

        self.goal_queue.append(msg)
        self.queued_keys.add(key)
        self.get_logger().info(
            f'Queued goal x={msg.x}, y={msg.y}, z={msg.z} '
            f'({len(self.goal_queue)} waiting)')

        # Idle (no active goal): start driving to the first queued one.
        if self.target_position is None:
            self.start_next_goal()

    def start_next_goal(self):
        """Dequeue the next goal and begin driving toward it."""
        msg = self.goal_queue.popleft()
        self.active_key = self.goal_key(msg)
        self.queued_keys.discard(self.active_key)
        self.target_position = msg
        self.min_distance = None  # reset overshoot tracking for the new goal
        self.driving_reverse = False
        self.gear_flips = 0
        self.get_logger().info(
            f'Driving to goal x={msg.x}, y={msg.y}, z={msg.z} '
            f'({len(self.goal_queue)} remaining in queue)')

        # Create timers if not already running (avoid multiple timers).
        # 20 Hz so the reached-check fires before the car can coast far
        # past the tolerance circle.
        if self.timer is None:
            self.timer = self.create_timer(0.05, self.drive_toward_target)
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
        gear = 'REV' if self.driving_reverse else 'FWD'
        self.get_logger().info(
            f'Distance to goal: {distance:.1f} {self.odom_units} [{gear}] | '
            f'bearing {self.expected_heading:.1f} deg, yaw {self.current_yaw_deg:.1f} deg, '
            f'error {error:+.1f} deg, linear.x {self.vel_data.linear.x:+.2f}, '
            f'angular.z {self.vel_data.angular.z:+.2f}')

    def finish_goal(self, reason: str):
        """Current goal is done: move on to the next queued one, or stop."""
        self.get_logger().info(reason)
        self.target_position = None
        self.active_key = None
        self.min_distance = None
        if self.goal_queue:
            self.start_next_goal()
        else:
            self.stop_robot('Goal queue empty. Stopping.')

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

    def adaptive_speed(self, distance):
        """Throttle scaled by distance: max_speed far out, easing down to
        min_speed as the car closes within slowdown_distance of the goal."""
        if distance >= self.slowdown_distance:
            return self.max_speed
        frac = max(distance, 0.0) / self.slowdown_distance
        return self.min_speed + (self.max_speed - self.min_speed) * frac

    def stabilize_steer(self, reverse=False):
        if self.expected_heading is None or self.current_yaw_deg is None:
            self.vel_data.angular.z = 0.0
            return
        error = normalize_angle_deg(self.expected_heading - self.current_yaw_deg)
        if reverse:
            # Backing up: point the TAIL at the target, so the error is
            # measured against yaw + 180.
            error = normalize_angle_deg(error - 180.0)

        if abs(error) < self.heading_tolerance:
            self.vel_data.angular.z = 0.0
            return

        kp = 0.03
        angular_z = kp * error
        max_angular_z = 0.3
        angular_z = max(-max_angular_z, min(max_angular_z, angular_z))
        if reverse:
            # Driving backward flips how steering rotates the car (bicycle
            # model: yaw rate = v/L * tan(steer), and v < 0), so flip the
            # command to keep the loop converging.
            angular_z = -angular_z
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
            self.finish_goal('Reached target coordinates.')
            return

        # Overshoot bail-out only exists when the car cannot reverse back to
        # the point, and only once it has actually BEEN near the goal.
        # Arming it from the start stopped the car way early: while arcing
        # toward a far goal the distance also grows past min + margin.
        if (not self.allow_reverse
                and self.min_distance < self.slowdown_distance
                and distance > self.min_distance + self.overshoot_margin):
            self.finish_goal(
                f'Crossed the target: distance grew to {distance:.1f} {self.odom_units} '
                f'(closest approach was {self.min_distance:.1f} {self.odom_units}).'
            )
            return

        # Pick the drive direction with hysteresis (switch to reverse past
        # 100 deg off the nose, back to forward under 80 deg) so it cannot
        # chatter between gears when the goal sits right beside the car.
        error_fwd = normalize_angle_deg(self.expected_heading - self.current_yaw_deg)
        if self.allow_reverse:
            if not self.driving_reverse and abs(error_fwd) > 100.0:
                self.driving_reverse = True
                self.gear_flips += 1
            elif self.driving_reverse and abs(error_fwd) < 80.0:
                self.driving_reverse = False
                self.gear_flips += 1
        else:
            self.driving_reverse = False

        # Shuttling back and forth over the point means the tolerance circle
        # is smaller than the car can physically hit - accept and move on
        # rather than hunting forever.
        if self.gear_flips >= self.max_gear_flips:
            self.finish_goal(
                f'Hunted over the goal {self.gear_flips} times, settling at '
                f'{distance:.1f} {self.odom_units} away (closest approach '
                f'{self.min_distance:.1f} {self.odom_units}).'
            )
            return

        speed = self.adaptive_speed(distance)
        self.vel_data.linear.x = -speed if self.driving_reverse else speed
        self.stabilize_steer(reverse=self.driving_reverse)
        self.vel_publisher.publish(self.vel_data)


def main(args=None):
    rclpy.init(args=args)
    goto_controller = GoToController()
    rclpy.spin(goto_controller)
    goto_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
