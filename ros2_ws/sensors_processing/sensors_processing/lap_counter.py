"""
Lap counter: counts laps on a box track by detecting the corners.

The track is a closed loop of equal corners (a square box is four 90 degree
turns), so a lap is simply turns_per_lap corners taken in the same direction.
Nothing here needs position, only /heading, which means the count does not
drift with the encoder odometry.

How a turn is detected: the raw compass heading is unwrapped into a running
total that keeps growing past 360, and that total is quantised into
turn_angle_deg sized steps. A corner is confirmed once the heading has moved
turn_confirm_ratio of a full corner away from the last confirmed one, and the
mark then advances by a WHOLE corner - so the last few degrees of the corner,
and any wobble on the straight after it, count against the next corner instead
of piling up into a phantom one.

Turns are signed, so a corner taken the wrong way (recovering from an
overshoot, or a reversed lap) subtracts instead of adding, and only a full set
of turns_per_lap in one direction makes a lap.

Publishes /lap_count and /turn_count (both latched, so a node started later
still sees the current values) and logs every corner and every lap.

Anything published on /reset_lap_count zeroes both counts and starts the timing
and distance over, as if the node had just come up, without losing the heading
tracking - use it between runs instead of restarting the node.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, Float32, Int32
from geometry_msgs.msg import Vector3


def normalize_angle_deg(angle):
    """Wrap to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


class LapCounterNode(Node):
    def __init__(self):
        super().__init__('lap_counter')

        # -- track shape --
        # Degrees per corner. 90 for a box, 120 for a triangle, and so on.
        self.declare_parameter('turn_angle_deg', 90.0)
        # Corners in one lap. 4 * 90 = one full 360 turn around a box.
        self.declare_parameter('turns_per_lap', 4)

        # -- turn detection --
        # Fraction of a corner that has to be turned before the corner counts.
        # Too low and a hard swerve on a straight registers as a corner, too
        # high and a corner cut short is missed.
        self.declare_parameter('turn_confirm_ratio', 0.75)
        # Corners can never be this close together, so a jittery heading around
        # the confirm threshold cannot register twice.
        self.declare_parameter('min_turn_interval_sec', 0.5)
        # A single heading message this far from the previous one is an IMU
        # glitch, not real rotation, and is dropped. At the MCU's heading rate
        # the car cannot physically turn this much between two messages.
        self.declare_parameter('max_heading_step_deg', 45.0)

        # The BNO055 heading grows clockwise, ROS yaw grows counter clockwise
        # (REP-103). Only affects which way a turn is labelled.
        self.declare_parameter('heading_clockwise', True)

        # -- logging --
        # Seconds between progress lines while driving. 0 disables them; the
        # turn and lap lines are always logged.
        self.declare_parameter('log_period_sec', 1.0)
        # Only the label used when logging the distance from /odom_vector,
        # which arrives already scaled by vector_odom's output_units.
        self.declare_parameter('odom_distance_units', 'cm')

        self.turn_angle_deg = float(self.get_parameter('turn_angle_deg').value)
        self.turns_per_lap = int(self.get_parameter('turns_per_lap').value)
        turn_confirm_ratio = float(self.get_parameter('turn_confirm_ratio').value)
        self.min_turn_interval_sec = float(self.get_parameter('min_turn_interval_sec').value)
        self.max_heading_step_deg = float(self.get_parameter('max_heading_step_deg').value)
        self.heading_clockwise = self.get_parameter('heading_clockwise').value
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.distance_units = self.get_parameter('odom_distance_units').value

        if self.turn_angle_deg <= 0.0:
            raise ValueError('turn_angle_deg must be positive')
        if self.turns_per_lap <= 0:
            raise ValueError('turns_per_lap must be positive')
        if not 0.0 < turn_confirm_ratio <= 1.0:
            raise ValueError('turn_confirm_ratio must be in (0, 1]')
        self.turn_confirm_deg = self.turn_angle_deg * turn_confirm_ratio

        # heading state, all in degrees, ROS convention (counter clockwise up)
        self.last_raw_yaw = None
        self.cumulative_yaw = 0.0
        # cumulative_yaw at the last confirmed corner
        self.turn_mark = 0.0

        self.turn_count = 0
        # signed, so a corner the wrong way undoes one, and reset every lap
        self.net_turns = 0
        self.lap_count = 0

        self.last_turn_time = None
        self.start_time = self.now_sec()
        self.lap_start_time = self.start_time

        # None until /odom_vector arrives; distance is only for the log line
        self.total_distance = None
        self.lap_start_distance = None

        # latched, so whatever starts after us still gets the current count
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.lap_pub = self.create_publisher(Int32, 'lap_count', latched)
        self.turn_pub = self.create_publisher(Int32, 'turn_count', latched)
        
        self.heading_sub = self.create_subscription(
            Float32, 'heading', self.heading_callback, 10)
        self.odom_sub = self.create_subscription(
            Vector3, 'odom_vector', self.odom_callback, 10)
        self.reset_sub = self.create_subscription(
            Empty, 'reset_lap_count', self.reset_callback, 10)

        self.lap_pub.publish(Int32(data=self.lap_count))
        self.turn_pub.publish(Int32(data=self.turn_count))

        self.get_logger().info(
            f'Lap counter: {self.turns_per_lap} turns of {self.turn_angle_deg:g} deg per lap, '
            f'a turn confirms at {self.turn_confirm_deg:.1f} deg, '
            f'publishing /lap_count and /turn_count, '
            f'reset with /reset_lap_count')

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def heading_callback(self, msg: Float32):
        yaw = -msg.data if self.heading_clockwise else msg.data

        # The first heading is only a baseline, otherwise the compass reading
        # itself would land in the first step and count as rotation.
        if self.last_raw_yaw is None:
            self.last_raw_yaw = yaw
            return

        step = normalize_angle_deg(yaw - self.last_raw_yaw)
        self.last_raw_yaw = yaw

        if abs(step) > self.max_heading_step_deg:
            self.get_logger().warn(
                f'Heading jumped {step:.1f} deg in one message, ignoring it',
                throttle_duration_sec=1.0)
            return

        # unwrapped, so it keeps counting past 360 instead of wrapping
        self.cumulative_yaw += step
        progress = self.cumulative_yaw - self.turn_mark

        if abs(progress) >= self.turn_confirm_deg and self.turn_is_due():
            self.register_turn(1 if progress > 0.0 else -1)
        elif self.log_period_sec > 0.0:
            self.get_logger().info(
                f'lap {self.lap_count} | turn {self.net_turns}/{self.turns_per_lap} | '
                f'into next turn {progress:+.1f}/{self.turn_angle_deg:g} deg | '
                f'heading {msg.data:.1f} deg',
                throttle_duration_sec=self.log_period_sec)

    def turn_is_due(self):
        """False while still inside the dead time after the last turn."""
        if self.last_turn_time is None:
            return True
        return self.now_sec() - self.last_turn_time >= self.min_turn_interval_sec

    def register_turn(self, direction):
        now = self.now_sec()
        self.turn_count += 1
        self.net_turns += direction
        self.last_turn_time = now
        # a whole corner, not the confirmed amount, so the rest of this corner
        # is carried into the next one instead of being counted twice
        self.turn_mark += direction * self.turn_angle_deg

        self.turn_pub.publish(Int32(data=self.turn_count))
        self.get_logger().info(
            f'Turn {self.turn_count} ({"left" if direction > 0 else "right"}, '
            f'{self.turn_angle_deg:g} deg) - {abs(self.net_turns)}/{self.turns_per_lap} '
            f'towards lap {self.lap_count + 1}, total heading {self.cumulative_yaw:.1f} deg')

        if abs(self.net_turns) >= self.turns_per_lap:
            self.register_lap(direction, now)

    def register_lap(self, direction, now):
        self.lap_count += 1
        # keep the remainder rather than zeroing, so a lap with one extra
        # corner in it does not lose that corner from the next lap
        self.net_turns -= direction * self.turns_per_lap

        lap_time = now - self.lap_start_time
        self.lap_start_time = now

        distance = ''
        if self.total_distance is not None and self.lap_start_distance is not None:
            distance = (f', {self.total_distance - self.lap_start_distance:.1f} '
                        f'{self.distance_units}')
        self.lap_start_distance = self.total_distance

        self.lap_pub.publish(Int32(data=self.lap_count))
        self.get_logger().info(
            f'=== LAP {self.lap_count} COMPLETE === '
            f'{"counter clockwise" if direction > 0 else "clockwise"}, '
            f'{lap_time:.1f} s{distance}, {self.turn_count} turns, '
            f'{now - self.start_time:.1f} s since start')

    def odom_callback(self, msg: Vector3):
        # vector_odom packs the total distance travelled into z
        self.total_distance = msg.z
        if self.lap_start_distance is None:
            self.lap_start_distance = msg.z

    def reset_callback(self, msg: Empty):
        del msg  # the message itself carries nothing, arriving at all is the command
        laps, turns = self.lap_count, self.turn_count

        self.turn_count = 0
        self.net_turns = 0
        self.lap_count = 0

        # last_raw_yaw and cumulative_yaw are left alone - they are only the
        # heading tracking, and clearing them would drop the next step. Marking
        # the corner here instead throws away the part turn we were in the
        # middle of, so the first corner after a reset is a whole one.
        self.turn_mark = self.cumulative_yaw
        self.last_turn_time = None

        now = self.now_sec()
        self.start_time = now
        self.lap_start_time = now
        # total_distance is the odometry's own running total and keeps counting,
        # only the mark this lap is measured from moves
        self.lap_start_distance = self.total_distance

        self.lap_pub.publish(Int32(data=self.lap_count))
        self.turn_pub.publish(Int32(data=self.turn_count))
        self.get_logger().info(
            f'Reset - {laps} laps and {turns} turns cleared, counting from zero')


def main(args=None):
    rclpy.init(args=args)
    lap_counter_node = LapCounterNode()
    try:
        rclpy.spin(lap_counter_node)
    except KeyboardInterrupt:
        pass
    finally:
        lap_counter_node.get_logger().info(
            f'Stopping after {lap_counter_node.lap_count} laps '
            f'and {lap_counter_node.turn_count} turns')
        lap_counter_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
