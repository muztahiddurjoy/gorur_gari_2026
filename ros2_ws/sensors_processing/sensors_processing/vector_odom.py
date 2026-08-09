"""
Vector odometry: encoder distance + IMU heading -> a single position vector.

The same dead reckoning encoder_odometry.py does, but published as a plain
Vector3 on /odom_vector (x, y, total distance travelled) for the nodes that
only want a position, plus the odom -> base_link transform for RViz.

Units: every internal calculation is in METRES, because that is what TF and
the rest of ROS require (REP-103). Only the Vector3 is scaled, by
output_units, so /odom_vector can read in centimetres while the transform
stays correct.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Vector3, TransformStamped
from tf2_ros import TransformBroadcaster

# how many output units one metre is, for the Vector3 only
OUTPUT_UNIT_SCALE = {'m': 1.0, 'cm': 100.0, 'mm': 1000.0}

# The encoder tick counter is the MCU's raw signed 32 bit accumulator and
# restarts at 0 whenever the MCU reboots. A jump larger than this many ticks
# in one message is treated as a counter restart rather than real motion.
COUNT_JUMP_REJECT_TICKS = 100000


def normalize_angle(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class VectorOdometryNode(Node):
    def __init__(self):
        super().__init__('vector_odometry')

        # -- rig geometry: MEASURE THESE, the defaults are only a starting point --
        # Metres, NOT millimetres. Putting 65 here makes every distance the
        # node reports 1000x too large.
        self.declare_parameter('wheel_diameter_m', 0.065)
        # Ticks per WHEEL revolution, after the gearbox, so no separate gear
        # ratio is needed. Measured over two straight line drives (549 ticks
        # over 31 cm, 641 ticks over 36 cm), which both give ~363 = 11 ppr at
        # one edge through the 33:1 gearbox. Note this disagrees with
        # ENCODER_COUNTS_PER_REV = 1320 in firmware/include/config.h, which
        # assumes x4 decoding - that constant does not touch the tick stream,
        # only the rpm the MCU reports, so rpm telemetry is ~3.6x too low.
        self.declare_parameter('encoder_counts_per_rev', 363)
        # Whatever error is left once the geometry above is right. Drive a
        # measured straight line and set this to (tape measure / reported).
        # See the calibration note in the startup log.
        self.declare_parameter('distance_scale', 0.976)

        # Unit of the /odom_vector message only. The TF is always metres.
        self.declare_parameter('output_units', 'cm')
        # Log rate limit in seconds. 0 logs every encoder message, which is the
        # readable option while calibrating. Publishing is never throttled.
        self.declare_parameter('log_period_sec', 0.0)

        # The BNO055 reports a compass style heading that grows clockwise, while
        # ROS wants yaw growing counter clockwise (REP-103).
        self.declare_parameter('heading_clockwise', True)
        # The BNO055 is mounted at the rear of the car facing BACKWARD, so it
        # reports the direction the tail points. Added to the raw heading to
        # get the direction the car actually drives. Every /heading consumer
        # that turns it into a yaw must use the same value.
        self.declare_parameter('heading_offset_deg', 180.0)
        # Keep False so the yaw always matches the MCU's heading (the OLED
        # value, just sign flipped into REP-103), and restarting this node
        # does not move the odom frame. True re-zeroes yaw at node start.
        self.declare_parameter('zero_heading_on_start', False)

        wheel_diameter = self.get_parameter('wheel_diameter_m').value
        counts_per_rev = self.get_parameter('encoder_counts_per_rev').value
        distance_scale = self.get_parameter('distance_scale').value
        self.output_units = self.get_parameter('output_units').value
        self.log_period_sec = self.get_parameter('log_period_sec').value
        self.heading_clockwise = self.get_parameter('heading_clockwise').value
        self.heading_offset_deg = float(self.get_parameter('heading_offset_deg').value)
        self.zero_heading_on_start = self.get_parameter('zero_heading_on_start').value

        if counts_per_rev <= 0:
            raise ValueError('encoder_counts_per_rev must be positive')
        if self.output_units not in OUTPUT_UNIT_SCALE:
            raise ValueError(f'output_units must be one of {sorted(OUTPUT_UNIT_SCALE)}')
        self.output_scale = OUTPUT_UNIT_SCALE[self.output_units]
        self.metres_per_tick = distance_scale * (math.pi * wheel_diameter) / counts_per_rev

        # integrated pose, metres and radians
        self.global_x = 0.0
        self.global_y = 0.0
        self.current_yaw = 0.0
        self.total_distance = 0.0

        # None until the first encoder message, so the MCU's count at startup
        # becomes the baseline instead of the first step.
        self.last_tick = None
        self.total_ticks = 0

        self.heading_offset = None
        self.raw_yaw = None
        self.mcu_heading_deg = None    # raw MCU heading, exactly what the OLED shows
        self.warned_no_heading = False

        self.pos = Vector3()

        self.odom_pub = self.create_publisher(Vector3, 'odom_vector', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tick_sub = self.create_subscription(Int32, 'encoder/count', self.tick_callback, 10)
        self.heading_sub = self.create_subscription(Float32, 'heading', self.heading_callback, 10)

        self.get_logger().info(
            f'Vector odometry: {self.metres_per_tick * 1000.0:.4f} mm/tick '
            f'(wheel {wheel_diameter * 100.0:.2f} cm, {counts_per_rev} counts/rev, '
            f'scale {distance_scale:g}), publishing /odom_vector in {self.output_units}')
        self.get_logger().info(
            'To calibrate: drive a straight measured line, then set '
            'encoder_counts_per_rev = ticks * pi * wheel_diameter_m / distance_m '
            'using the tick total in the log below.')

    def tick_callback(self, msg: Int32):
        count = msg.data

        # First sample only establishes a baseline. Without this the MCU's
        # whole count since boot lands in the first delta and the pose jumps.
        if self.last_tick is None:
            self.last_tick = count
            return

        del_tick = count - self.last_tick
        self.last_tick = count
        if abs(del_tick) > COUNT_JUMP_REJECT_TICKS:
            self.get_logger().warn(
                f'Encoder count jumped by {del_tick}, treating as a restart')
            return

        del_distance = del_tick * self.metres_per_tick
        self.total_ticks += del_tick
        self.total_distance += del_distance

        previous_yaw = self.current_yaw
        if self.raw_yaw is not None:
            self.current_yaw = self.raw_yaw
        elif not self.warned_no_heading:
            self.warned_no_heading = True
            self.get_logger().warn(
                'No /heading yet - odometry will be a straight line until the IMU reports')

        # Drive along the heading held over this step, which is a closer
        # approximation of the arc than using either endpoint alone.
        mid_yaw = previous_yaw + normalize_angle(self.current_yaw - previous_yaw) / 2.0
        self.global_x += del_distance * math.cos(mid_yaw)
        self.global_y += del_distance * math.sin(mid_yaw)

        self.pos.x = self.global_x * self.output_scale
        self.pos.y = self.global_y * self.output_scale
        self.pos.z = self.total_distance * self.output_scale
        self.odom_pub.publish(self.pos)

        # ticks and heading are the MCU's raw values (same as its OLED), so
        # they survive a node restart; dist/x/y are this node's integration.
        heading_str = (f'{self.mcu_heading_deg:.1f}'
                       if self.mcu_heading_deg is not None else 'n/a')
        line = (f'mcu ticks={count} heading={heading_str} deg | '
                f'x={self.pos.x:.2f} y={self.pos.y:.2f} '
                f'dist={self.pos.z:.2f} {self.output_units} '
                f'yaw={math.degrees(self.current_yaw):.1f} deg')
        if self.log_period_sec > 0.0:
            self.get_logger().info(line, throttle_duration_sec=self.log_period_sec)
        else:
            self.get_logger().info(line)

        # TF stays in metres whatever /odom_vector is scaled to
        self.publish_transform(self.global_x, self.global_y, 0.0, self.current_yaw)

    def heading_callback(self, msg: Float32):
        """Cache the IMU heading, converted into a ROS yaw in radians."""
        self.mcu_heading_deg = msg.data
        # offset undoes the backward IMU mounting; the log keeps the raw value
        yaw = math.radians(msg.data + self.heading_offset_deg)
        if self.heading_clockwise:
            yaw = -yaw
        if self.heading_offset is None:
            self.heading_offset = yaw if self.zero_heading_on_start else 0.0
        self.raw_yaw = normalize_angle(yaw - self.heading_offset)

    def publish_transform(self, x, y, z, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    vector_odom_node = VectorOdometryNode()
    try:
        rclpy.spin(vector_odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        vector_odom_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
