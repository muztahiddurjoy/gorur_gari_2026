"""
Dead reckoning odometry from the MCU's wheel encoder.

Distance travelled comes from the encoder, orientation comes from the IMU
heading that mcu_bridge already publishes. Integrating those gives a pose on
/odom plus the matching odom -> base_link transform, which is what RViz needs
to draw the robot moving around.
"""
import math

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from tf2_ros import TransformBroadcaster

# The encoder tick counter is the MCU's raw signed 32 bit accumulator and
# restarts at 0 whenever the MCU reboots. A jump larger than this many ticks
# in one message is treated as a counter restart rather than real motion.
COUNT_JUMP_REJECT_TICKS = 100000


def yaw_to_quaternion(yaw):
    """Rotation about Z only, so the quaternion collapses to two terms."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def normalize_angle(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class EncoderOdometryNode(Node):

    def __init__(self):
        super().__init__('encoder_odometry')

        # -- rig geometry: MEASURE THESE, the defaults are only a starting point --
        self.declare_parameter('wheel_diameter_m', 0.065)
        # matches ENCODER_COUNTS_PER_REV in firmware/include/config.h
        self.declare_parameter('encoder_counts_per_rev', 1320)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        # The BNO055 reports a compass style heading that grows clockwise, while
        # ROS wants yaw growing counter clockwise (REP-103). Flip this if the
        # robot turns the wrong way in RViz.
        self.declare_parameter('heading_clockwise', True)
        # Added to the raw heading to get the direction the car actually
        # drives. The BNO055 is mounted at the rear facing BACKWARD, but that
        # does NOT make this 180: its Euler yaw is referenced to the sensor's
        # own attitude at power-on, so a constant mounting rotation about z
        # cancels. Must match vector_odom and goto_controller.
        self.declare_parameter('heading_offset_deg', 0.0)
        # Keep False so the yaw always matches the MCU's heading (the OLED
        # value, just sign flipped into REP-103), and restarting this node
        # does not move the odom frame. True re-zeroes yaw at node start.
        self.declare_parameter('zero_heading_on_start', False)

        self.wheel_diameter = self.get_parameter('wheel_diameter_m').value
        self.counts_per_rev = self.get_parameter('encoder_counts_per_rev').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.heading_clockwise = self.get_parameter('heading_clockwise').value
        self.heading_offset_deg = float(self.get_parameter('heading_offset_deg').value)
        self.zero_heading_on_start = self.get_parameter('zero_heading_on_start').value

        if self.counts_per_rev <= 0:
            raise ValueError('encoder_counts_per_rev must be positive')
        self.metres_per_tick = (math.pi * self.wheel_diameter) / self.counts_per_rev

        # integrated pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_count = None
        self.last_stamp = None
        self.heading_offset = None
        self.raw_yaw = None
        self.warned_no_heading = False

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Float32, 'heading', self.handle_heading, 10)
        # odometry is driven off the encoder, so this callback does the work
        self.create_subscription(Int32, 'encoder/count', self.handle_encoder_count, 10)

        self.get_logger().info(
            f'Odometry from encoder: {self.metres_per_tick * 1000:.4f} mm/tick '
            f'(wheel {self.wheel_diameter * 100:.1f} cm, {self.counts_per_rev} counts/rev)')

    def handle_heading(self, msg):
        """Cache the IMU heading, converted into a ROS yaw in radians."""
        # offset undoes the backward IMU mounting
        yaw = math.radians(msg.data + self.heading_offset_deg)
        if self.heading_clockwise:
            yaw = -yaw
        if self.heading_offset is None:
            self.heading_offset = yaw if self.zero_heading_on_start else 0.0
        self.raw_yaw = normalize_angle(yaw - self.heading_offset)

    def handle_encoder_count(self, msg):
        now = self.get_clock().now()
        count = msg.data

        # First sample only establishes a baseline, there is no delta yet.
        if self.last_count is None:
            self.last_count = count
            self.last_stamp = now
            return

        delta_ticks = count - self.last_count
        dt = (now - self.last_stamp).nanoseconds / 1e9
        self.last_count = count
        self.last_stamp = now

        if abs(delta_ticks) > COUNT_JUMP_REJECT_TICKS:
            self.get_logger().warn(
                f'Encoder count jumped by {delta_ticks}, treating as a restart')
            return
        if dt <= 0.0:
            return

        distance = delta_ticks * self.metres_per_tick

        previous_yaw = self.yaw
        if self.raw_yaw is not None:
            self.yaw = self.raw_yaw
        elif not self.warned_no_heading:
            self.warned_no_heading = True
            self.get_logger().warn(
                'No /heading yet - odometry will be a straight line until the IMU reports')

        # Drive along the heading held over this step, which is a closer
        # approximation of the arc than using either endpoint alone.
        mid_yaw = previous_yaw + normalize_angle(self.yaw - previous_yaw) / 2.0
        self.x += distance * math.cos(mid_yaw)
        self.y += distance * math.sin(mid_yaw)

        self.publish_odometry(now, distance / dt, normalize_angle(self.yaw - previous_yaw) / dt)

    def publish_odometry(self, stamp, linear_x, angular_z):
        stamp_msg = stamp.to_msg()
        orientation = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z

        # Dead reckoning drifts without bound, so advertise loose but finite
        # confidence on the axes we actually estimate and lock the rest.
        unused = 1e6
        odom.pose.covariance[0] = 0.05    # x
        odom.pose.covariance[7] = 0.05    # y
        odom.pose.covariance[14] = unused  # z
        odom.pose.covariance[21] = unused  # roll
        odom.pose.covariance[28] = unused  # pitch
        odom.pose.covariance[35] = 0.05   # yaw
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = unused
        odom.twist.covariance[14] = unused
        odom.twist.covariance[21] = unused
        odom.twist.covariance[28] = unused
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp_msg
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
