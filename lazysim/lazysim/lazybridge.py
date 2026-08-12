"""
lazybridge — the simulator's stand-in for mcu_bridge.

On the real car, controls/mcu_bridge.py is the only thing that talks to
the ESP32: it takes /cmd_vel in and puts encoder, heading, servo and
button telemetry out over MAVLink. It cannot run in simulation (there is
no /dev/esp32_s3), so this node takes its place and presents the exact
same ROS interface. Everything downstream — vector_odom, lap_counter,
open_round_run, disparity_extender — runs against the sim completely
unchanged, which is the whole point of the exercise.

    ROS in                                ROS out (identical to mcu_bridge)
    ──────────────────────────────        ────────────────────────────────
    /cmd_vel        Twist                 encoder/count      Int32
    /cam_servo      Int8  (degrees)       encoder/speed      Float32 (rpm)
                                          encoder/direction  Int8  (-1/0/+1)
    Ignition in (via ros_gz_bridge)       steering_angle     UInt8 (degrees)
    ──────────────────────────────        heading            Float32 (degrees)
    /lazybot/joint_states  JointState     /button_status     Bool
    /imu                   Imu            /joint_states      JointState

    Ignition out (via ros_gz_bridge)
    ────────────────────────────────
    /lazybot/steer_left    Float64 (rad)
    /lazybot/steer_right   Float64 (rad)
    /lazybot/drive_left    Float64 (rad/s)
    /lazybot/drive_right   Float64 (rad/s)
    /lazybot/cam_servo     Float64 (rad)

Two deliberate departures from mcu_bridge:

  * No launch boost. mcu_bridge multiplies the command that starts a
    move by launch_boost_gain because the real drivetrain cannot break
    static friction at cruise throttle. The simulated rear axle is
    driven by an exact velocity command, so there is no stiction to
    break, and applying the boost here would just make the sim car
    accelerate twice as hard as the real one.

  * No sonar. All four SONAR_*_ENABLED flags are false in
    firmware/include/config.h and mcu_bridge publishes nothing for a
    sonar that is not plugged in, so neither does this.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32, Float64, Int8, Int32, UInt8
from std_srvs.srv import Trigger

# The rear axle is physically coupled on the real car (WRO rules require
# it), so both wheels always get the same speed and either one is a
# faithful source of encoder ticks.
REAR_LEFT_JOINT = 'base_to_back_left_wheel'
REAR_RIGHT_JOINT = 'base_to_back_right_wheel'

# Below this the car is "stopped" as far as encoder/direction is concerned.
STOPPED_RAD_PER_SEC = 0.05

TWO_PI = 2.0 * math.pi


def clamp(value, low, high):
    return max(low, min(high, value))


def quaternion_yaw(x, y, z, w):
    """Yaw only — roll and pitch are not interesting on a flat mat."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LazyBridge(Node):
    def __init__(self):
        super().__init__('lazybridge')

        # ── Geometry: mirrors ros2_ws/config/bot_config.yaml and the
        #    xacro properties in description/lazyBot.xacro. All three
        #    have to agree or the sim steers differently from the model
        #    it is drawing. ────────────────────────────────────────────
        self.declare_parameter('wheelbase_m', 0.140)
        self.declare_parameter('track_m', 0.106)
        self.declare_parameter('wheel_radius_m', 0.025)   # 50 mm diameter
        self.declare_parameter('hinge_inset_m', 0.012)
        self.declare_parameter('max_steer_deg', 60.0)     # bot.max_steer_deg

        # ── /cmd_vel interpretation: the same scaling mcu_bridge applies
        #    on the way to the wire, so a command that would saturate the
        #    real servo or the int8 throttle field saturates here too. ──
        self.declare_parameter('cmd_to_mps', 1.0)         # linear.x of 1.0 -> m/s
        self.declare_parameter('throttle_scale', 100.0)   # mcu_bridge throttle_scale
        self.declare_parameter('throttle_min', -128)
        self.declare_parameter('throttle_max', 127)
        self.declare_parameter('steering_center_deg', 90)
        self.declare_parameter('steering_gain_deg', 60.0)
        self.declare_parameter('steering_min_deg', 0)
        self.declare_parameter('steering_max_deg', 180)

        # ── Encoder ─────────────────────────────────────────────────────
        # Ticks per WHEEL revolution as seen at the tick stream. This is
        # bot.encoder_counts_per_rev, and deliberately NOT the firmware's
        # ENCODER_COUNTS_PER_REV.
        self.declare_parameter('encoder_counts_per_rev', 363)
        # ...which is what the firmware DOES use to compute the rpm it
        # reports (firmware/include/config.h). The mismatch makes real
        # rpm telemetry read about 3.6x low, and reproducing that here
        # keeps the sim honest about what the wire actually carries.
        self.declare_parameter('rpm_counts_per_rev', 1320)

        # ── Heading ─────────────────────────────────────────────────────
        # The BNO055 reports degrees growing CLOCKWISE, referenced to
        # however it was sitting at power-on. Ignition gives a world
        # referenced counter-clockwise quaternion, so the first yaw seen
        # becomes the zero and the sign is flipped.
        self.declare_parameter('heading_clockwise', True)
        self.declare_parameter('zero_heading_on_start', True)

        # ── Start button ────────────────────────────────────────────────
        # 0 disables. Otherwise /button_status goes true this many
        # seconds after startup, so an unattended run needs no poking.
        self.declare_parameter('auto_start_sec', 0.0)

        # ── Safety ──────────────────────────────────────────────────────
        # If whatever was driving dies, coast to a stop rather than
        # carrying on at the last commanded speed forever.
        self.declare_parameter('cmd_timeout_sec', 1.0)

        g = lambda n: self.get_parameter(n).value
        self.wheelbase = float(g('wheelbase_m'))
        self.track = float(g('track_m'))
        self.wheel_radius = float(g('wheel_radius_m'))
        self.hinge_inset = float(g('hinge_inset_m'))
        self.max_steer = math.radians(float(g('max_steer_deg')))
        self.cmd_to_mps = float(g('cmd_to_mps'))
        self.throttle_scale = float(g('throttle_scale'))
        self.throttle_min = int(g('throttle_min'))
        self.throttle_max = int(g('throttle_max'))
        self.steer_center = int(g('steering_center_deg'))
        self.steer_gain = float(g('steering_gain_deg'))
        self.steer_min_deg = int(g('steering_min_deg'))
        self.steer_max_deg = int(g('steering_max_deg'))
        self.counts_per_rev = int(g('encoder_counts_per_rev'))
        self.rpm_counts_per_rev = int(g('rpm_counts_per_rev'))
        self.heading_clockwise = bool(g('heading_clockwise'))
        self.zero_heading_on_start = bool(g('zero_heading_on_start'))
        self.auto_start_sec = float(g('auto_start_sec'))
        self.cmd_timeout_sec = float(g('cmd_timeout_sec'))

        # Distance between the two kingpins, which is what the Ackermann
        # split is actually about — not the distance between tyres.
        self.kingpin_track = self.track - 2.0 * self.hinge_inset

        # ── State ───────────────────────────────────────────────────────
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_time = None
        self.servo_deg = self.steer_center
        self.cam_target_rad = 0.0

        self.wheel_angle_rad = None     # rear axle angle, from /joint_states
        self.wheel_rate_rad_s = 0.0
        self.latest_joint_state = None

        self.yaw_zero = None
        self.heading_deg = 0.0

        # ── Ignition-side command publishers ────────────────────────────
        self.steer_left_pub = self.create_publisher(Float64, '/lazybot/steer_left', 10)
        self.steer_right_pub = self.create_publisher(Float64, '/lazybot/steer_right', 10)
        self.drive_left_pub = self.create_publisher(Float64, '/lazybot/drive_left', 10)
        self.drive_right_pub = self.create_publisher(Float64, '/lazybot/drive_right', 10)
        self.cam_servo_pub = self.create_publisher(Float64, '/lazybot/cam_servo', 10)

        # ── Real-bot telemetry publishers (mcu_bridge's interface) ──────
        self.encoder_count_pub = self.create_publisher(Int32, 'encoder/count', 10)
        self.encoder_speed_pub = self.create_publisher(Float32, 'encoder/speed', 10)
        self.encoder_direction_pub = self.create_publisher(Int8, 'encoder/direction', 10)
        self.steering_angle_pub = self.create_publisher(UInt8, 'steering_angle', 10)
        self.heading_pub = self.create_publisher(Float32, 'heading', 10)
        self.button_pub = self.create_publisher(Bool, '/button_status', 10)
        # Ignition publishes joint state every physics step (1 kHz). Pass
        # it on to robot_state_publisher at a sane 50 Hz instead, so TF
        # is not flooded with a thousand identical transforms a second.
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── Subscriptions ───────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int8, '/cam_servo', self.cam_servo_callback, 1)
        self.create_subscription(JointState, '/lazybot/joint_states',
                                 self.joint_state_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Press the start button by hand:
        #   ros2 service call /lazybot/press_start std_srvs/srv/Trigger
        self.create_service(Trigger, '/lazybot/press_start', self.press_start)

        # 50 Hz for both loops, matching the MCU's telemetry cadence.
        self.create_timer(0.02, self.control_loop)
        self.create_timer(0.02, self.telemetry_loop)

        if self.auto_start_sec > 0.0:
            self.auto_start_timer = self.create_timer(
                self.auto_start_sec, self.auto_start)

        self.get_logger().info(
            f'lazybridge up — wheelbase {self.wheelbase * 1000:.0f} mm, '
            f'track {self.track * 1000:.0f} mm, wheel {self.wheel_radius * 2000:.0f} mm, '
            f'lock ±{math.degrees(self.max_steer):.0f}°, '
            f'{self.counts_per_rev} ticks/rev')
        self.get_logger().info(
            'Start button: ros2 service call /lazybot/press_start std_srvs/srv/Trigger'
            + (f' (auto-pressing in {self.auto_start_sec:.0f}s)'
               if self.auto_start_sec > 0.0 else ''))

    # ══════════════════════════════════════════════════════════════════
    # Commands in → joints out
    # ══════════════════════════════════════════════════════════════════

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def cam_servo_callback(self, msg: Int8):
        self.cam_target_rad = math.radians(float(msg.data))

    def ackermann_split(self, steer_rad):
        """The steering command -> the two kingpin angles.

        Both front wheels have to point at the same instantaneous centre
        of rotation, so the inner one turns further than the outer. This
        is the geometry the real steering linkage approximates, and
        getting it right matters because the LiDAR-derived target angle
        is interpreted as a path curvature, not a wheel angle.

        steer_rad is taken as the angle of the INNER wheel, not of some
        imaginary centreline wheel. That is what makes max_steer_deg mean
        what the README says it means — "a physical lock of about 60
        degrees to each side" is a stop the steering arm actually hits.
        Read the other way round, a full-lock command works out to an
        inner wheel angle of 74 degrees, which no linkage on the car can
        reach; the joint clips it at 60 and the sim quietly drives a
        wider arc than it reports.

        Positive is LEFT for both returned angles.
        """
        if abs(steer_rad) < 1e-3:
            return steer_rad, steer_rad

        half = self.kingpin_track / 2.0
        # Turn radius to the vehicle centreline at the rear axle, derived
        # from the inner wheel rather than assumed.
        radius = self.wheelbase / math.tan(abs(steer_rad)) + half

        inner = math.atan2(self.wheelbase, radius - half)
        outer = math.atan2(self.wheelbase, radius + half)

        if steer_rad > 0.0:            # turning left: left wheel is inner
            return inner, outer
        return -outer, -inner          # turning right: right wheel is inner

    def control_loop(self):
        linear = self.cmd_linear
        angular = self.cmd_angular

        # Watchdog: silence means nobody is driving.
        if self.last_cmd_time is not None and self.cmd_timeout_sec > 0.0:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout_sec:
                linear = 0.0

        # Throttle takes the same trip through the int8 wire field it
        # would on the real car, so a command that clips there clips here.
        throttle_wire = clamp(linear * self.throttle_scale,
                              self.throttle_min, self.throttle_max)
        speed_mps = (throttle_wire / self.throttle_scale) * self.cmd_to_mps
        wheel_rate = speed_mps / self.wheel_radius

        # Steering likewise goes through the servo clamp before it
        # reaches the linkage.
        servo = clamp(self.steer_center + angular * self.steer_gain,
                      self.steer_min_deg, self.steer_max_deg)
        self.servo_deg = int(round(servo))
        steer_rad = clamp(
            ((servo - self.steer_center) / self.steer_gain) * self.max_steer,
            -self.max_steer, self.max_steer)

        left_rad, right_rad = self.ackermann_split(steer_rad)

        self.steer_left_pub.publish(Float64(data=float(left_rad)))
        self.steer_right_pub.publish(Float64(data=float(right_rad)))
        # Solid rear axle — one speed, both wheels.
        self.drive_left_pub.publish(Float64(data=float(wheel_rate)))
        self.drive_right_pub.publish(Float64(data=float(wheel_rate)))
        self.cam_servo_pub.publish(Float64(data=float(self.cam_target_rad)))

    # ══════════════════════════════════════════════════════════════════
    # Simulated sensors → the telemetry the MCU would have sent
    # ══════════════════════════════════════════════════════════════════

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg
        names = list(msg.name)
        angle = rate = None
        for joint in (REAR_LEFT_JOINT, REAR_RIGHT_JOINT):
            if joint not in names:
                continue
            i = names.index(joint)
            if i < len(msg.position):
                angle = msg.position[i]
            if i < len(msg.velocity):
                rate = msg.velocity[i]
            break
        if angle is not None:
            self.wheel_angle_rad = angle
        if rate is not None:
            self.wheel_rate_rad_s = rate

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        yaw = quaternion_yaw(q.x, q.y, q.z, q.w)

        # The BNO055 reads 0 at power-on however it is bolted on, so the
        # first yaw the sim reports becomes this run's zero.
        if self.yaw_zero is None:
            self.yaw_zero = yaw if self.zero_heading_on_start else 0.0

        relative = yaw - self.yaw_zero
        degrees = math.degrees(relative)
        if self.heading_clockwise:
            degrees = -degrees
        # The chip reports 0..360, and both consumers (vector_odom's
        # normalize_angle and lap_counter's wrap180) expect to have to
        # unwrap it, so hand them the same shape the hardware does.
        self.heading_deg = degrees % 360.0
        self.heading_pub.publish(Float32(data=float(self.heading_deg)))

    def telemetry_loop(self):
        # Steering servo angle, straight from the last command.
        self.steering_angle_pub.publish(
            UInt8(data=int(clamp(self.servo_deg, 0, 255))))

        if self.wheel_angle_rad is None:
            return  # no joint feedback yet, so no encoder to report

        # Raw cumulative ticks, exactly what the MCU's int32 accumulator
        # holds. vector_odom baselines itself off the first sample, so
        # starting from zero here is fine.
        ticks = int(round((self.wheel_angle_rad / TWO_PI) * self.counts_per_rev))
        self.encoder_count_pub.publish(Int32(data=ticks))

        # Speed as the firmware computes it: tick rate converted with
        # ENCODER_COUNTS_PER_REV, magnitude only, into a uint8 field.
        ticks_per_sec = abs(self.wheel_rate_rad_s) / TWO_PI * self.counts_per_rev
        rpm = ticks_per_sec * 60.0 / self.rpm_counts_per_rev
        self.encoder_speed_pub.publish(Float32(data=float(clamp(rpm, 0.0, 255.0))))

        if abs(self.wheel_rate_rad_s) < STOPPED_RAD_PER_SEC:
            direction = 0
        elif self.wheel_rate_rad_s > 0.0:
            direction = 1
        else:
            direction = -1
        self.encoder_direction_pub.publish(Int8(data=direction))

        # Rate-limited passthrough for robot_state_publisher.
        if self.latest_joint_state is not None:
            self.joint_state_pub.publish(self.latest_joint_state)

    # ══════════════════════════════════════════════════════════════════
    # Start button
    # ══════════════════════════════════════════════════════════════════

    def press_start(self, request, response):
        self.button_pub.publish(Bool(data=True))
        self.get_logger().info('Start button pressed.')
        response.success = True
        response.message = 'start button pressed'
        return response

    def auto_start(self):
        self.auto_start_timer.cancel()
        self.button_pub.publish(Bool(data=True))
        self.get_logger().info(
            f'Start button auto-pressed after {self.auto_start_sec:.0f}s.')


def main(args=None):
    rclpy.init(args=args)
    node = LazyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
