"""
sim_mcu - an ESP32-S3 running firmware/src/main.cpp, minus the ESP32.

The real controls/mcu_bridge runs in the simulator unmodified. It opens a serial port
and speaks MAVLink, so this node makes it a serial port (a pty) and answers the way
the firmware does: the launch boost, throttle scaling and steering clamps in mcu_bridge
are therefore the real code, not a copy that can drift.

    mcu_bridge <-- MAVLink over a pty --> sim_mcu <-- ros_gz_bridge --> Ignition

What each firmware piece became:

    MotorController    wire throttle -> duty = |throttle| / 255, so even a full-scale
                       int8 command is under 50% duty. Zero brakes. No watchdog: like
                       the firmware, the last throttle stays latched until replaced.
    SteeringController servoCorrection = 23: the horn sits at servo 67 for wire 90, so
                       travel is -67 .. +90 deg about centre and the two directions
                       really are unequal. Then MG996R slew, and the linkage.
    EncoderReader      Ticks from the axle angle. rpm from the same 50 ms sampling and
                       ENCODER_COUNTS_PER_REV = 400, the car's ticks per wheel turn.
    Heading            BNO055 Euler x: 0..360 degrees, clockwise, zero at power-on.
    Sonars             All disabled in config.h, so they read 255, "no echo".
    Button             Latched into exactly one 50 ms telemetry frame, like a tap.
    StatusLed          Same state machine: set() from the connect frame, and the start
                       countdown blink (START_BLINK_COUNT x START_BLINK_INTERVAL_MS) on
                       a button press. Shown as a light above the car in RViz
                       (/gorusim/status_led_marker) and as a Bool on /gorusim/status_led.

Press the button with:

    ros2 service call /gorusim/press_start std_srvs/srv/Trigger

Drive the status LED the way any node on the car would, through mcu_bridge:

    ros2 topic pub --once /status_led std_msgs/msg/Bool '{data: false}'
    ros2 topic pub --once /status_led/blink std_msgs/msg/Float32 '{data: 2.0}'
"""
import fcntl
import math
import os
import pty
import termios
import tty

import rclpy
import yaml
from controls import mcu_to_ros2, ros2_to_mcu
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from gorusim.linkage import LinkageGeometry, SteeringLinkage, interpolate

# firmware/include/config.h
MOTOR_MAX_DUTY = 255                # MOTOR_PWM_RESOLUTION_BITS = 8
STEERING_CENTER_ANGLE = 90
SERVO_CORRECTION_DEG = 23           # firmware/lib/steering_control/steering_control.h
ENCODER_COUNTS_PER_REV = 400
ENCODER_SAMPLE_INTERVAL_S = 0.050
SENSOR_TX_INTERVAL_S = 0.050
SONAR_NO_READING_CM = 255
START_BLINK_COUNT = 3
START_BLINK_INTERVAL_S = 1.000      # START_BLINK_INTERVAL_MS

# firmware/src/main.cpp
MCU_SYSTEM_ID = 1
MCU_COMPONENT_ID = 200
WIRE_DIRECTION = {0: 0, 1: 1, -1: 2}

# steering_control.cpp writes (wire - correction); ESP32Servo clamps that to 0..180.
HORN_CENTRE_SERVO_DEG = STEERING_CENTER_ANGLE - SERVO_CORRECTION_DEG

REAR_AXLE_JOINT = 'rear_axle_joint'
AXLE_STOPPED_RAD_S = 0.05

# The LED in RViz: a bead above the lidar, where it cannot hide behind the CAD.
STATUS_LED_MARKER_XYZ = (-0.06, 0.0, 0.16)
STATUS_LED_MARKER_DIAMETER_M = 0.03
STATUS_LED_LIT_RGBA = (0.1, 1.0, 0.1, 1.0)
STATUS_LED_DARK_RGBA = (0.15, 0.15, 0.15, 0.6)
STATUS_LED_STEP_S = 0.005           # loop() rate is far above this; fine enough to see
STATUS_LED_MARKER_PERIOD_S = 0.1    # resent so an RViz opened later still shows it


def clamp(value, low, high):
    return max(low, min(high, value))


def duty_from_throttle(throttle):
    """Signed duty 0..1 for a wire throttle. MotorController::to(), 8 bit PWM."""
    return clamp(throttle, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY) / float(MOTOR_MAX_DUTY)


def duty_after_stiction(duty, axle_rate, breakaway_duty):
    """A stopped gearbox ignores a command too weak to break it free.

    Once rolling, only rolling friction applies, which the axle joint carries.
    """
    if abs(axle_rate) < AXLE_STOPPED_RAD_S and abs(duty) < breakaway_duty:
        return 0.0
    return duty


def horn_from_wire(steering):
    """Horn angle in degrees about centre for a wire steering value.

    SteeringController::to() clamps to 0..180, then writes (angle - 23), and the servo
    library clamps that to 0..180 as well. So the low end runs out at wire 23.
    """
    wire = clamp(steering, 0, 180)
    return clamp(wire - SERVO_CORRECTION_DEG, 0, 180) - HORN_CENTRE_SERVO_DEG


def heading_from_yaw(yaw_rad, yaw_zero_rad):
    """BNO055 Euler x: degrees, clockwise, 0..360, relative to attitude at power-on."""
    heading = (-math.degrees(yaw_rad - yaw_zero_rad)) % 360.0
    # A tiny negative angle wraps to exactly 360.0 in floating point.
    return 0.0 if heading >= 360.0 else heading


def quaternion_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class StatusLedModel:
    """
    Port of firmware/lib/status_led StatusLed, with the time passed in, not millis().

    Kept line for line with status_led.cpp, including its two quirks: set() during a
    blink only records where to land afterwards, and a blink restarted mid sequence
    keeps the level from before the first one.
    """

    def __init__(self):
        self.state = False          # begin(): off until the bridge says hello
        self.pulses_left = 0
        self.period = 0.0
        self.phase_start = 0.0
        self.restore_state = False

    def is_blinking(self):
        return self.pulses_left > 0

    def set(self, lit):
        if self.is_blinking():
            self.restore_state = bool(lit)
            return
        self.state = bool(lit)

    def blink(self, times, period, now):
        if times <= 0 or period <= 0.0:
            return
        if not self.is_blinking():
            self.restore_state = self.state
        self.pulses_left = int(times)
        self.period = float(period)
        self.phase_start = now
        self.state = True           # every cycle starts lit

    def update(self, now):
        if not self.is_blinking():
            return
        elapsed = now - self.phase_start
        if self.state:
            if elapsed >= self.period / 2.0:
                self.state = False
            return
        if elapsed < self.period:
            return
        self.pulses_left -= 1
        if self.pulses_left == 0:
            self.state = self.restore_state
            return
        self.phase_start += self.period
        self.state = True


class PtySerial:
    """The car's USB CDC port: raw bytes, no echo, survives the far end closing."""

    def __init__(self, link_path):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        attrs = termios.tcgetattr(self.slave)
        attrs[3] &= ~termios.ECHO
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)
        flags = fcntl.fcntl(self.master, fcntl.F_GETFL)
        fcntl.fcntl(self.master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self.link_path = link_path
        if os.path.islink(link_path) or os.path.exists(link_path):
            os.remove(link_path)
        os.symlink(os.ttyname(self.slave), link_path)

    def write(self, data):
        try:
            os.write(self.master, bytes(data))
        except BlockingIOError:
            pass  # nobody is reading; a real UART would drop this as well

    def read(self):
        try:
            return os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            return b''

    def close(self):
        if os.path.islink(self.link_path):
            os.remove(self.link_path)
        os.close(self.slave)
        os.close(self.master)


class SimMcu(Node):
    def __init__(self):
        super().__init__('sim_mcu')

        self.declare_parameter('port_link', '/tmp/gorusim_esp32')
        self.declare_parameter('cad_geometry_path', '')
        # See config/sim_hardware.yaml for where each of these comes from.
        self.declare_parameter('encoder_counts_per_wheel_rev', 371.93)
        self.declare_parameter('motor_no_load_speed_rad_s', 98.3)
        self.declare_parameter('motor_breakaway_duty', 0.15)
        self.declare_parameter('servo_slew_deg_per_s', 353.0)
        self.declare_parameter('inner_stop_deg', 39.0)
        self.declare_parameter('outer_stop_deg', 31.0)
        self.declare_parameter('auto_start_sec', 0.0)

        def param(name):
            return self.get_parameter(name).value

        self.counts_per_rev = float(param('encoder_counts_per_wheel_rev'))
        self.no_load_speed = float(param('motor_no_load_speed_rad_s'))
        self.breakaway_duty = float(param('motor_breakaway_duty'))
        self.servo_slew = float(param('servo_slew_deg_per_s'))
        self.auto_start_sec = float(param('auto_start_sec'))

        linkage = SteeringLinkage(self.load_geometry(
            param('cad_geometry_path'), param('inner_stop_deg'), param('outer_stop_deg')))
        # Full horn travel, solved once: this runs at 200 Hz.
        self.steer_table = linkage.table(horn_from_wire(0), horn_from_wire(180), 0.5)

        # ---- state, as main.cpp holds it ----
        self.throttle = 0
        self.steering_wire = STEERING_CENTER_ANGLE
        self.horn_deg = 0.0
        self.last_step_time = None
        self.connected = False

        self.axle_angle = None
        self.axle_rate = 0.0
        self.latest_joints = None
        self.ticks_prev = 0
        self.rpm = 0.0
        self.rpm_sample_time = None

        self.yaw_zero = None
        self.heading_deg = 0.0
        self.button_latch = False

        self.status_led = StatusLedModel()
        self.status_led_published = None

        # ---- the serial port ----
        self.serial = PtySerial(str(param('port_link')))
        self.rx = ros2_to_mcu.MAVLink(self.serial, srcSystem=MCU_SYSTEM_ID,
                                      srcComponent=MCU_COMPONENT_ID)
        self.rx.robust_parsing = True
        self.tx = mcu_to_ros2.MAVLink(self.serial, srcSystem=MCU_SYSTEM_ID,
                                      srcComponent=MCU_COMPONENT_ID)

        # ---- Ignition side ----
        self.motor_pub = self.create_publisher(Float64, '/gorubot/motor_target', 10)
        self.steer_left_pub = self.create_publisher(Float64, '/gorubot/steer_left', 10)
        self.steer_right_pub = self.create_publisher(Float64, '/gorubot/steer_right', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(JointState, '/gorubot/joint_states', self.on_joints, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.create_service(Trigger, '/gorusim/press_start', self.press_start)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_led_pub = self.create_publisher(Bool, '/gorusim/status_led', latched)
        self.status_led_marker_pub = self.create_publisher(
            Marker, '/gorusim/status_led_marker', 10)

        # The serial link is real time: mcu_bridge's writes arrive whenever they do.
        # Everything else is simulated hardware, so it runs on the physics clock.
        self.create_timer(0.005, self.drain_serial)
        self.create_timer(0.005, self.step_actuators)
        self.create_timer(ENCODER_SAMPLE_INTERVAL_S, self.sample_encoder)
        self.create_timer(SENSOR_TX_INTERVAL_S, self.send_telemetry)
        self.create_timer(0.02, self.publish_joints)
        self.create_timer(STATUS_LED_STEP_S, self.step_status_led)
        self.create_timer(STATUS_LED_MARKER_PERIOD_S, self.publish_status_led_marker)
        if self.auto_start_sec > 0.0:
            self.auto_start_timer = self.create_timer(self.auto_start_sec, self.auto_start)

        low, high = self.steer_table[0], self.steer_table[-1]
        self.get_logger().info(
            f'Virtual ESP32-S3 on {self.serial.link_path} -> {os.ttyname(self.serial.slave)}. '
            f'{self.counts_per_rev:.1f} ticks per wheel rev. Wheels at full travel: '
            f'left turn {math.degrees(high[1]):.1f}/{math.degrees(high[2]):.1f} deg, '
            f'right turn {math.degrees(low[2]):.1f}/{math.degrees(low[1]):.1f} deg (inner/outer).')

    def load_geometry(self, path, inner_stop, outer_stop):
        """LinkageGeometry from the extractor's yaml. Field names match by design."""
        values = {}
        if path:
            with open(path, 'r') as handle:
                data = yaml.safe_load(handle)
            for name in LinkageGeometry.__dataclass_fields__:
                if name in data:
                    values[name] = float(data[name])
        values['inner_stop_deg'] = float(inner_stop)
        values['outer_stop_deg'] = float(outer_stop)
        return LinkageGeometry(**values)

    # ---- bytes from mcu_bridge -> what loop() does with them -----------------

    def drain_serial(self):
        data = self.serial.read()
        if not data:
            return
        try:
            messages = self.rx.parse_buffer(data) or []
        except Exception as error:  # noqa: B902 - a corrupt frame must not kill the MCU
            self.get_logger().warn(f'Bad MAVLink from mcu_bridge: {error}',
                                   throttle_duration_sec=2.0)
            return
        for message in messages:
            kind = message.get_type()
            if kind == 'GORUR_GARI_ROS2_TO_MCU_MSG':
                self.throttle = int(message.throttle)
                self.steering_wire = int(message.steering)
            elif kind == 'GORUR_GARI_ROS2_TO_MCU_CONNECT_MSG':
                connected = bool(message.connected)
                # the firmware prints every one, but /status_led/blink sends this
                # frame several times a second; only the first hello gets a line
                if not self.connected:
                    self.get_logger().info('ROS2 bridge connected.')
                self.connected = True
                self.status_led.set(connected)

    # ---- motor and servo -----------------------------------------------------

    def step_actuators(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_step_time is None else max(now - self.last_step_time, 0.0)
        self.last_step_time = now

        target = horn_from_wire(self.steering_wire)
        reach = self.servo_slew * dt
        self.horn_deg += clamp(target - self.horn_deg, -reach, reach)
        left, right = interpolate(self.steer_table, math.radians(self.horn_deg))

        duty = duty_after_stiction(duty_from_throttle(self.throttle), self.axle_rate,
                                   self.breakaway_duty)
        self.motor_pub.publish(Float64(data=duty * self.no_load_speed))
        self.steer_left_pub.publish(Float64(data=float(left)))
        self.steer_right_pub.publish(Float64(data=float(right)))

    # ---- encoder -------------------------------------------------------------

    def on_joints(self, msg):
        self.latest_joints = msg
        if REAR_AXLE_JOINT in msg.name:
            index = list(msg.name).index(REAR_AXLE_JOINT)
            self.axle_angle = msg.position[index]
            if index < len(msg.velocity):
                self.axle_rate = msg.velocity[index]

    def ticks(self):
        if self.axle_angle is None:
            return 0
        return int(round(self.axle_angle / (2.0 * math.pi) * self.counts_per_rev))

    def sample_encoder(self):
        """EncoderReader::update(): rpm is delta ticks over ENCODER_COUNTS_PER_REV."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.rpm_sample_time is not None:
            elapsed = now - self.rpm_sample_time
            if elapsed > 0.0:
                current = self.ticks()
                self.rpm = (current - self.ticks_prev) / ENCODER_COUNTS_PER_REV * 60.0 / elapsed
                self.ticks_prev = current
        self.rpm_sample_time = now

    # ---- heading -------------------------------------------------------------

    def on_imu(self, msg):
        q = msg.orientation
        yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
        if self.yaw_zero is None:
            self.yaw_zero = yaw
        self.heading_deg = heading_from_yaw(yaw, self.yaw_zero)

    # ---- telemetry -----------------------------------------------------------

    def send_telemetry(self):
        if self.rpm > 0.0:
            direction = 1
        elif self.rpm < 0.0:
            direction = -1
        else:
            direction = 0
        button = 1 if self.button_latch else 0
        self.button_latch = False
        self.tx.gorur_gari_mcu_to_ros2_msg_send(
            encoder_count=self.ticks(),
            encoder_speed=int(clamp(abs(self.rpm), 0, 255)),
            encoder_direction=WIRE_DIRECTION[direction],
            servo=int(clamp(self.steering_wire, 0, 180)),
            heading=float(self.heading_deg),
            sonar_1=SONAR_NO_READING_CM,
            sonar_2=SONAR_NO_READING_CM,
            sonar_3=SONAR_NO_READING_CM,
            sonar_4=SONAR_NO_READING_CM,
            button=button)

    # ---- status LED --------------------------------------------------------

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def step_status_led(self):
        """status_led.update() from loop(); a change goes out on /gorusim/status_led."""
        self.status_led.update(self.now_s())
        if self.status_led.state != self.status_led_published:
            self.status_led_published = self.status_led.state
            self.status_led_pub.publish(Bool(data=self.status_led.state))
            self.publish_status_led_marker()

    def publish_status_led_marker(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'status_led'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = \
            STATUS_LED_MARKER_XYZ
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = STATUS_LED_MARKER_DIAMETER_M
        rgba = STATUS_LED_LIT_RGBA if self.status_led.state else STATUS_LED_DARK_RGBA
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        self.status_led_marker_pub.publish(marker)

    def publish_joints(self):
        # Ignition publishes joint state every physics step. Pass robot_state_publisher
        # 50 Hz rather than flooding TF with a thousand identical transforms a second.
        if self.latest_joints is not None:
            self.joint_pub.publish(self.latest_joints)

    # ---- start button --------------------------------------------------------

    def press_start(self, request, response):
        self.button_latch = True
        self.status_led.blink(START_BLINK_COUNT, START_BLINK_INTERVAL_S, self.now_s())
        self.get_logger().info('Start button pressed.')
        response.success = True
        response.message = 'start button pressed'
        return response

    def auto_start(self):
        self.auto_start_timer.cancel()
        self.button_latch = True
        self.status_led.blink(START_BLINK_COUNT, START_BLINK_INTERVAL_S, self.now_s())
        self.get_logger().info(f'Start button auto-pressed after {self.auto_start_sec:.0f} s.')

    def destroy_node(self):
        self.serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimMcu()
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
