import math
import os
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from pymavlink.dialects.v20 import common as mavlink_common
import serial
from controls import mcu_to_ros2
from controls import ros2_to_mcu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Int32, Int8, String, UInt8, Float32

# sonar_1..4 on the wire map to these sensors in this order, see firmware/pin-map.md
SONAR_NAMES = ['front', 'left', 'right', 'rear']
SONAR_FRAME_IDS = ['sonar_front_link', 'sonar_left_link', 'sonar_right_link', 'sonar_rear_link']
SONAR_TOPICS = ['sonar/front', 'sonar/left', 'sonar/right', 'sonar/rear']
SONAR_MAX_RANGE_M = 2.55  # wire field is uint8_t cm, capped at 255
SONAR_FIELD_OF_VIEW_RAD = 0.26  # ~15 degrees, typical HC-SR04 beam angle
SONAR_NO_READING_CM = 255  # disabled sonar, or no echo before the timeout

# a sonar that is not plugged in gets no publisher and is never published. keep
# these defaults in step with the SONAR_*_ENABLED flags in firmware/include/config.h
# (the firmware decides what is actually measured, this decides what is exposed).
SONAR_ENABLED_DEFAULTS = {'front': False, 'left': False, 'right': False, 'rear': False}

# encoder_direction on the wire: 0 = stopped, 1 = forward, 2 = reverse (see firmware/src/main.cpp)
WIRE_DIRECTION_TO_SIGN = {0: 0, 1: 1, 2: -1}

# /cmd_vel -> wire command scaling. the firmware applies whatever throttle and
# steering arrive on the wire (firmware/src/main.cpp), so this node is the only
# place the drive limits are enforced. these in-code values are fallbacks - the
# real tuning is the mcu_bridge section of ros2_ws/config/bot_config.yaml,
# which the launch files load.
THROTTLE_SCALE = 100.0    # wire throttle = linear.x * this
THROTTLE_MIN = -128       # int8 wire field floor (full reverse)
THROTTLE_MAX = 127        # int8 wire field ceiling (full forward)
STEERING_CENTER_DEG = 90  # servo angle that rolls straight
STEERING_GAIN_DEG = 60.0  # wire angle = center + angular.z * this
STEERING_MIN_DEG = 0      # servo clamp, full left
STEERING_MAX_DEG = 180    # servo clamp, full right

# The ESP32-S3 is on a USB cable into the devkit's port labelled "USB" (the
# chip's own USB-Serial/JTAG controller, USB VID 303a), not its "UART" port.
# It enumerates as /dev/ttyACM0; the udev rule in ros2_ws/config/99-gorur.rules
# gives it the stable name /dev/gorur_mcu, so it can never be confused with the
# LiDAR's CP2102. /dev/ttyACM0 is only tried when the stable name is missing.
MCU_PORT = '/dev/gorur_mcu'
MCU_FALLBACK_PORT = '/dev/ttyACM0'
MCU_BAUDRATE = 115200  # ignored by USB CDC; = MAV_SERIAL_BAUD in firmware/include/config.h
# When the port goes away (ESP32 reset or re-flashed, cable pulled) it is
# closed and reopened on this period, for as long as it takes.
MCU_RECONNECT_INTERVAL_S = 1.0
# A frame that cannot be written within this long is dropped (the MCU is not
# reading), so a stuck MCU can never stall the node. Not 0: pyserial's
# "non-blocking" write_timeout=0 spins forever when the tty buffer is full.
MCU_WRITE_TIMEOUT_S = 0.01

# Drive command stream. The firmware moves the car only while valid drive
# commands keep arriving: it brakes and centres the steering when none has
# arrived for CMD_TIMEOUT_MS (100 ms, firmware/include/config.h). So instead of
# forwarding /cmd_vel as it comes, this node re-sends the latest command at a
# fixed 50 Hz for as long as /cmd_vel is fresh, which rides through four lost
# frames in a row. Do NOT lower the rate without raising the firmware timeout.
DRIVE_TX_INTERVAL_S = 0.02
# /cmd_vel older than this is stale: the node stops sending drive commands and
# the firmware brakes on its own 100 ms timeout. 0.3 s comfortably covers the
# slowest driving node (10 Hz), so a live node never trips it.
CMD_VEL_STALE_S = 0.3
# mirror of the firmware's CMD_TIMEOUT_MS, for the logs and the tests
FIRMWARE_CMD_TIMEOUT_S = 0.1

# STATUSTEXT (#253, common dialect): how the firmware reports trouble, e.g. the
# IMU missing or faulted. mcu_to_ros2 does not define it, so the parser hands
# it over undecoded (UNKNOWN_253, raw frame attached) and decode_mcu_statustext
# decodes it with pymavlink's common dialect.
STATUSTEXT_MSG_ID = 253
_COMMON_DECODER = mavlink_common.MAVLink(None)

# the drivetrain does not make enough torque at cruise throttle to break static
# friction, so the command that starts a move from a standstill gets scaled up
# for a moment. once the bot is rolling, momentum carries it and the commanded
# velocity goes through untouched.
LAUNCH_BOOST_GAIN = 2.0
LAUNCH_BOOST_DURATION_S = 1.0  # 0.0 -> boost only the single command that starts the move
LAUNCH_IDLE_TIMEOUT_S = 0.5  # no cmd_vel for this long counts as stopped, so the boost re-arms

# how long to wait after (re)opening the port before announcing ourselves to the
# MCU. The ESP32's USB port reappears as soon as the chip resets, well before
# its firmware is running, so a frame sent at once could be lost.
MCU_CONNECT_NOTICE_DELAY_S = 1.0

# run stopwatch -> OLED. the autonomy/run_timer node publishes /run_time and
# /run_timer_state, this node is the only thing holding the serial port so it
# is what puts them on the wire. nothing is sent until run_timer speaks up, so
# a stack launched without it behaves exactly as it did before.
RUN_TIMER_TX_INTERVAL_S = 0.1   # 10 Hz, one frame per displayed tenth
# run_timer publishes at 10 Hz, so this much silence is the node being gone,
# not a dropped frame. the MCU stops being told anything and marks its own
# display stale rather than showing a time that stopped being true.
RUN_TIMER_STALE_S = 1.0
# wire values of the state field in gorur_gari_ros2_to_mcu_timer_msg, see
# mav_msg/ros2_to_mcu.xml and firmware/lib/run_timer/run_timer.h
RUN_TIMER_WIRE_STATE = {'IDLE': 0, 'RUNNING': 1, 'STOPPED': 2}
RUN_TIMER_ELAPSED_MS_MAX = 0xFFFFFFFF  # uint32 wire field ceiling

# status LED from any node. The firmware has no LED message of its own: the only
# frame that touches the LED is the connect notice, and it does
# status_led.set(connected != 0) (firmware/src/main.cpp). So steady on/off is
# that frame with connected = 1/0, and blinking is this node toggling it on a
# timer. The same frame also flips the RGB pixel between "connected" (green)
# and "disconnected" (red), so the pixel blinks along - nothing else changes.
STATUS_LED_TOPIC = '/status_led'              # std_msgs/Bool: steady on/off, stops a blink
STATUS_LED_BLINK_TOPIC = '/status_led/blink'  # std_msgs/Float32: blink rate in Hz, <= 0 stops
# each toggle is one 9 byte frame on a 115200 baud link, so even the cap is
# noise next to the 50 Hz telemetry - the cap is there to keep the blink visible
STATUS_LED_MAX_BLINK_HZ = 10.0


def blink_half_period(rate_hz, max_rate_hz=STATUS_LED_MAX_BLINK_HZ):
    """
    Seconds between LED toggles for a blink rate, or None to stop blinking.

    One blink is a full on+off cycle, so the toggle interval is half the
    period. Rates above max_rate_hz are capped rather than refused, so a
    caller asking for "fast" still gets a blink.
    """
    if math.isnan(rate_hz) or rate_hz <= 0.0:
        return None
    return 0.5 / min(rate_hz, max_rate_hz)


def open_mcu_port(path, baudrate, serial_class=serial.Serial):
    """
    Open the ESP32's USB CDC port without resetting it.

    The ESP32-S3's USB-Serial/JTAG controller wires the CDC modem lines to its
    reset and boot logic: RTS high with DTR low holds the chip in reset, and DTR
    high as it leaves reset selects the ROM bootloader. Linux raises both lines
    together when the tty opens, which is harmless (both high = run). They are
    then dropped RTS FIRST - dropping DTR first would pass through RTS high /
    DTR low, a reset pulse. No hardware flow control (rtscts / dsrdtr), which
    would drive those lines on its own.
    """
    port = serial_class()
    port.port = path
    port.baudrate = baudrate
    port.timeout = 0                         # reads never wait
    port.write_timeout = MCU_WRITE_TIMEOUT_S
    port.dsrdtr = False
    port.rtscts = False
    port.xonxoff = False
    port.open()
    try:
        port.rts = False
        port.dtr = False
    except (OSError, serial.SerialException):
        pass  # no modem lines: a pseudo terminal (gorusim, the tests)
    return port


def drive_command_is_fresh(age_s, stale_s=CMD_VEL_STALE_S):
    """
    Whether a /cmd_vel that arrived age_s seconds ago may still be sent.

    None (nothing received yet) and anything at or past stale_s is stale.
    """
    return age_s is not None and 0.0 <= age_s < stale_s


def decode_mcu_statustext(raw_frame):
    """
    Decode a STATUSTEXT frame from the MCU into (severity, text), or None.

    The frame is checked (CRC, length) against the common dialect, so a frame
    that merely carries id 253 but is not a valid STATUSTEXT returns None.
    """
    try:
        msg = _COMMON_DECODER.decode(bytearray(raw_frame))
    except Exception:
        return None
    if msg is None or msg.get_type() != 'STATUSTEXT':
        return None
    text = msg.text
    if isinstance(text, (bytes, bytearray)):
        text = text.decode('utf-8', errors='replace')
    return int(msg.severity), str(text).rstrip('\x00')


def run_time_to_wire_ms(seconds):
    """
    Convert seconds off /run_time to the uint32 millisecond field on the wire.

    Truncated rather than rounded, so the tenth the firmware prints is the
    same tenth run_timer's own /run_time_str prints. A run time that reads
    differently on the screen and in the logs is a run time nobody trusts.
    """
    if not math.isfinite(seconds) or seconds < 0.0:
        seconds = 0.0
    return min(int(seconds * 1000.0), RUN_TIMER_ELAPSED_MS_MAX)


class MCUBridgeNode(Node):
    def __init__(self):
        super().__init__('mcu_bridge')
        self.declare_parameter('port', MCU_PORT)
        # '' = no fallback (the simulator sets that, so it can never grab a
        # real /dev/ttyACM0)
        self.declare_parameter('fallback_port', MCU_FALLBACK_PORT)
        self.declare_parameter('baudrate', MCU_BAUDRATE)
        self.port = str(self.get_parameter('port').value)
        self.fallback_port = str(self.get_parameter('fallback_port').value or '')
        self.baudrate = int(self.get_parameter('baudrate').value)
        fallback = f', fallback {self.fallback_port}' if self.fallback_port else ''
        self.get_logger().info(f'Connecting to MCU on {self.port}{fallback}.')
        self.mcu_connected = False
        self.ser = None          # the open pyserial port while connected
        self.mav_rx = None       # mcu_to_ros2 parser, one per connection
        self.mav_tx = None       # ros2_to_mcu encoder writing to self.ser
        self.mcu_port_in_use = None
        self.last_connect_attempt = None

        # /cmd_vel scaling and clamps, see the mcu_bridge section of
        # ros2_ws/config/bot_config.yaml
        self.declare_parameter('throttle_scale', THROTTLE_SCALE)
        self.declare_parameter('throttle_min', THROTTLE_MIN)
        self.declare_parameter('throttle_max', THROTTLE_MAX)
        self.declare_parameter('steering_center_deg', STEERING_CENTER_DEG)
        self.declare_parameter('steering_gain_deg', STEERING_GAIN_DEG)
        self.declare_parameter('steering_min_deg', STEERING_MIN_DEG)
        self.declare_parameter('steering_max_deg', STEERING_MAX_DEG)
        self.throttle_scale = float(self.get_parameter('throttle_scale').value)
        self.throttle_min = int(self.get_parameter('throttle_min').value)
        self.throttle_max = int(self.get_parameter('throttle_max').value)
        self.steering_center_deg = int(self.get_parameter('steering_center_deg').value)
        self.steering_gain_deg = float(self.get_parameter('steering_gain_deg').value)
        self.steering_min_deg = int(self.get_parameter('steering_min_deg').value)
        self.steering_max_deg = int(self.get_parameter('steering_max_deg').value)

        # override without editing code, e.g.
        #   ros2 run controls mcu_bridge --ros-args -p sonar_front_enabled:=true
        self.sonar_enabled = []
        for name in SONAR_NAMES:
            param = f'sonar_{name}_enabled'
            self.declare_parameter(param, SONAR_ENABLED_DEFAULTS[name])
            self.sonar_enabled.append(bool(self.get_parameter(param).value))

        self.declare_parameter('launch_boost_gain', LAUNCH_BOOST_GAIN)
        self.declare_parameter('launch_boost_duration', LAUNCH_BOOST_DURATION_S)
        self.declare_parameter('launch_idle_timeout', LAUNCH_IDLE_TIMEOUT_S)
        self.launch_boost_gain = float(self.get_parameter('launch_boost_gain').value)
        self.launch_boost_duration = float(self.get_parameter('launch_boost_duration').value)
        self.launch_idle_timeout = float(self.get_parameter('launch_idle_timeout').value)
        # launch boost state: are we rolling, until when is the kick live, and when
        # did the last cmd_vel land (a gap means the bot coasted to a stop)
        self.is_moving = False
        self.launch_boost_until = None
        self.last_cmd_vel_time = None

        self.sonar_pubs = [
            self.create_publisher(Range, topic, 10) if on else None
            for topic, on in zip(SONAR_TOPICS, self.sonar_enabled)
        ]
        live = [n for n, on in zip(SONAR_NAMES, self.sonar_enabled) if on]
        self.get_logger().info(f'Sonars enabled: {", ".join(live) if live else "none"}')
        self.encoder_count_pub = self.create_publisher(Int32, 'encoder/count', 10)
        self.encoder_speed_pub = self.create_publisher(Float32, 'encoder/speed', 10)
        self.encoder_direction_pub = self.create_publisher(Int8, 'encoder/direction', 10)
        self.steering_angle_pub = self.create_publisher(UInt8, 'steering_angle', 10)
        self.heading_pub = self.create_publisher(Float32, 'heading', 10)
        self.button_pub = self.create_publisher(Bool, '/button_status', 10)

        # run stopwatch, cached here and shipped to the OLED by the tx timer
        # below. None until run_timer publishes for the first time.
        self.run_time_sec = None
        self.run_time_stamp = None
        self.run_timer_state = 'IDLE'

        # status LED, see STATUS_LED_TOPIC. lit by default because the connect
        # notice lights it; steady is where a stopped blink lands.
        self.declare_parameter('status_led_max_blink_hz', STATUS_LED_MAX_BLINK_HZ)
        self.status_led_max_blink_hz = float(self.get_parameter('status_led_max_blink_hz').value)
        self.status_led_steady = True
        self.status_led_lit = True
        self.status_led_blink_timer = None
        # nothing goes on the wire before the connect notice: until then the esp32
        # may still be in its bootloader. the notice carries whatever was asked for.
        self.mcu_announced = False

        # the latest drive command worked out from /cmd_vel, and when that
        # /cmd_vel arrived. send_drive_command re-sends it at 50 Hz while fresh.
        self.drive_cmd = None
        self.drive_cmd_time = None
        self.drive_streaming = False  # for the start/stop log lines only

        # tell the MCU we are here, once per connection, MCU_CONNECT_NOTICE_DELAY_S
        # after the port opens. It lights its status LED on this and nothing
        # else. Armed by connect_mcu(), cancelled again once it has fired.
        self.connect_notice_timer = self.create_timer(
            MCU_CONNECT_NOTICE_DELAY_S, self.send_connect_notice)
        self.connect_notice_timer.cancel()

        self.connect_mcu()
        self.cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.handle_cmd_vel, 10)
        self.run_time_sub = self.create_subscription(
            Float32, '/run_time', self.handle_run_time, 10)
        self.run_timer_state_sub = self.create_subscription(
            String, '/run_timer_state', self.handle_run_timer_state, 10)
        self.status_led_sub = self.create_subscription(
            Bool, STATUS_LED_TOPIC, self.handle_status_led, 10)
        self.status_led_blink_sub = self.create_subscription(
            Float32, STATUS_LED_BLINK_TOPIC, self.handle_status_led_blink, 10)
        # drain incoming sensor telemetry at 50 Hz (and reconnect when the
        # port has gone)
        self.mcu_poll_timer = self.create_timer(0.02, self.poll_mcu)
        # the drive command stream, see DRIVE_TX_INTERVAL_S
        self.drive_tx_timer = self.create_timer(DRIVE_TX_INTERVAL_S, self.send_drive_command)
        # push the run stopwatch out to the OLED at 10 Hz
        self.run_timer_tx_timer = self.create_timer(
            RUN_TIMER_TX_INTERVAL_S, self.send_run_time)

    # ---- the link to the MCU ----

    def candidate_ports(self):
        ports = [self.port]
        if self.fallback_port and self.fallback_port != self.port:
            ports.append(self.fallback_port)
        return ports

    def connect_mcu(self):
        """Open the first port that exists; on success arm the connect notice."""
        self.last_connect_attempt = time.monotonic()
        tried = []
        for path in self.candidate_ports():
            if not os.path.exists(path):
                tried.append(f'{path}: not present')
                continue
            try:
                port = open_mcu_port(path, self.baudrate)
            except (OSError, serial.SerialException) as e:
                tried.append(f'{path}: {e}')
                continue
            self.ser = port
            self.mcu_port_in_use = path
            self.mav_rx = mcu_to_ros2.MAVLink(None, srcSystem=2, srcComponent=1)
            self.mav_rx.robust_parsing = True
            self.mav_tx = ros2_to_mcu.MAVLink(port, srcSystem=2, srcComponent=1)
            self.mcu_connected = True
            self.mcu_announced = False
            which = '' if path == self.port else ' (fallback port)'
            self.get_logger().info(f'Connected to MCU on {path}{which}.')
            self.connect_notice_timer.reset()
            return True
        self.get_logger().warn(
            f'MCU not reachable ({"; ".join(tried)}), '
            f'retrying every {MCU_RECONNECT_INTERVAL_S:.0f} s.',
            throttle_duration_sec=10.0)
        return False

    def disconnect_mcu(self, reason):
        """Drop a dead port; poll_mcu reconnects every MCU_RECONNECT_INTERVAL_S."""
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.mav_tx = None
        self.mcu_connected = False
        self.mcu_announced = False
        self.connect_notice_timer.cancel()
        self.last_connect_attempt = time.monotonic()
        self.get_logger().warn(
            f'Lost the MCU on {self.mcu_port_in_use} ({reason}); '
            f'reconnecting every {MCU_RECONNECT_INTERVAL_S:.0f} s.')

    def mcu_send(self, what, send):
        """
        Run send() against the MCU link; True if it went out.

        A port that has died starts the reconnect cycle. A write that timed
        out (the MCU is not reading) only costs this frame.
        """
        if not self.mcu_connected:
            return False
        try:
            send()
            return True
        except serial.SerialTimeoutException:
            self.get_logger().warn(f'MCU is not reading, dropped a {what} frame.',
                                   throttle_duration_sec=2.0)
        except (OSError, serial.SerialException) as e:
            self.disconnect_mcu(f'{what} write failed: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to send {what} to MCU: {e}',
                                    throttle_duration_sec=2.0)
        return False

    def send_connect_notice(self):
        # one shot per connection: connect_mcu() re-arms it after a reconnect
        self.connect_notice_timer.cancel()
        if not self.mcu_connected:
            return
        self.mcu_announced = True
        # normally lit; off only if a node already asked for that on /status_led
        lit = self.status_led_lit
        lit_flag = 1 if lit else 0
        sent = self.mcu_send(
            'connect notice',
            lambda: self.mav_tx.gorur_gari_ros2_to_mcu_connect_msg_send(connected=lit_flag))
        if sent:
            self.get_logger().info(
                f'Announced connection to MCU (status LED {"on" if lit else "off"}).')

    # ---- status LED, driven through the connect frame (see STATUS_LED_TOPIC) ----

    def send_status_led(self, lit):
        """Remember the level and put it on the wire, once the MCU has been greeted."""
        self.status_led_lit = bool(lit)
        if not (self.mcu_connected and self.mcu_announced):
            return  # send_connect_notice will carry it
        lit_flag = 1 if self.status_led_lit else 0
        self.mcu_send('status LED', lambda: self.mav_tx.gorur_gari_ros2_to_mcu_connect_msg_send(
            connected=lit_flag))

    def stop_status_led_blink(self):
        if self.status_led_blink_timer is not None:
            self.destroy_timer(self.status_led_blink_timer)
            self.status_led_blink_timer = None

    def handle_status_led(self, msg: Bool):
        """Steady on or off. Also ends a blink."""
        self.stop_status_led_blink()
        self.status_led_steady = bool(msg.data)
        self.send_status_led(self.status_led_steady)
        self.get_logger().info(f'Status LED {"on" if self.status_led_steady else "off"}.')

    def handle_status_led_blink(self, msg: Float32):
        """Blink at msg.data Hz; zero or less goes back to the last steady level."""
        half_period = blink_half_period(float(msg.data), self.status_led_max_blink_hz)
        self.stop_status_led_blink()
        if half_period is None:
            self.send_status_led(self.status_led_steady)
            self.get_logger().info('Status LED blink stopped.')
            return
        # every blink starts lit, like StatusLed::blink() on the MCU
        self.send_status_led(True)
        self.status_led_blink_timer = self.create_timer(half_period, self.toggle_status_led)
        self.get_logger().info(f'Status LED blinking at {0.5 / half_period:.2f} Hz.')

    def toggle_status_led(self):
        self.send_status_led(not self.status_led_lit)

    def handle_run_time(self, msg: Float32):
        """Cache the run stopwatch. send_run_time is what puts it on the wire."""
        self.run_time_sec = float(msg.data)
        self.run_time_stamp = self.get_clock().now()

    def handle_run_timer_state(self, msg: String):
        """IDLE / RUNNING / STOPPED, from the same run_timer node."""
        self.run_timer_state = str(msg.data).strip().upper()

    def send_run_time(self):
        """
        Push the run stopwatch to the MCU so the OLED can show the run time.

        Silent until run_timer publishes for the first time, so a stack
        launched without it leaves the wire exactly as it was. Silent again
        once the stream goes stale: the firmware would rather mark its own
        clock stale than keep drawing a time nobody is updating any more.
        """
        if not self.mcu_connected or self.run_time_sec is None:
            return
        if (self.get_clock().now() - self.run_time_stamp
                > Duration(seconds=RUN_TIMER_STALE_S)):
            self.get_logger().warn(
                'No /run_time for over a second - is autonomy/run_timer still up? '
                'The OLED will mark its clock stale.',
                throttle_duration_sec=5.0)
            return

        elapsed_ms = run_time_to_wire_ms(self.run_time_sec)
        # an unknown state reads as idle, which shows a blank clock rather
        # than a running one that never moves
        state = RUN_TIMER_WIRE_STATE.get(self.run_timer_state, 0)

        self.mcu_send('run time', lambda: self.mav_tx.gorur_gari_ros2_to_mcu_timer_msg_send(
            elapsed_ms=elapsed_ms, state=state))

    def apply_launch_boost(self, linear_x, now):
        """
        Scale up the throttle that gets the bot moving again from a standstill.

        The first non-zero linear.x after a stop is multiplied by the boost gain
        (and every command within launch_boost_duration of it, so the kick is not
        gone in one 20 ms frame). After that the bot is rolling and the commanded
        velocity is passed straight through.
        """
        # a gap in cmd_vel means nobody is driving, so the bot rolled to a stop
        if (self.last_cmd_vel_time is not None
                and (now - self.last_cmd_vel_time) > Duration(seconds=self.launch_idle_timeout)):
            self.is_moving = False
            self.launch_boost_until = None
        self.last_cmd_vel_time = now

        if linear_x == 0.0:
            # commanded stop: next non-zero command starts a new move
            self.is_moving = False
            self.launch_boost_until = None
            return linear_x

        if not self.is_moving:
            self.is_moving = True
            self.launch_boost_until = now + Duration(seconds=self.launch_boost_duration)
            self.get_logger().info(
                f'Launch boost: x{self.launch_boost_gain} for {self.launch_boost_duration}s')
            return linear_x * self.launch_boost_gain

        if self.launch_boost_until is not None:
            if now < self.launch_boost_until:
                return linear_x * self.launch_boost_gain
            self.launch_boost_until = None  # boosted long enough, back to normal
        return linear_x

    def handle_cmd_vel(self, msg: Twist):
        try:
            # if not self.mcu_connected: #handle the case where the MCU is not connected
            #     self.get_logger().error("MCU is not connected. cannot send command.")
            #     return;
            linear_x = self.apply_launch_boost(msg.linear.x, self.get_clock().now())
            throttle = int(max(self.throttle_min,
                               min(self.throttle_max, linear_x * self.throttle_scale)))
            steering = max(self.steering_min_deg,
                           min(self.steering_max_deg,
                               int(self.steering_center_deg
                                   + msg.angular.z * self.steering_gain_deg)))
            self.get_logger().info(
                f'Sending cmd_vel to MCU: throttle={throttle}, steering={steering}')
            # not sent from here: send_drive_command puts it on the wire at a
            # steady 50 Hz for as long as it stays fresh
            self.drive_cmd = (throttle, steering)
            self.drive_cmd_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f'Error in handle_cmd_vel: {e}')
            return

    def send_drive_command(self):
        """
        50 Hz: re-send the latest drive command while /cmd_vel is fresh.

        Once /cmd_vel is older than CMD_VEL_STALE_S nothing is sent at all, and
        the firmware brakes and centres the steering on its own timeout
        (FIRMWARE_CMD_TIMEOUT_S) - the same path as the bridge dying or the
        UART being unplugged, so it is exercised every time a driving node
        stops.
        """
        age_s = None
        if self.drive_cmd_time is not None:
            age_s = (self.get_clock().now() - self.drive_cmd_time).nanoseconds * 1e-9

        if not drive_command_is_fresh(age_s):
            if self.drive_streaming:
                self.drive_streaming = False
                self.get_logger().warn(
                    f'/cmd_vel stale ({age_s:.2f} s old): stopped sending drive commands, '
                    f'the MCU brakes within {FIRMWARE_CMD_TIMEOUT_S * 1000:.0f} ms.')
            return

        if not self.drive_streaming:
            self.drive_streaming = True
            self.get_logger().info(
                f'Fresh /cmd_vel: sending drive commands to the MCU at '
                f'{1.0 / DRIVE_TX_INTERVAL_S:.0f} Hz.')

        if not self.mcu_connected:
            self.get_logger().error('MCU is not connected. cannot send command.',
                                    throttle_duration_sec=2.0)
            return
        throttle, steering = self.drive_cmd
        self.mcu_send('drive command', lambda: self.mav_tx.gorur_gari_ros2_to_mcu_msg_send(
            throttle=throttle, steering=steering))

    def poll_mcu(self):
        """50 Hz: read whatever the MCU sent, or try to get the port back."""
        if not self.mcu_connected:
            if (self.last_connect_attempt is None or
                    time.monotonic() - self.last_connect_attempt >= MCU_RECONNECT_INTERVAL_S):
                self.connect_mcu()
            return
        try:
            # read(>=1) with timeout 0 returns at once, and is what notices a
            # vanished device (EIO / "readiness to read but returned no data")
            data = self.ser.read(max(1, self.ser.in_waiting))
        except (OSError, serial.SerialException) as e:
            self.disconnect_mcu(f'read failed: {e}')
            return
        if not data:
            return
        try:
            msgs = self.mav_rx.parse_buffer(data) or []
        except Exception as e:
            self.get_logger().error(f'Error parsing MCU data: {e}', throttle_duration_sec=2.0)
            return
        for msg in msgs:
            msg_type = msg.get_type()
            if msg_type == 'GORUR_GARI_MCU_TO_ROS2_MSG':
                self.handle_mcu_sensor_msg(msg)
            elif msg_type in (f'UNKNOWN_{STATUSTEXT_MSG_ID}', 'STATUSTEXT'):
                self.handle_mcu_statustext(msg)
            elif msg_type == 'BAD_DATA':
                # resync noise (the ROM boot banner after a reset, or we opened
                # mid packet), not fatal
                self.get_logger().debug(f'Skipping non-MAVLink bytes: {msg.reason}')

    def handle_mcu_statustext(self, msg):
        """Log a STATUSTEXT from the MCU at the matching level."""
        if msg.get_type() == 'STATUSTEXT':  # already decoded by a full dialect
            decoded = (int(msg.severity), str(msg.text))
        else:
            decoded = decode_mcu_statustext(msg.get_msgbuf())
        if decoded is None:
            self.get_logger().debug('Undecodable STATUSTEXT from the MCU, skipped')
            return
        severity, text = decoded
        line = f'MCU: {text}'
        if severity <= mavlink_common.MAV_SEVERITY_ERROR:
            self.get_logger().error(line)
        elif severity == mavlink_common.MAV_SEVERITY_WARNING:
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)

    def handle_mcu_sensor_msg(self, msg):
        now = self.get_clock().now().to_msg()

        for cm, topic_pub, frame_id in zip(
            (msg.sonar_1, msg.sonar_2, msg.sonar_3, msg.sonar_4),
            self.sonar_pubs,
            SONAR_FRAME_IDS,
        ):
            if topic_pub is None:
                continue  # sonar not plugged in, nothing to report

            range_msg = Range()
            range_msg.header.stamp = now
            range_msg.header.frame_id = frame_id
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = SONAR_FIELD_OF_VIEW_RAD
            range_msg.min_range = 0.02
            range_msg.max_range = SONAR_MAX_RANGE_M
            # 255 means "no echo within timeout" -> report max range, not a false reading
            if cm >= SONAR_NO_READING_CM:
                range_msg.range = SONAR_MAX_RANGE_M
            else:
                range_msg.range = cm / 100.0
            topic_pub.publish(range_msg)

        direction_sign = WIRE_DIRECTION_TO_SIGN.get(msg.encoder_direction, 0)

        # encoder_count and heading now travel raw (same values the MCU's
        # OLED prints), no reconstruction needed on this end.
        self.encoder_count_pub.publish(Int32(data=msg.encoder_count))
        self.encoder_speed_pub.publish(Float32(data=float(msg.encoder_speed)))
        self.encoder_direction_pub.publish(Int8(data=direction_sign))
        self.steering_angle_pub.publish(UInt8(data=msg.servo))
        self.heading_pub.publish(Float32(data=msg.heading))
        # 1 = held down, or tapped since the MCU's last frame (see firmware/src/main.cpp)
        self.button_pub.publish(Bool(data=bool(msg.button)))

    def odom_handler(self):
        # Placeholder for odometry handling logic
        pass


def main(args=None):
    rclpy.init(args=args)
    mcu_bridge_node = MCUBridgeNode()
    try:
        rclpy.spin(mcu_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        mcu_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
