import math

import geometry_msgs
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from pymavlink import mavutil
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

MCU_PORT = '/dev/esp32_s3'  # udev alias for the ESP32-S3 native USB CDC port
MCU_BAUDRATE = 115200

# the drivetrain does not make enough torque at cruise throttle to break static
# friction, so the command that starts a move from a standstill gets scaled up
# for a moment. once the bot is rolling, momentum carries it and the commanded
# velocity goes through untouched.
LAUNCH_BOOST_GAIN = 2.0
LAUNCH_BOOST_DURATION_S = 1.0  # 0.0 -> boost only the single command that starts the move
LAUNCH_IDLE_TIMEOUT_S = 0.5  # no cmd_vel for this long counts as stopped, so the boost re-arms

# how long to wait after opening the port before announcing ourselves to the MCU.
# long enough for the esp32 to clear its bootloader if opening the port reset it.
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
        self.declare_parameter('baudrate', MCU_BAUDRATE)
        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.get_logger().info(f'Connecting to MCU on {self.port} at {self.baudrate} baud.')
        self.mcu_connected = False;

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

        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baudrate)
            self.mav_rx = mcu_to_ros2.MAVLink(self.master, srcSystem=2, srcComponent=1)
            # mavutil sets this on the parser it builds itself, but we replace that
            # parser with the mcu_to_ros2 dialect and the default is False. Without
            # it any non-MAVLink byte (the esp32 boot banner it prints when we open
            # the port) raises instead of resyncing, killing the whole read loop.
            self.mav_rx.robust_parsing = True
            self.master.mav = self.mav_rx
            # pymavlink defaults this connection to wire protocol 1.0, so the first
            # 0xFD (MAVLink2 magic) byte makes mavutil "auto upgrade" by throwing our
            # dialect away and installing stock ardupilotmega, which has never heard
            # of msgid 50001 - every frame then decodes as BAD_DATA. Declare 2.0 up
            # front and mark the stream as already sniffed so that never fires.
            self.master.WIRE_PROTOCOL_VERSION = '2.0'
            self.master.first_byte = False

            self.mav_tx = ros2_to_mcu.MAVLink(self.master, srcSystem=2, srcComponent=1);
            self.get_logger().info('Successfully connected to MCU.')
            self.mcu_connected = True

        except Exception as e:
            self.get_logger().error(f'Failed to connect to MCU: {e}')
            # rclpy.shutdown()
            # return
        self.cmd_vel = self.create_subscription(Twist,'/cmd_vel', self.handle_cmd_vel, 10)
        self.run_time_sub = self.create_subscription(
            Float32, '/run_time', self.handle_run_time, 10)
        self.run_timer_state_sub = self.create_subscription(
            String, '/run_timer_state', self.handle_run_timer_state, 10)
        # tell the MCU we are here, exactly once. it lights its status LED on this
        # and nothing else, so this is the one announcement that matters. fired off
        # a timer rather than inline because opening the port toggles DTR, which can
        # reset the esp32 - a frame sent right now would land in the bootloader.
        self.connect_notice_timer = self.create_timer(
            MCU_CONNECT_NOTICE_DELAY_S, self.send_connect_notice)
        # drain incoming sensor telemetry at 50 Hz
        self.mcu_poll_timer = self.create_timer(0.02, self.poll_mcu)
        # push the run stopwatch out to the OLED at 10 Hz
        self.run_timer_tx_timer = self.create_timer(
            RUN_TIMER_TX_INTERVAL_S, self.send_run_time)
        # self.timer = self.create_timer(0.1, self.send_heartbeat)  # Send heartbeat every 0.1 seconds


    def send_connect_notice(self):
        # one shot: whether or not the frame gets through, we never send it again
        self.connect_notice_timer.cancel()
        if not self.mcu_connected:
            return
        try:
            self.mav_tx.gorur_gari_ros2_to_mcu_connect_msg_send(connected=1)
            self.get_logger().info('Announced connection to MCU (status LED on).')
        except Exception as e:
            self.get_logger().error(f'Failed to announce connection to MCU: {e}')

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

        try:
            self.mav_tx.gorur_gari_ros2_to_mcu_timer_msg_send(
                elapsed_ms=elapsed_ms, state=state)
        except Exception as e:
            self.get_logger().error(
                f'Failed to send the run time to the MCU: {e}',
                throttle_duration_sec=5.0)

    def apply_launch_boost(self, linear_x, now):
        """Scale up the throttle that gets the bot moving again from a standstill.

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

    def handle_cmd_vel(self,msg:Twist):
        try:
            # if not self.mcu_connected: #handle the case where the MCU is not connected
            #     self.get_logger().error("MCU is not connected. cannot send command.")
            #     return;
            linear_x = self.apply_launch_boost(msg.linear.x, self.get_clock().now())
            throttle = int(max(self.throttle_min,
                               min(self.throttle_max, linear_x * self.throttle_scale)))
            steering = max(self.steering_min_deg,
                           min(self.steering_max_deg,
                               int(self.steering_center_deg + msg.angular.z * self.steering_gain_deg)))
            self.get_logger().info(f'Sending cmd_vel to MCU: throttle={throttle}, steering={steering}')
            if self.mcu_connected:
                self.mav_tx.gorur_gari_ros2_to_mcu_msg_send(
                    throttle=throttle,
                    steering=steering
                )
            else:
                self.get_logger().error("MCU is not connected. cannot send command.")
        except Exception as e:
            self.get_logger().error(f'Error in handle_cmd_vel: {e}')
            return

    def poll_mcu(self):
        if not self.mcu_connected:
            return
        try:
            while True:
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    break
                msg_type = msg.get_type()
                if msg_type == 'GORUR_GARI_MCU_TO_ROS2_MSG':
                    self.handle_mcu_sensor_msg(msg)
                elif msg_type == 'BAD_DATA':
                    # resync noise (boot banner, or we opened mid packet), not fatal
                    self.get_logger().debug(f'Skipping non-MAVLink bytes: {msg.reason}')
        except Exception as e:
            self.get_logger().error(f'Error polling MCU: {e}')

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

    def send_heartbeat(self):
        msg = self.master.recv_match(blocking=False)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                self.get_logger().info('Received heartbeat from MCU.')
            if msg.get_type() == 'COMMAND_ACK':
                self.get_logger().info(f'Received command acknowledgment: {msg}')
            else:
                self.get_logger().info(f'Received message: {msg}')
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, 0
            )
            self.get_logger().info('Heartbeat sent to MCU.')
        except Exception as e:
            self.get_logger().error(f'Failed to send heartbeat: {e}')
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