import geometry_msgs
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from pymavlink import mavutil
from controls import mcu_to_ros2
from controls import ros2_to_mcu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Int32, Int8, UInt8, Float32

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

# the drivetrain does not make enough torque at cruise throttle to break static
# friction, so the command that starts a move from a standstill gets scaled up
# for a moment. once the bot is rolling, momentum carries it and the commanded
# velocity goes through untouched.
LAUNCH_BOOST_GAIN = 2
LAUNCH_BOOST_DURATION_S = 1  # 0.0 -> boost only the single command that starts the move
LAUNCH_IDLE_TIMEOUT_S = 0.5  # no cmd_vel for this long counts as stopped, so the boost re-arms

# how long to wait after opening the port before announcing ourselves to the MCU.
# long enough for the esp32 to clear its bootloader if opening the port reset it.
MCU_CONNECT_NOTICE_DELAY_S = 1.0

class MCUBridgeNode(Node):
    def __init__(self):
        super().__init__('mcu_bridge')
        self.port = '/dev/esp32_s3'
        self.baudrate = 115200
        self.get_logger().info(f'Connecting to MCU on {self.port} at {self.baudrate} baud.')
        self.mcu_connected = False;

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
        # tell the MCU we are here, exactly once. it lights its status LED on this
        # and nothing else, so this is the one announcement that matters. fired off
        # a timer rather than inline because opening the port toggles DTR, which can
        # reset the esp32 - a frame sent right now would land in the bootloader.
        self.connect_notice_timer = self.create_timer(
            MCU_CONNECT_NOTICE_DELAY_S, self.send_connect_notice)
        # drain incoming sensor telemetry at 50 Hz
        self.mcu_poll_timer = self.create_timer(0.02, self.poll_mcu)
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
            throttle = int(max(-128, min(127, linear_x * 100)))  # Scale linear.x to -128-127
            steering = max(0, min(180, int(90+ msg.angular.z * 60)))  # Scale angular.z to 0-180 degrees
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