import geometry_msgs
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from contols import mcu_to_ros2
from contols import ros2_to_mcu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Int32, Int8, UInt8, Float32

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

# heading travels as centidegrees in a uint16_t, published as degrees 0..360
HEADING_CDEG_PER_DEG = 100.0

class MCUBridgeNode(Node):
    def __init__(self):
        super().__init__('mcu_bridge')
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.get_logger().info(f'Connecting to MCU on {self.port} at {self.baudrate} baud.')
        self.mcu_connected = False;
        self.encoder_total = 0

        # override without editing code, e.g.
        #   ros2 run contols mcu_bridge --ros-args -p sonar_front_enabled:=true
        self.sonar_enabled = []
        for name in SONAR_NAMES:
            param = f'sonar_{name}_enabled'
            self.declare_parameter(param, SONAR_ENABLED_DEFAULTS[name])
            self.sonar_enabled.append(bool(self.get_parameter(param).value))

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
        # drain incoming sensor telemetry at 50 Hz
        self.mcu_poll_timer = self.create_timer(0.02, self.poll_mcu)
        # self.timer = self.create_timer(0.1, self.send_heartbeat)  # Send heartbeat every 0.1 seconds


    def handle_cmd_vel(self,msg:Twist):
        try:
            # if not self.mcu_connected: #handle the case where the MCU is not connected
            #     self.get_logger().error("MCU is not connected. cannot send command.")
            #     return;
            throttle = int(max(-128, min(127, msg.linear.x * 100)))  # Scale linear.x to -128-127
            steering = max(45, min(135, int(90+ msg.angular.z * 45)))  # Scale angular.z to 45-135 degrees
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
        self.encoder_total += direction_sign * msg.encoder_count

        self.encoder_count_pub.publish(Int32(data=self.encoder_total))
        self.encoder_speed_pub.publish(Float32(data=float(msg.encoder_speed)))
        self.encoder_direction_pub.publish(Int8(data=direction_sign))
        self.steering_angle_pub.publish(UInt8(data=msg.servo))
        self.heading_pub.publish(Float32(data=msg.heading / HEADING_CDEG_PER_DEG))

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