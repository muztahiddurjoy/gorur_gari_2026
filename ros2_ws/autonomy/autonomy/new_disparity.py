import math
import time
from threading import Lock, Thread

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int32, Bool, Empty
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

# Run states of the start gate.
STATE_STANDBY = 'STANDBY'   # powered up, waiting for the button
STATE_ARMING = 'ARMING'     # button seen, counting the start delay down
STATE_RUNNING = 'RUNNING'   # driving

# While parked we still send zeros so the MCU never sits on a stale throttle,
# but at a fraction of the control rate - there is nothing to steer.
STOP_REPUBLISH_INTERVAL_S = 0.5


def clamp(val, mini, maxi):
    """Constrain value between min and max (handles swapped bounds)."""
    lo, hi = (mini, maxi) if mini <= maxi else (maxi, mini)
    return max(lo, min(hi, val))


def remap(old_val, old_min, old_max, new_min, new_max):
    """Map a value from one range to another, clamped."""
    new_val = (new_max - new_min) * (old_val - old_min) / (old_max - old_min) + new_min
    return clamp(new_val, new_min, new_max)

def lerp(a, b, t):
    """Linear interpolation between a and b by factor t, clamped."""
    return clamp(a + (b - a) * t, a, b)


class DisparityExtenderNode(Node):

    def __init__(self):
        super().__init__('disparity_extender_node')

        # ── Vehicle Geometry (Circle Cast) ────────────────────────────
        self.declare_parameter('cast_range_min', 0.13)
        self.declare_parameter('cast_range_max', 0.16)
        self.declare_parameter('cast_precision', 81)
        self.declare_parameter('cast_skip', 4)
        self.declare_parameter('cast_skip_fine', 1)

        # ── FOV ───────────────────────────────────────────────────────
        self.declare_parameter('look_range_deg', 80.0)
        self.declare_parameter('vision_angle_tolerance_deg', 15.0)

        # ── Speed ─────────────────────────────────────────────────────
        self.declare_parameter('max_speed', 0.60)
        self.declare_parameter('speed_cap_corner', 0.30)
        self.declare_parameter('speed_cap_straight', 0.45)
        self.declare_parameter('boost_max', 1.35)
        self.declare_parameter('boost_angle_thresh', 7.5)
        self.declare_parameter('boost_dist_thresh', 1.10)

        # ── Tower Detection ───────────────────────────────────────────
        self.declare_parameter('edge_slope_thresh', 0.25)
        self.declare_parameter('tower_width_min', 0.02)
        self.declare_parameter('tower_width_max', 0.10)

        # ── Safety ────────────────────────────────────────────────────
        self.declare_parameter('danger_dist', 0.22)
        self.declare_parameter('danger_angle_min', 25.0)
        self.declare_parameter('danger_angle_max', 90.0)

        # ── Master Controls ────────────────────────────────────────────
        self.declare_parameter('enable_auto_steering', False)

        # ── Start Gate ────────────────────────────────────────────────
        self.declare_parameter('require_button_start', True)
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('button_topic', '/button_status')

        # ── Steering ─────────────────────────────────────────────────
        self.declare_parameter('str_ang_thresh', 60.0)

        # ── Game Modes ───────────────────────────────────────────────
        self.declare_parameter('chase_tower_mode', False)

        # ── Topics ────────────────────────────────────────────────────
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Load all parameters
        self.enable_auto_steering = bool(self.get_parameter('enable_auto_steering').value)

        self.cast_range_min = float(self.get_parameter('cast_range_min').value)
        self.cast_range_max = float(self.get_parameter('cast_range_max').value)
        self.cast_precision = int(self.get_parameter('cast_precision').value)
        self.cast_skip = int(self.get_parameter('cast_skip').value)
        self.cast_skip_fine = int(self.get_parameter('cast_skip_fine').value)

        self.look_range_rad = math.radians(float(self.get_parameter('look_range_deg').value))
        self.vision_angle_tolerance_rad = math.radians(float(self.get_parameter('vision_angle_tolerance_deg').value))

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.speed_cap_corner = float(self.get_parameter('speed_cap_corner').value)
        self.speed_cap_straight = float(self.get_parameter('speed_cap_straight').value)
        self.boost_max = float(self.get_parameter('boost_max').value)
        self.boost_angle_thresh = float(self.get_parameter('boost_angle_thresh').value)
        self.boost_dist_thresh = float(self.get_parameter('boost_dist_thresh').value)

        self.edge_slope_thresh = float(self.get_parameter('edge_slope_thresh').value)
        self.tower_width_min = float(self.get_parameter('tower_width_min').value)
        self.tower_width_max = float(self.get_parameter('tower_width_max').value)

        self.danger_dist = float(self.get_parameter('danger_dist').value)
        self.danger_angle_min = float(self.get_parameter('danger_angle_min').value)
        self.danger_angle_max = float(self.get_parameter('danger_angle_max').value)

        self.str_ang_thresh = float(self.get_parameter('str_ang_thresh').value)

        self.chase_tower_mode = bool(self.get_parameter('chase_tower_mode').value)

        self.require_button_start = bool(self.get_parameter('require_button_start').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        button_topic = self.get_parameter('button_topic').value

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # ── LiDAR State ──────────────────────────────────────────────
        self.ang_min = -math.pi
        self.ang_max = math.pi
        self.ang_inc = math.radians(0.5)
        self.ranges = []
        self.intensities = []
        self.new_lidar_val = False
        self._scan_lock = Lock()  # Guards shared LiDAR state between callback and calc thread

        # ── Navigation State ─────────────────────────────────────────
        self.current_speed = 0.0
        self.speed = 0.0
        self.speed_boost = 1.0
        self.speed_cap = self.speed_cap_straight
        self.target_cap = self.speed_cap_straight
        self.str_angle = 0.0
        self.target_ang = 0.0
        self.target_dist = 0.0
        self.cast_r = self.cast_range_min
        self.closest_color = "N"  # "R", "G", or "N"
        self.camera_pillar_angle = 0.0
        self.detected_towers = []
        self.last_time = time.time()
        self.lap_count = 0
        self.has_reset_laps = False
        self.reset_timer_ticks = 0

        # ── Start Gate State ─────────────────────────────────────────
        # With the gate disabled the node behaves as it always did: it drives as
        # soon as it has scan data.
        self.run_state = STATE_STANDBY if self.require_button_start else STATE_RUNNING
        self.arm_start_time = 0.0
        self.prev_button = False       # for rising edge detection on /button_status
        self.last_countdown_logged = -1
        self.last_stop_pub_time = 0.0
        self.last_scan_time = 0.0         # Watchdog: last time /scan was received

        # ── Subscriptions & Publishers ────────────────────────────────
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.lidar_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.color_sub = self.create_subscription(Float32MultiArray, '/closest_obj', self.color_callback, 1)
        self.lap_sub = self.create_subscription(Int32, '/lap_count', self.lap_callback, 10)
        self.button_sub = self.create_subscription(Bool, button_topic, self.button_callback, 10)
        self.lap_reset_pub = self.create_publisher(Empty, '/reset_lap_count', 10)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.debug_scan_pub = self.create_publisher(LaserScan, '/scan_processed', 10)
        self.marker_pub = self.create_publisher(Marker, '/disparity/target_marker', 10)
        self.tower_pub = self.create_publisher(Float32MultiArray, '/tower_detections', 10)

        self.last_color_time = 0.0

        # ── Control Loop Timer (40 Hz) ────────────────────────────────
        self.create_timer(0.025, self.control_loop)

        # ── LiDAR Processing Timer (20 Hz) ────────────────────────────
        self.create_timer(0.05, self.calc_lidar_step)

        self.get_logger().info(
            f'[Disparity Extender] Initialized | '
            f'Cast: [{self.cast_range_min:.2f}m – {self.cast_range_max:.2f}m] | '
            f'FOV: ±{math.degrees(self.look_range_rad):.0f}° | '
            f'Chase Mode: {self.chase_tower_mode}'
        )
        if self.require_button_start:
            self.get_logger().info(
                f'STANDBY — press the start button ({button_topic}) to run. '
                f'The car moves {self.start_delay_sec:.0f}s after the press.'
            )
        else:
            self.get_logger().warn('Start button gate disabled — driving immediately.')
