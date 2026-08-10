"""
Modified Disparity Extender Algorithm — Ported from LazyGo WRO2025.

This node implements Team LazyGo's Circle-Cast Ray Marching algorithm for
autonomous navigation, adapted to the gorur_gari_2026 ROS 2 workspace.

Core Algorithm:
1. For every candidate ray in the forward FOV, "march" a circle of the robot's
   half-width along it. If any neighboring ray's obstacle point falls within
   that circle, shorten the candidate's safe distance.
2. Pick the ray with the maximum safe distance → that's the target heading.
3. Detect towers via edge contrast (falling/rising slope pairs) and filter by
   physical width (s = r·θ ≈ 5cm).
4. Apply WRO color rules: Green = pass left, Red = pass right.
5. Boost speed on clear straights, emergency override on close side obstacles.

Start gate:
The node never drives on its own. It comes up in STANDBY - spinning, processing
LiDAR, publishing debug topics, but holding the car at zero. Pressing the start
button on the car (MCU publishes /button_status) moves it to ARMING, and after
start_delay_sec it goes RUNNING and the algorithm above takes the wheel. The
MCU blinks its status LED once per second through that same delay, so the
countdown is visible on the car itself (see firmware/src/main.cpp).

Reference: https://github.com/A-N-M-Noor/LazyGo_WRO2025
"""

import math
import time
from threading import Lock, Thread

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String, Int32, Empty
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
        self.color_sub = self.create_subscription(String, '/closest_obj', self.color_callback, 1)
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
            f'[LazyGo Disparity Extender] Initialized | '
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

    # ══════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════

    def odom_callback(self, msg: Odometry):
        """Track vehicle speed for dynamic cast radius."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)

    def color_callback(self, msg: String):
        """Update the active WRO color override (R, G, N)."""
        self.closest_color = msg.data.strip().upper()
        self.last_color_time = time.time()
        if self.closest_color not in ("R", "G"):
            self.closest_color = "N"

    def lap_callback(self, msg: Int32):
        """Update lap count from the lap_counter node."""
        self.lap_count = msg.data

    def button_callback(self, msg: Bool):
        """
        Start button from the MCU. /button_status stays true for as long as the
        button is held, so only the press edge counts - otherwise holding it
        would re-arm on every frame.
        """
        pressed = bool(msg.data)
        rising = pressed and not self.prev_button
        self.prev_button = pressed
        if not rising:
            return

        if self.run_state == STATE_STANDBY:
            self.run_state = STATE_ARMING
            self.arm_start_time = time.time()
            self.last_countdown_logged = -1
            self.get_logger().info(
                f'Start button pressed — going in {self.start_delay_sec:.0f}s.')
        else:
            self.get_logger().info(
                f'Start button pressed but already {self.run_state}, ignoring.')

    def update_run_state(self):
        """Advance ARMING → RUNNING once the start delay has elapsed."""
        if self.run_state != STATE_ARMING:
            return

        remaining = self.start_delay_sec - (time.time() - self.arm_start_time)
        if remaining <= 0.0:
            self.run_state = STATE_RUNNING
            self.get_logger().info('GO — disparity extender is driving.')
            return

        # one line per second, matching the MCU's LED blinks
        secs_left = int(math.ceil(remaining))
        if secs_left != self.last_countdown_logged:
            self.last_countdown_logged = secs_left
            self.get_logger().info(f'Starting in {secs_left}...')

    def publish_stop(self):
        """Hold the car at zero, republished slowly so it can't latch a stale command."""
        now = time.time()
        if now - self.last_stop_pub_time < STOP_REPUBLISH_INTERVAL_S:
            return
        self.last_stop_pub_time = now
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
    def lidar_callback(self, msg: LaserScan):
        """Buffer incoming LaserScan data for the background thread."""
        with self._scan_lock:
            self.ang_min = msg.angle_min
            self.ang_max = msg.angle_max
            self.ang_inc = msg.angle_increment
            self.ranges = list(msg.ranges)
            self.intensities = list(msg.intensities) if msg.intensities else [1.0] * len(msg.ranges)
            self.range_min = msg.range_min
            self.range_max = msg.range_max
            self.scan_header = msg.header
            self.new_lidar_val = True
            self.last_scan_time = time.time()

    def control_loop(self):
        """Publish motor commands at a fixed rate."""
        # The gate is ticked before the enable check so the countdown keeps
        # running even while auto steering is switched off.
        self.update_run_state()

        if not bool(self.get_parameter('enable_auto_steering').value):
            return

        # ── LAP RESET LOGIC ──
        if not self.has_reset_laps:
            self.reset_timer_ticks += 1
            if self.reset_timer_ticks >= 20:  # Wait 0.5s for ROS 2 Discovery to connect publishers
                self.lap_reset_pub.publish(Empty())
                self.has_reset_laps = True
                self.lap_count = 0  # Force local reset too
                self.get_logger().info('Sent lap count reset signal on startup.')
            else:
                return  # Don't drive while waiting for reset

        # ── START GATE: STANDBY / ARMING park the car ──
        if self.run_state != STATE_RUNNING:
            self.publish_stop()
            self.get_logger().info(
                f'{self.run_state} — waiting for the start button.',
                throttle_duration_sec=5.0)
            return
        # ── LAP COMPLETION STOP ──
        if self.lap_count >= 3:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info('3 Laps Completed! Bot Stopped.', throttle_duration_sec=1.0)
            return

        # ── LIDAR WATCHDOG ──
        if self.last_scan_time > 0.0 and (time.time() - self.last_scan_time) > 0.2:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().error(
                'LiDAR timeout! No /scan for >200ms — emergency stop.',
                throttle_duration_sec=1.0)
            return

        cmd = Twist()
        capped_speed = clamp(self.speed * self.speed_boost, -self.speed_cap * self.speed_boost,
                             self.speed_cap * self.speed_boost)
        cmd.linear.x = float(capped_speed)
        cmd.angular.z = float(self.str_angle)
        self.cmd_pub.publish(cmd)

    # ══════════════════════════════════════════════════════════════════
    # LiDAR Index ↔ Angle Helpers
    # ══════════════════════════════════════════════════════════════════

    def a2i(self, ang_rad):
        """Angle (radians) → array index."""
        return round((ang_rad - self.ang_min) / self.ang_inc)

    def i2a(self, i):
        """Array index → angle (radians)."""
        return self.ang_min + self.ang_inc * i

    def ind_range(self, ang_min_rad, ang_max_rad):
        """Return [start_idx, end_idx] for the given angle window."""
        return [self.a2i(ang_min_rad), self.a2i(ang_max_rad)]

    # ══════════════════════════════════════════════════════════════════
    # Missing Data Interpolation (from LazyGo's LidarHandler)
    # ══════════════════════════════════════════════════════════════════

    def fix_missing(self, i):
        """
        Interpolate missing/invalid LiDAR rays using nearest valid neighbors.
        Invalid = intensity ≤ 0.05, range > 4m, or inf.
        """
        n = len(self.ranges)
        if i < 0 or i >= n:
            return 0.0

        # If valid, return as-is
        if (len(self.intensities) > i and self.intensities[i] > 0.05
                and self.ranges[i] <= 4.0
                and self.ranges[i] != float('inf')
                and not math.isnan(self.ranges[i])):
            return self.ranges[i]

        # Search left for a valid neighbor
        first = i
        while first > 0:
            first -= 1
            if (self.intensities[first] > 0.05
                    and self.ranges[first] <= 4.0
                    and self.ranges[first] != float('inf')
                    and not math.isnan(self.ranges[first])):
                break
        else:
            first = 0

        # Search right for a valid neighbor
        last = i
        while last < n - 1:
            last += 1
            if (self.intensities[last] > 0.05
                    and self.ranges[last] <= 4.0
                    and self.ranges[last] != float('inf')
                    and not math.isnan(self.ranges[last])):
                break
        else:
            last = n - 1

        # Validate neighbors
        if (self.intensities[first] <= 0.05 or self.ranges[first] > 4.0 or
            self.intensities[last] <= 0.05 or self.ranges[last] > 4.0):
            return 0.0
        if first == last:
            return self.ranges[first]

        lerp_factor = (i - first) / (last - first) if last != first else 0.5

        # If large distance gap between neighbors, snap to closer side
        if abs(self.ranges[first] - self.ranges[last]) > 0.1:
            return self.ranges[first] if lerp_factor < 0.5 else self.ranges[last]

        return lerp(self.ranges[first], self.ranges[last], lerp_factor)

    def fix_all_missing(self):
        """Fix all invalid rays within the search FOV (+ margin)."""
        chk = self.ind_range(-self.look_range_rad, self.look_range_rad)
        margin = 50
        start = max(0, chk[0] - margin)
        end = min(len(self.ranges), chk[1] + margin)
        for i in range(start, end):
            self.ranges[i] = self.fix_missing(i)

    # ══════════════════════════════════════════════════════════════════
    # Circle-Cast Ray Marching (LazyGo's core algorithm)
    # ══════════════════════════════════════════════════════════════════

    def hit_circle(self, ray_ang, check_dst, check_ang, radius):
        """
        Geometric check: would a circle of `radius` centered on the ray at
        `ray_ang` collide with a point at distance `check_dst` and angle
        `check_ang`?

        Returns the check point's distance if collision, else False.
        """
        d_theta = abs(ray_ang - check_ang)
        if check_dst <= 0.0:
            return False
        coll_ang = radius / check_dst
        if d_theta <= coll_ang:
            return check_dst
        return False

    def marching(self, indx, radius=None):
        """
        Circle-cast / ray marching for a single candidate ray at `indx`.

        Checks all neighboring rays within ±cast_precision to see if the
        robot's circular body (of radius `radius`) would hit any obstacle.

        Returns: {"dst": safe_distance, "ang": ray_angle}
        """
        if radius is None:
            radius = self.cast_r

        n = len(self.ranges)
        if indx < 0 or indx >= n:
            return {"dst": 0.0, "ang": 0.0}

        target_dst = self.ranges[indx]
        target_ang = self.i2a(indx)

        rng_start = indx - self.cast_precision * self.cast_skip_fine
        rng_end = indx + self.cast_precision * self.cast_skip_fine

        min_hit = {"dst": 1000.0, "ang": 0.0}

        for i in range(rng_start, rng_end, self.cast_skip_fine):
            if i < 0 or i >= n:
                continue

            ray_dst = self.ranges[i]
            ray_ang = self.i2a(i)

            hit = self.hit_circle(target_ang, ray_dst, ray_ang, radius)
            if hit and hit < min_hit["dst"]:
                min_hit = {"dst": ray_dst, "ang": target_ang}

        if min_hit["dst"] >= 1000.0:
            return {"dst": target_dst, "ang": target_ang}
        else:
            return {"dst": min_hit["dst"], "ang": min_hit["ang"]}

    # ══════════════════════════════════════════════════════════════════
    # Tower Detection via Edge Contrast
    # ══════════════════════════════════════════════════════════════════

    def find_towers(self):
        """
        Detect towers by scanning for falling→rising edge pairs in LiDAR.
        Filter by physical width: tower_width_min < s = r·θ < tower_width_max.
        """
        towers = []
        cont_stack = []
        n = len(self.ranges)
        chk = self.ind_range(-self.look_range_rad, self.look_range_rad)

        for i in range(max(self.cast_skip, chk[0]), min(n, chk[1])):
            slope = self.ranges[i] - self.ranges[i - self.cast_skip]

            # Falling edge (object starts)
            if slope < -self.edge_slope_thresh:
                cont_stack.append(i)
            # Rising edge (object ends)
            elif slope > self.edge_slope_thresh:
                if cont_stack:
                    pop = cont_stack.pop()
                    mid = (i + pop) // 2
                    if 0 <= mid < n and self.ranges[mid] > 0.0:
                        # Physical width of the detected object
                        sz = self.ranges[mid] * abs(i - pop) * self.ang_inc
                        if self.tower_width_min < sz < self.tower_width_max:
                            ang = self.i2a(mid)
                            towers.append({
                                "index": mid,
                                "ang": ang,
                                "dst": self.ranges[mid],
                                "x": self.ranges[mid] * math.cos(ang),
                                "y": self.ranges[mid] * math.sin(ang),
                            })

        # Sort by distance (closest first)
        towers.sort(key=lambda t: t["dst"])
        return towers

    # ══════════════════════════════════════════════════════════════════
    # Max Safe Distance Selection (with WRO Color Override)
    # ══════════════════════════════════════════════════════════════════

    def get_max_d(self, towers):
        """
        Scan all rays in the forward FOV, compute safe distance via marching,
        and return the direction with the maximum safe distance.

        If a tower color override is active (Green → pass left, Red → pass right),
        restrict the search range to enforce the correct side.
        """
        best = {"dst": 0.0, "ang": 0.0}
        chk = self.ind_range(-self.look_range_rad, self.look_range_rad)
        n = len(self.ranges)

        # WRO Color Override: restrict search range
        if towers and self.closest_color == "G":
            # Green → only search rays LEFT of the closest tower (force pass left)
            chk[0] = towers[0]["index"]
        elif towers and self.closest_color == "R":
            # Red → only search rays RIGHT of the closest tower (force pass right)
            chk[1] = towers[0]["index"]

        for i in range(chk[0], chk[1], self.cast_skip):
            if i < 0 or i >= n:
                continue

            # Skip invalid data
            if self.intensities[i] <= 0.05 or self.ranges[i] > 3.0:
                self.ranges[i] = self.fix_missing(i)
                self.intensities[i] = 1.0

            if self.ranges[i] <= 0.0 or self.ranges[i] > 3.0:
                continue

            # Circle-cast marching
            dt = self.marching(i)

            if dt["dst"] > best["dst"]:
                best = dt

        return best["dst"], best["ang"]

    # ══════════════════════════════════════════════════════════════════
    # Emergency Danger Sense
    # ══════════════════════════════════════════════════════════════════

    def danger_sense(self, target_ang_deg):
        """
        If any obstacle is within danger_dist at side angles
        [danger_angle_min, danger_angle_max], and the current heading steers
        toward it, apply a proportional corrective steer AWAY from the obstacle.

        Severity scales linearly with proximity:
          - At the danger boundary: no correction (severity = 0)
          - At contact distance:    full corrective override (severity = 1)

        The correction blends between the original target heading and a fixed
        escape angle (half of str_ang_thresh toward the safe side).
        """
        n = len(self.ranges)
        closest_left_dist = float('inf')
        closest_right_dist = float('inf')

        # Only scan the two side danger zones instead of all N rays
        left_zone = self.ind_range(
            math.radians(self.danger_angle_min),
            math.radians(self.danger_angle_max))
        right_zone = self.ind_range(
            math.radians(-self.danger_angle_max),
            math.radians(-self.danger_angle_min))

        for i in list(range(left_zone[0], left_zone[1])) + list(range(right_zone[0], right_zone[1])):
            if i < 0 or i >= n:
                continue
            if self.intensities[i] <= 0.05 or self.ranges[i] > 3.0:
                continue
            if self.ranges[i] < self.danger_dist:
                ang = math.degrees(self.i2a(i))
                if ang > 0 and self.ranges[i] < closest_left_dist:
                    closest_left_dist = self.ranges[i]
                elif ang < 0 and self.ranges[i] < closest_right_dist:
                    closest_right_dist = self.ranges[i]

        escape_angle = self.str_ang_thresh * 0.5  # Max corrective steer (30° default)

        # Left danger: obstacle on left side, heading steers left → push right
        if closest_left_dist < self.danger_dist and target_ang_deg > 0:
            severity = 1.0 - (closest_left_dist / self.danger_dist)
            return target_ang_deg * (1.0 - severity) + (-escape_angle) * severity

        # Right danger: obstacle on right side, heading steers right → push left
        if closest_right_dist < self.danger_dist and target_ang_deg < 0:
            severity = 1.0 - (closest_right_dist / self.danger_dist)
            return target_ang_deg * (1.0 - severity) + escape_angle * severity

        return target_ang_deg

    # ══════════════════════════════════════════════════════════════════
    # Main LiDAR Processing Timer (LazyGo's calc_lidar)
    # ══════════════════════════════════════════════════════════════════

    def calc_lidar_step(self):
        """
        Timer callback: runs the full navigation pipeline on each new scan.
        """
        if not self.new_lidar_val:
            return

        # Atomically copy LiDAR data under lock, then process outside lock
        with self._scan_lock:
            self.new_lidar_val = False
            ranges_copy = self.ranges.copy()
            intensities_copy = self.intensities.copy()

        self.ranges = ranges_copy
        self.intensities = intensities_copy

        dt = time.time() - self.last_time
        self.last_time = time.time()
        dt = max(dt, 0.001)  # Prevent division by zero

        n = len(self.ranges)
        if n == 0:
            return

        # --- Color Decay ---
        if time.time() - self.last_color_time > 1.0:
            self.closest_color = "N"

        # 1. Dynamic Cast Radius (robot width expands with speed)
        speed_ratio = self.current_speed / self.max_speed if self.max_speed > 0 else 0.0
        self.cast_r = remap(speed_ratio, 0.45, 1.0, self.cast_range_min, self.cast_range_max)

        # 2. Fix missing/invalid rays
        self.fix_all_missing()

        # 3. Detect towers
        self.detected_towers = self.find_towers()

        # Publish tower angles
        tower_msg = Float32MultiArray()
        tower_msg.data = [float(t["ang"]) for t in self.detected_towers]
        self.tower_pub.publish(tower_msg)

        # 4. Find best path (max safe distance with WRO color override)
        max_d, t_ang = self.get_max_d(self.detected_towers)

        # --- Minimum Safe Distance Floor ---
        if max_d < 0.15:
            self.target_dist = 0.0
            self.target_ang = 0.0
            self.speed = 0.0
            self.speed_boost = 1.0
            self.str_angle = 0.0
            self.publish_debug_scan()
            self.publish_target_marker()
            return

        # --- GAME MODE: Chase the Tower ---
        if self.chase_tower_mode and self.detected_towers:
            t_ang = self.detected_towers[0]["ang"]
            max_d = self.detected_towers[0]["dst"]

        # 5. Smooth target angle (LazyGo's dual lerp)
        delta = abs(t_ang - self.target_ang)
        if delta > 0.5:
            self.target_ang = t_ang
        else:
            self.target_ang = lerp(self.target_ang, t_ang, min(35 * dt, 1.0))
        self.target_ang = lerp(self.target_ang, t_ang, 0.1)
        self.target_dist = max_d

        # Convert to degrees for steering
        target_deg = math.degrees(self.target_ang)

        # 6. Emergency danger sense
        target_deg = self.danger_sense(target_deg)

        # 7. Speed boosting on clear straights
        self.speed_boost = 1.0
        boost_idx = self.a2i(math.radians(target_deg))
        if 0 <= boost_idx < n:
            marching_hit = self.marching(boost_idx, radius=self.cast_r / 2.0)
            hit_d = marching_hit["dst"]
            if abs(target_deg) < self.boost_angle_thresh and hit_d > self.boost_dist_thresh:
                self.speed_boost = self.boost_max

        # 8. Map target angle to steering range [-1, 1]
        s_ang = remap(target_deg, -self.str_ang_thresh, self.str_ang_thresh, -1.0, 1.0)
        self.str_angle = s_ang

        # 9. Calculate speed based on distance
        mult = remap(max_d, 1.0, 2.0, 0.65, 1.0)
        self.speed = self.max_speed * mult * self.speed_boost

        # 10. Dynamic speed cap (instant decel, smooth accel)
        self.target_cap = self.speed_cap_straight
        # Could add corner position detection here; for now use steering magnitude
        if abs(target_deg) > 40.0:
            self.target_cap = self.speed_cap_corner

        if self.target_cap < self.speed_cap:
            self.speed_cap = self.target_cap
        else:
            self.speed_cap = lerp(self.speed_cap, self.target_cap, min(dt * 5, 1.0))

        # 11. Throttled terminal feedback
        self.get_logger().info(
            f'[NAV] Target: {target_deg:+6.1f}° | '
            f'Dist: {max_d:4.2f}m | '
            f'CastR: {self.cast_r:.3f}m | '
            f'Boost: {self.speed_boost:.2f}x | '
            f'Towers: {len(self.detected_towers)} | '
            f'Color: {self.closest_color}',
            throttle_duration_sec=0.5
        )

        # 12. Publish debug scan (ranges after fix_missing)
        self.publish_debug_scan()

        # 13. Publish target arrow marker
        self.publish_target_marker()



    # ══════════════════════════════════════════════════════════════════
    # Visualization Publishers
    # ══════════════════════════════════════════════════════════════════

    def publish_debug_scan(self):
        """Publish the processed ranges as a LaserScan for RViz."""
        if not hasattr(self, 'scan_header'):
            return
        debug = LaserScan()
        debug.header.stamp = self.get_clock().now().to_msg()
        debug.header.frame_id = self.scan_header.frame_id if self.scan_header.frame_id else 'laser'
        debug.angle_min = self.ang_min
        debug.angle_max = self.ang_max
        debug.angle_increment = self.ang_inc
        debug.range_min = self.range_min
        debug.range_max = self.range_max
        debug.ranges = [float(r) for r in self.ranges]
        self.debug_scan_pub.publish(debug)

    def publish_target_marker(self):
        """Publish a 3D arrow marker pointing at the chosen target path."""
        if not hasattr(self, 'scan_header'):
            return
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.scan_header.frame_id if self.scan_header.frame_id else 'laser'
        marker.ns = "disparity_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = float(math.sin(self.target_ang / 2.0))
        marker.pose.orientation.w = float(math.cos(self.target_ang / 2.0))
        marker.scale.x = float(self.target_dist)
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtenderNode()
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
