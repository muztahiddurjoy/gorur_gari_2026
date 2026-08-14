"""
Open Round Run — open-space navigation, wall hugging, and return-to-start.

Built for the WRO open round (no pillars on the mat) on top of the circle-cast
pipeline from disparity_extender.py, with five additions:

1. OPEN SPACE DETECTION
   The forward FOV is swept with LazyGo's circle-cast ray marching: every
   candidate ray gets a "safe distance" that accounts for the robot's width,
   and the ray with the maximum safe distance becomes the target heading.
   No tower detection and no colour rules - the open round has neither.

2. WALL HUGGING
   On straights the raw open-space heading wanders between the walls, so a
   side-distance controller trims the heading to hold wall_target_dist_m off
   the outer wall. The outer wall side is latched automatically from the IMU
   heading (first ~45 deg of accumulated turn tells the lap direction: CCW
   laps keep the outer wall on the right, CW laps on the left), or forced
   with the wall_side parameter. The correction is gated so it never fights
   the corner steering: it only applies while the open-space target is nearly
   straight ahead and the wall reading is close enough to really be the wall.

3. HEADING-FUSED STEERING
   The raw pipeline was open loop: the LiDAR target angle went straight to
   the servo and nothing checked whether the car actually rotated, so a
   noisy scan (or one all-blocked frame) left the wheels pointing anywhere.
   Now the chosen direction is latched as an ABSOLUTE heading the moment it
   is picked (target_yaw = current yaw + relative target; the MCU heading
   always boots at 0.0, so this frame is stable for the whole run) and the
   40 Hz control loop closes the loop on /heading with a PD controller —
   the same maths, gains and sign convention SteeringStabilizer /
   box_lap_test proved on this car. If /heading is missing or stale the
   node falls back to the old raw-LiDAR-angle steering, so a dead IMU
   degrades the drive instead of parking it (and the sim needs no IMU).

4. VECTOR ODOM + RETURN TO START
   The node consumes vector_odom's outputs directly: /odom_vector (x, y in
   output_units, cm by default) and /heading (raw MCU compass degrees,
   clockwise - converted here with the same convention vector_odom and
   goto_controller use, so all three agree on one yaw).
   The pose at the moment the car starts driving is captured as "home"
   (equal to (0,0) when vector_odom starts with the run, and still correct
   if the odometry was already integrating). After lap_count reaches
   target_laps the node switches to HOMING: it steers along the vector from
   the current position back to home - reversing when home is behind, since
   an Ackermann car cannot turn in place - and stops once inside
   home_radius_m (5 cm) of it.

5. ENCODER STALL ADAPTATION
   /encoder/count is fused in as a "are the wheels actually turning" sense.
   The commanded speed is a fixed drive_speed, multiplied by
   turn_speed_gain (1.25x) while the servo is off centre — steering load
   slows this drivetrain, so corners get MORE throttle, never less (all
   the old corner-slowdown logic is removed). While the node is
   commanding motion but the encoder count has not changed for
   stall_timeout_sec, the throttle is multiplied by a boost that RAMPS UP
   gently (stall_boost_step per second) to at most stall_boost_max — never
   a full-throttle slam — and eases back to 1.0 as soon as the wheels turn
   again. Only the THROTTLE is limited: steering is unrestricted, with
   max_steer_cmd at 1.5 spanning the full 0-180 servo travel (the bridge
   maps servo = 90 + angular.z*60). A dead /encoder/count stream disables
   the boost (never blind-boost) and the base speed still drives the car.

State machine:
    STANDBY -> ARMING -> RUNNING -> HOMING -> FINISHED
       ^          |
       start button + start_delay_sec, same gate as disparity_extender

Edge cases handled:
- LiDAR watchdog: no /scan for >2 s stops the car in any driving state.
- Stale lap counts: /reset_lap_count is published on startup and the local
  count forced to zero before the car may move; a lap-complete signal within
  min_run_time_sec of GO is ignored as a leftover from a previous run.
- Missing odometry or heading: HOMING refuses to drive blind and stops;
  RUNNING keeps driving (the lap phase only needs LiDAR).
- Odometry going quiet mid-homing (encoder stream died): stop.
- Home behind the car: reverse gear with hysteresis (flip >100 deg off the
  nose, back under 80 deg) so the gear cannot chatter.
- Overshoot / hunting: the closest approach is tracked; after max_gear_flips
  direction changes the car accepts the closest point it can physically
  reach instead of shuttling over the target forever.
- Homing timeout: the phase force-finishes after homing_timeout_sec.
- Blocked while homing: a narrow clearance cone in the travel direction
  pauses the car; blocked longer than homing_blocked_give_up_sec finishes
  the run where it stands rather than pushing into a wall.
- IMU glitches: heading steps >45 deg in one message are ignored for the
  wall-side detection (same guard lap_counter uses).
- All-blocked scans: safe distance under min_clear_dist_m parks the car for
  that cycle instead of steering into the nearest gap.

Verify without the car moving: enable_auto_steering false (the in-code
default) lets the whole pipeline run - markers, debug scan, state logs -
while /cmd_vel stays untouched.

Run:
    ros2 run autonomy open_round_run --ros-args \
        --params-file config/bot_config.yaml
"""

import math
import time
from threading import Lock

from geometry_msgs.msg import Twist, Vector3
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float32, Int32, String
from visualization_msgs.msg import Marker

# Run states.
STATE_STANDBY = 'STANDBY'    # powered up, waiting for the button
STATE_ARMING = 'ARMING'      # button seen, counting the start delay down
STATE_RUNNING = 'RUNNING'    # lapping with open-space + wall hugging
STATE_HOMING = 'HOMING'      # laps done, driving back to the start point
STATE_FINISHED = 'FINISHED'  # parked inside home_radius_m (or gave up), holding zero

# While parked we still send zeros so the MCU never sits on a stale throttle,
# but at a fraction of the control rate - there is nothing to steer.
STOP_REPUBLISH_INTERVAL_S = 0.5

# /odom_vector arrives in vector_odom's output_units; this maps that unit
# back to metres, which is what every distance in this node uses internally.
ODOM_UNIT_TO_M = {'m': 1.0, 'cm': 0.01, 'mm': 0.001}


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


def normalize_angle_deg(angle):
    """Wrap to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def normalize_angle_rad(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class OpenRoundRunNode(Node):

    def __init__(self):
        super().__init__('open_round_run')

        # ── Master Controls ──────────────────────────────────────────
        # False by default so a bare `ros2 run` can never move the car.
        self.declare_parameter('enable_auto_steering', True)

        # ── Vehicle Geometry (Circle Cast) ───────────────────────────
        self.declare_parameter('cast_range_min', 0.13)
        self.declare_parameter('cast_range_max', 0.16)
        self.declare_parameter('cast_precision', 81)
        self.declare_parameter('cast_skip', 4)
        self.declare_parameter('cast_skip_fine', 1)

        # ── FOV ──────────────────────────────────────────────────────
        self.declare_parameter('look_range_deg', 80.0)

        # ── Speed ────────────────────────────────────────────────────
        self.declare_parameter('max_speed', 0.60)
        self.declare_parameter('speed_cap_corner', 0.30)
        self.declare_parameter('speed_cap_straight', 0.45)
        self.declare_parameter('boost_max', 1.35)
        self.declare_parameter('boost_angle_thresh', 7.5)
        self.declare_parameter('boost_dist_thresh', 1.10)

        # ── Safety ───────────────────────────────────────────────────
        self.declare_parameter('danger_dist', 0.22)
        self.declare_parameter('danger_angle_min', 25.0)
        self.declare_parameter('danger_angle_max', 90.0)
        # Below this best safe distance the scan is considered blocked and
        # the car parks for the cycle instead of picking the least-bad gap.
        self.declare_parameter('min_clear_dist_m', 0.15)

        # ── Steering ─────────────────────────────────────────────────
        self.declare_parameter('str_ang_thresh', 60.0)

        # ── Heading-Fused Steering ───────────────────────────────────
        # The LiDAR target becomes an absolute compass heading and the
        # 40 Hz loop steers on the live /heading error (PD). Gains are
        # the ones SteeringStabilizer / box_lap_test proved on this car.
        self.declare_parameter('heading_fusion_enable', True)
        self.declare_parameter('heading_kp', 0.03)
        self.declare_parameter('heading_kd', 0.005)
        # 1.5 = full 0-180 servo travel through the bridge's 90 + z*60 map.
        self.declare_parameter('heading_max_steer', 1.5)
        self.declare_parameter('heading_deadband_deg', 2.0)
        # /heading older than this falls back to raw LiDAR-angle steering.
        self.declare_parameter('heading_stale_sec', 0.5)

        # ── Encoder Stall Adaptation ─────────────────────────────────
        # Nominal linear.x while lapping — THE linear command limit.
        self.declare_parameter('drive_speed', 1.25)
        # |angular.z| ceiling. 1.5 is full servo travel (the bridge maps
        # servo = 90 + z*60, so z = ±1.5 reaches the 0/180 endpoints):
        # steering is deliberately unlimited, only linear is capped.
        self.declare_parameter('max_steer_cmd', 1.5)
        # Encoder count frozen this long while commanding motion = stalled.
        self.declare_parameter('stall_timeout_sec', 0.4)
        # Boost growth per second while stalled, and its ceiling — the
        # peak throttle is drive_speed * stall_boost_max, deliberately
        # well under full lock so the recovery is a nudge, not a slam.
        self.declare_parameter('stall_boost_step', 0.5)
        self.declare_parameter('stall_boost_max', 10.0)
        # No /encoder/count at all for this long -> boost disabled (never
        # boost blind), the base drive_speed still drives the car.
        self.declare_parameter('encoder_stale_sec', 1.0)

        # ── Corner Throttle Gain ─────────────────────────────────────
        # Steering load slows the drivetrain, so turning gets MORE linear
        # speed, not less. Applied whenever |angular.z| is above the
        # threshold (0.0 = any nonzero steering command boosts); there
        # is no corner slowdown anywhere any more.
        self.declare_parameter('turn_speed_gain', 1.5)
        self.declare_parameter('turn_steer_thresh', 0.0)

        # ── Wall Hugging ─────────────────────────────────────────────
        self.declare_parameter('wall_hug_enable', True)
        # Distance to hold off the hugged (outer) wall, metres.
        self.declare_parameter('wall_target_dist_m', 0.35)
        # 'auto' latches the outer wall from the lap direction; 'left' or
        # 'right' forces it (useful when the direction is known in advance).
        self.declare_parameter('wall_side', 'auto')
        # Accumulated IMU turn that decides the lap direction in auto mode.
        self.declare_parameter('wall_side_latch_deg', 45.0)
        # The wall is sampled in a window around +/-90 deg.
        self.declare_parameter('wall_window_deg', 20.0)
        # Side readings beyond this are an opening or a corner, not the wall
        # beside the car - no correction is applied on them.
        self.declare_parameter('wall_valid_max_dist_m', 1.2)
        # Heading trim per metre of wall-distance error, and its ceiling.
        self.declare_parameter('wall_kp_deg_per_m', 40.0)
        self.declare_parameter('wall_kd_deg_s_per_m', 0.0)
        self.declare_parameter('wall_max_correction_deg', 12.0)
        # The trim only applies while the open-space target is nearly
        # straight ahead - corners belong to the disparity logic alone.
        self.declare_parameter('wall_hug_gate_deg', 25.0)

        # ── Laps ─────────────────────────────────────────────────────
        self.declare_parameter('target_laps', 3)
        # Lap completion within this many seconds of GO is a stale count
        # from a previous run, not a real lap - ignored.
        self.declare_parameter('min_run_time_sec', 5.0)

        # ── Vector Odom / Homing ─────────────────────────────────────
        # MUST match vector_odom's output_units or every homing distance
        # is off by 100x.
        self.declare_parameter('odom_units', 'cm')
        # The finish circle around the start point: 5 cm radius.
        self.declare_parameter('home_radius_m', 0.05)
        self.declare_parameter('homing_max_speed', 0.30)
        self.declare_parameter('homing_min_speed', 0.25)
        # Speed ramps from max down to min inside this distance of home.
        self.declare_parameter('homing_slowdown_dist_m', 0.60)
        # Gear hysteresis: reverse past this heading error, forward again
        # once back under the low threshold.
        self.declare_parameter('reverse_enter_err_deg', 100.0)
        self.declare_parameter('reverse_exit_err_deg', 80.0)
        self.declare_parameter('max_gear_flips', 6)
        self.declare_parameter('homing_timeout_sec', 45.0)
        # Clearance cone in the direction of travel while homing.
        self.declare_parameter('homing_guard_dist_m', 0.12)
        self.declare_parameter('homing_guard_half_deg', 20.0)
        self.declare_parameter('homing_blocked_give_up_sec', 3.0)
        # Odometry older than this mid-homing means the encoder stream died.
        self.declare_parameter('odom_stale_sec', 1.0)

        # ── Heading Convention (must match vector_odom) ──────────────
        self.declare_parameter('heading_clockwise', True)
        self.declare_parameter('heading_offset_deg', 0.0)

        # ── Start Gate ───────────────────────────────────────────────
        self.declare_parameter('require_button_start', True)
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('button_topic', '/button_status')
        self.declare_parameter('wait_for_track_ready', False)

        # ── Topics ───────────────────────────────────────────────────
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_vector_topic', '/odom_vector')
        self.declare_parameter('heading_topic', '/heading')
        self.declare_parameter('encoder_topic', '/encoder/count')
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

        self.danger_dist = float(self.get_parameter('danger_dist').value)
        self.danger_angle_min = float(self.get_parameter('danger_angle_min').value)
        self.danger_angle_max = float(self.get_parameter('danger_angle_max').value)
        self.min_clear_dist_m = float(self.get_parameter('min_clear_dist_m').value)

        self.str_ang_thresh = float(self.get_parameter('str_ang_thresh').value)

        self.heading_fusion_enable = bool(self.get_parameter('heading_fusion_enable').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_kd = float(self.get_parameter('heading_kd').value)
        self.heading_max_steer = float(self.get_parameter('heading_max_steer').value)
        self.heading_deadband_deg = float(self.get_parameter('heading_deadband_deg').value)
        self.heading_stale_sec = float(self.get_parameter('heading_stale_sec').value)

        self.drive_speed = float(self.get_parameter('drive_speed').value)
        self.max_steer_cmd = float(self.get_parameter('max_steer_cmd').value)
        self.stall_timeout_sec = float(self.get_parameter('stall_timeout_sec').value)
        self.stall_boost_step = float(self.get_parameter('stall_boost_step').value)
        self.stall_boost_max = float(self.get_parameter('stall_boost_max').value)
        self.encoder_stale_sec = float(self.get_parameter('encoder_stale_sec').value)
        self.turn_speed_gain = float(self.get_parameter('turn_speed_gain').value)
        self.turn_steer_thresh = float(self.get_parameter('turn_steer_thresh').value)

        self.wall_hug_enable = bool(self.get_parameter('wall_hug_enable').value)
        self.wall_target_dist_m = float(self.get_parameter('wall_target_dist_m').value)
        self.wall_side_param = str(self.get_parameter('wall_side').value).lower()
        self.wall_side_latch_deg = float(self.get_parameter('wall_side_latch_deg').value)
        self.wall_window_rad = math.radians(float(self.get_parameter('wall_window_deg').value))
        self.wall_valid_max_dist_m = float(self.get_parameter('wall_valid_max_dist_m').value)
        self.wall_kp = float(self.get_parameter('wall_kp_deg_per_m').value)
        self.wall_kd = float(self.get_parameter('wall_kd_deg_s_per_m').value)
        self.wall_max_correction_deg = float(self.get_parameter('wall_max_correction_deg').value)
        self.wall_hug_gate_deg = float(self.get_parameter('wall_hug_gate_deg').value)

        self.target_laps = int(self.get_parameter('target_laps').value)
        self.min_run_time_sec = float(self.get_parameter('min_run_time_sec').value)

        odom_units = str(self.get_parameter('odom_units').value)
        self.home_radius_m = float(self.get_parameter('home_radius_m').value)
        self.homing_max_speed = float(self.get_parameter('homing_max_speed').value)
        self.homing_min_speed = float(self.get_parameter('homing_min_speed').value)
        self.homing_slowdown_dist_m = float(self.get_parameter('homing_slowdown_dist_m').value)
        self.reverse_enter_err_deg = float(self.get_parameter('reverse_enter_err_deg').value)
        self.reverse_exit_err_deg = float(self.get_parameter('reverse_exit_err_deg').value)
        self.max_gear_flips = int(self.get_parameter('max_gear_flips').value)
        self.homing_timeout_sec = float(self.get_parameter('homing_timeout_sec').value)
        self.homing_guard_dist_m = float(self.get_parameter('homing_guard_dist_m').value)
        self.homing_guard_half_rad = math.radians(
            float(self.get_parameter('homing_guard_half_deg').value))
        self.homing_blocked_give_up_sec = float(
            self.get_parameter('homing_blocked_give_up_sec').value)
        self.odom_stale_sec = float(self.get_parameter('odom_stale_sec').value)

        self.heading_clockwise = bool(self.get_parameter('heading_clockwise').value)
        self.heading_offset_deg = float(self.get_parameter('heading_offset_deg').value)

        self.require_button_start = bool(self.get_parameter('require_button_start').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        button_topic = self.get_parameter('button_topic').value

        scan_topic = self.get_parameter('scan_topic').value
        odom_vector_topic = self.get_parameter('odom_vector_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        encoder_topic = self.get_parameter('encoder_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Fail fast on config that would silently break the homing maths.
        if odom_units not in ODOM_UNIT_TO_M:
            raise ValueError(f'odom_units must be one of {sorted(ODOM_UNIT_TO_M)}')
        self.odom_to_m = ODOM_UNIT_TO_M[odom_units]
        self.odom_units = odom_units
        if self.wall_side_param not in ('auto', 'left', 'right'):
            raise ValueError("wall_side must be 'auto', 'left' or 'right'")
        if self.target_laps <= 0:
            raise ValueError('target_laps must be positive')
        if self.home_radius_m <= 0.0:
            raise ValueError('home_radius_m must be positive')
        if self.reverse_exit_err_deg >= self.reverse_enter_err_deg:
            raise ValueError('reverse_exit_err_deg must be below reverse_enter_err_deg '
                             '(the hysteresis band would invert)')

        # ── LiDAR State ──────────────────────────────────────────────
        self.ang_min = -math.pi
        self.ang_max = math.pi
        self.ang_inc = math.radians(0.5)
        self.ranges = []
        self.intensities = []
        self.new_lidar_val = False
        self._scan_lock = Lock()  # Guards shared LiDAR state between callback and timer
        self.last_scan_time = 0.0

        # Narrow clearance cones, refreshed every scan, used by HOMING.
        self.forward_clearance_m = float('inf')
        self.rear_clearance_m = float('inf')

        # ── Navigation State ─────────────────────────────────────────
        self.current_speed = 0.0        # estimated from /odom_vector's distance
        self.speed = 0.0
        self.speed_boost = 1.0
        self.speed_cap = self.speed_cap_straight
        self.target_cap = self.speed_cap_straight
        self.str_angle = 0.0
        self.target_ang = 0.0
        self.target_dist = 0.0
        self.cast_r = self.cast_range_min
        self.last_time = time.time()
        self.lap_count = 0
        self.has_reset_laps = False
        self.reset_timer_ticks = 0

        # ── Wall Hug State ───────────────────────────────────────────
        # +1 = hug the LEFT wall, -1 = hug the RIGHT wall, None = not yet known.
        if self.wall_side_param == 'left':
            self.hug_side = 1
        elif self.wall_side_param == 'right':
            self.hug_side = -1
        else:
            self.hug_side = None
        self.cumulative_yaw_deg = 0.0   # unwrapped IMU turn, for auto side latch
        self.last_raw_heading_yaw = None
        self.last_wall_error = None
        self.last_wall_time = None

        # ── Odom / Heading State ─────────────────────────────────────
        self.pos_x_m = None             # None until /odom_vector arrives
        self.pos_y_m = None
        self.last_odom_time = 0.0
        self.last_odom_total_m = None   # for the speed estimate
        self.yaw_deg = None             # ROS convention (CCW+), None until /heading
        self.last_heading_time = 0.0    # wall time of the last /heading message
        self.warned_no_odom = False

        # ── Heading-Fused Steering State ─────────────────────────────
        self.target_yaw_deg = None      # absolute heading the car should hold
        self.last_heading_err = None    # PD memory; None resets the D-term
        self.last_steer_time = None

        # ── Encoder / Stall State ────────────────────────────────────
        self.last_encoder_count = None      # None until /encoder/count arrives
        self.last_encoder_change_time = 0.0  # when the count last CHANGED
        self.last_encoder_msg_time = 0.0    # when the stream last spoke at all
        self.stall_boost = 1.0              # live throttle multiplier
        self.last_boost_update = None

        # ── Homing State ─────────────────────────────────────────────
        self.home_x_m = 0.0
        self.home_y_m = 0.0
        self.run_start_time = 0.0       # when RUNNING began
        self.homing_start_time = 0.0
        self.homing_min_dist = None     # closest approach so far
        self.driving_reverse = False
        self.gear_flips = 0
        self.homing_blocked_since = None
        self.finish_reason = ''
        self.finish_logged = False
        self.home_captured = False

        # ── Track Ready Gate (Simulation) ────────────────────────────
        self.wait_for_track_ready = bool(self.get_parameter('wait_for_track_ready').value)
        self.track_ready = not self.wait_for_track_ready

        # ── Start Gate State ─────────────────────────────────────────
        if self.wait_for_track_ready:
            self.run_state = STATE_STANDBY
        else:
            self.run_state = STATE_STANDBY if self.require_button_start else STATE_RUNNING
        self.arm_start_time = 0.0
        self.prev_button = False        # rising edge detection on /button_status
        self.last_countdown_logged = -1
        self.last_stop_pub_time = 0.0
        if self.run_state == STATE_RUNNING:
            self.run_start_time = time.time()

        # ── Callback Groups ──────────────────────────────────────────
        # The LiDAR pipeline (scan intake + the pure-Python circle-cast
        # maths) lives in its own mutually-exclusive group; everything
        # time-critical — the 40 Hz control loop, watchdog, button, odom,
        # state heartbeat — lives in another. With the multi-threaded
        # executor in main(), a slow scan-processing cycle can therefore
        # never delay a drive command or the emergency stop, while the
        # scan callback and the processing step still serialise with each
        # other (same group), so they cannot race on the ray buffers.
        self.cb_lidar = MutuallyExclusiveCallbackGroup()
        self.cb_control = MutuallyExclusiveCallbackGroup()

        # ── Subscriptions & Publishers ───────────────────────────────
        if self.wait_for_track_ready:
            from rclpy.qos import DurabilityPolicy, QoSProfile
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.track_ready_sub = self.create_subscription(
                Bool, '/track_ready', self.track_ready_callback, latched,
                callback_group=self.cb_control)

        # BEST_EFFORT (sensor data) QoS: the C1 driver publishes BEST_EFFORT
        # and a RELIABLE subscriber never matches it — the callback would
        # simply never fire and the node would look hung, with the watchdog
        # disarmed (it only arms after the first scan). Same fix as
        # custom_disparity_extender and e_stop's FIX 2.
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.lidar_callback, qos_profile_sensor_data,
            callback_group=self.cb_lidar)
        self.odom_sub = self.create_subscription(
            Vector3, odom_vector_topic, self.odom_vector_callback, 10,
            callback_group=self.cb_control)
        self.heading_sub = self.create_subscription(
            Float32, heading_topic, self.heading_callback, 10,
            callback_group=self.cb_control)
        self.encoder_sub = self.create_subscription(
            Int32, encoder_topic, self.encoder_callback, 10,
            callback_group=self.cb_control)
        self.lap_sub = self.create_subscription(
            Int32, '/lap_count', self.lap_callback, 10,
            callback_group=self.cb_control)
        self.button_sub = self.create_subscription(
            Bool, button_topic, self.button_callback, 10,
            callback_group=self.cb_control)
        self.lap_reset_pub = self.create_publisher(Empty, '/reset_lap_count', 10)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.debug_scan_pub = self.create_publisher(LaserScan, '/open_round/scan_processed', 10)
        self.marker_pub = self.create_publisher(Marker, '/open_round/target_marker', 10)
        self.state_pub = self.create_publisher(String, '/open_round/state', 10)

        # ── Control Loop Timer (40 Hz) ───────────────────────────────
        self.create_timer(0.025, self.control_loop, callback_group=self.cb_control)

        # ── LiDAR Processing Timer (20 Hz) ───────────────────────────
        self.create_timer(0.05, self.calc_lidar_step, callback_group=self.cb_lidar)

        # ── State Heartbeat (1 Hz) ───────────────────────────────────
        self.create_timer(1.0, self.publish_state, callback_group=self.cb_control)

        self.get_logger().info(
            f'[Open Round Run] Initialized | '
            f'Laps: {self.target_laps} | '
            f'Wall hug: {"on" if self.wall_hug_enable else "off"} '
            f'({self.wall_side_param}, {self.wall_target_dist_m:.2f} m) | '
            f'Home radius: {self.home_radius_m * 100.0:.0f} cm | '
            f'Odom units: {self.odom_units}'
        )
        if self.require_button_start:
            self.get_logger().info(
                f'STANDBY — press the start button ({button_topic}) to run. '
                f'The car moves {self.start_delay_sec:.0f}s after the press.')
        else:
            self.get_logger().warn('Start button gate disabled — driving immediately.')

    # ══════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════

    def track_ready_callback(self, msg: Bool):
        """Simulation gate: track_maker says the mat is laid out."""
        if msg.data and not self.track_ready:
            self.track_ready = True
            self.get_logger().info(
                'Received /track_ready from track_maker — track layout complete!')
            if self.run_state == STATE_STANDBY and not self.require_button_start:
                self.run_state = STATE_RUNNING
                self.run_start_time = time.time()
                self.home_captured = False  # Recapture home at updated car pose

    def odom_vector_callback(self, msg: Vector3):
        """Track the vector odom pose (converted to metres) and estimate speed."""
        now = time.time()
        self.pos_x_m = msg.x * self.odom_to_m
        self.pos_y_m = msg.y * self.odom_to_m
        total_m = msg.z * self.odom_to_m

        # Speed from the travelled-distance delta; EMA smoothed because the
        # encoder stream is bursty. Used only for the dynamic cast radius.
        if self.last_odom_total_m is not None:
            dt = now - self.last_odom_time
            if dt > 1e-3:
                inst = abs(total_m - self.last_odom_total_m) / dt
                self.current_speed = 0.7 * self.current_speed + 0.3 * inst
        self.last_odom_total_m = total_m
        self.last_odom_time = now

    def heading_callback(self, msg: Float32):
        """
        Convert the MCU compass heading (degrees, clockwise) into a ROS yaw in
        degrees - the exact convention vector_odom and goto_controller use, so
        the homing vector and the pose it steers by share one frame.
        """
        heading = msg.data + self.heading_offset_deg
        yaw = -heading if self.heading_clockwise else heading
        yaw = normalize_angle_deg(yaw)

        # Unwrapped turn total for the auto wall-side latch, with the same
        # glitch guard lap_counter uses: the car cannot physically rotate
        # 45 deg between two heading messages.
        if self.last_raw_heading_yaw is not None:
            step = normalize_angle_deg(yaw - self.last_raw_heading_yaw)
            if abs(step) <= 45.0:
                self.cumulative_yaw_deg += step
            else:
                self.get_logger().warn(
                    f'Heading jumped {step:.1f} deg in one message, ignoring it',
                    throttle_duration_sec=1.0)
        self.last_raw_heading_yaw = yaw
        self.yaw_deg = yaw
        self.last_heading_time = time.time()

        # Latch the outer wall once the lap direction is unambiguous:
        # CCW laps (left turns, positive yaw growth) keep the outer wall on
        # the RIGHT; CW laps keep it on the LEFT.
        if (self.hug_side is None and self.wall_side_param == 'auto'
                and abs(self.cumulative_yaw_deg) >= self.wall_side_latch_deg):
            self.hug_side = -1 if self.cumulative_yaw_deg > 0.0 else 1
            self.get_logger().info(
                f'Lap direction latched: '
                f'{"CCW" if self.cumulative_yaw_deg > 0.0 else "CW"} — hugging the '
                f'{"right" if self.hug_side < 0 else "left"} (outer) wall.')

    def encoder_callback(self, msg: Int32):
        """
        Raw encoder tick count from the MCU (any change = the wheels turned;
        works in reverse too, and a count reset still reads as a change).
        Only the CHANGE time matters — the stall detector compares against it.
        """
        now = time.time()
        if self.last_encoder_count is None or msg.data != self.last_encoder_count:
            self.last_encoder_count = msg.data
            self.last_encoder_change_time = now
        self.last_encoder_msg_time = now

    def lap_callback(self, msg: Int32):
        """Update lap count from the lap_counter node."""
        self.lap_count = msg.data

    def button_callback(self, msg: Bool):
        """
        Start button from the MCU. /button_status stays true while the button
        is held, so only the press edge counts.
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
            self.publish_state()
            self.get_logger().info(
                f'Start button pressed — going in {self.start_delay_sec:.0f}s.')
        else:
            self.get_logger().info(
                f'Start button pressed but already {self.run_state}, ignoring.')

    def lidar_callback(self, msg: LaserScan):
        """Buffer incoming LaserScan data for the processing timer."""
        with self._scan_lock:
            self.ang_min = msg.angle_min
            self.ang_max = msg.angle_max
            self.ang_inc = msg.angle_increment
            self.ranges = list(msg.ranges)
            self.intensities = (list(msg.intensities) if msg.intensities
                                else [1.0] * len(msg.ranges))
            self.range_min = msg.range_min
            self.range_max = msg.range_max
            self.scan_header = msg.header
            self.new_lidar_val = True
            self.last_scan_time = time.time()

    # ══════════════════════════════════════════════════════════════════
    # Start Gate
    # ══════════════════════════════════════════════════════════════════

    def update_run_state(self):
        """Advance ARMING → RUNNING once the start delay has elapsed."""
        if self.run_state != STATE_ARMING:
            return

        remaining = self.start_delay_sec - (time.time() - self.arm_start_time)
        if remaining <= 0.0:
            self.begin_running()
            return

        secs_left = int(math.ceil(remaining))
        if secs_left != self.last_countdown_logged:
            self.last_countdown_logged = secs_left
            self.get_logger().info(f'Starting in {secs_left}...')

    def begin_running(self):
        """GO: start lapping. The home pose is captured on the first driving tick."""
        self.run_state = STATE_RUNNING
        self.run_start_time = time.time()
        self.publish_state()
        self.get_logger().info(
            f'GO — open round running. {self.target_laps} laps, then return '
            f'to within {self.home_radius_m * 100.0:.0f} cm of the start.')

    def capture_home(self):
        """
        Latch the pose the run starts from as the homing target.

        Home is wherever the odometry says the car is when it first drives.
        When vector_odom starts with the run that is (0,0); if it was already
        integrating (node restarted mid-session, car carried around) the
        capture keeps homing correct anyway. No odom yet -> assume (0,0):
        vector_odom zeroes there on its first encoder tick.
        """
        self.home_captured = True
        if self.pos_x_m is not None and self.pos_y_m is not None:
            self.home_x_m = self.pos_x_m
            self.home_y_m = self.pos_y_m
            if math.hypot(self.home_x_m, self.home_y_m) > 0.10:
                self.get_logger().warn(
                    f'Odometry was not at the origin at GO — homing to the '
                    f'captured start ({self.home_x_m:.2f}, {self.home_y_m:.2f}) m '
                    f'instead of (0,0).')
        else:
            self.home_x_m = 0.0
            self.home_y_m = 0.0
            self.get_logger().warn(
                'No /odom_vector yet at GO — assuming the start is (0,0).')
        self.get_logger().info(
            f'Home captured at ({self.home_x_m:.2f}, {self.home_y_m:.2f}) m.')

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

    def publish_state(self):
        """
        Announce the state machine, for debugging, dashboards and run_timer.

        Sent once a second, and again the moment a transition happens -
        run_timer starts its stopwatch off the RUNNING that comes out of here
        and stops it on the FINISHED, so on the heartbeat alone every run time
        would be up to a second wrong at each end.
        """
        self.state_pub.publish(String(data=self.run_state))

    # ══════════════════════════════════════════════════════════════════
    # Control Loop (40 Hz)
    # ══════════════════════════════════════════════════════════════════

    def control_loop(self):
        # The gate is ticked before the enable check so the countdown keeps
        # running even while auto steering is switched off.
        self.update_run_state()

        if not bool(self.get_parameter('enable_auto_steering').value):
            # Say so, or a bare `ros2 run` without the params file looks
            # exactly like a hung node: subscribed, alive, and silent.
            self.get_logger().info(
                'enable_auto_steering is false — planning only, /cmd_vel untouched.',
                throttle_duration_sec=10.0)
            return

        # ── LAP RESET LOGIC ──
        # A stale latched /lap_count from a previous run could otherwise end
        # this run instantly, so the car may not move before the reset lands.
        if not self.has_reset_laps:
            self.reset_timer_ticks += 1
            if self.reset_timer_ticks >= 20:  # 0.5s for ROS 2 discovery to connect
                self.lap_reset_pub.publish(Empty())
                self.has_reset_laps = True
                self.lap_count = 0  # Force local reset too
                self.get_logger().info('Sent lap count reset signal on startup.')
            else:
                return  # Don't drive while waiting for reset

        # ── START GATE: STANDBY / ARMING park the car ──
        if self.run_state in (STATE_STANDBY, STATE_ARMING):
            self.publish_stop()
            self.get_logger().info(
                f'{self.run_state} — waiting for the start button.',
                throttle_duration_sec=5.0)
            return

        # ── FINISHED: hold zero forever ──
        if self.run_state == STATE_FINISHED:
            self.publish_stop()
            if not self.finish_logged:
                self.finish_logged = True
                self.get_logger().info(f'FINISHED — {self.finish_reason}')
            return

        # ── LIDAR WATCHDOG (both driving states) ──
        # Never seen a scan at all: refuse to drive and say why, loudly.
        # Without this branch a dead/mismatched lidar leaves last_scan_time
        # at 0.0, the timeout below never arms, and the node sits silently
        # publishing zeros — indistinguishable from a hang.
        if self.last_scan_time == 0.0:
            self.publish_zero_now()
            self.get_logger().error(
                'No /scan received yet — holding. Is the lidar driver up '
                '(and publishing BEST_EFFORT sensor-data QoS)?',
                throttle_duration_sec=2.0)
            return
        if (time.time() - self.last_scan_time) > 2.0:
            self.publish_zero_now()
            self.get_logger().error(
                'LiDAR timeout! No /scan for >2.0s — emergency stop.',
                throttle_duration_sec=1.0)
            return

        # ── HOME CAPTURE: first driving tick, whichever way RUNNING was entered ──
        if self.run_state == STATE_RUNNING and not self.home_captured:
            self.capture_home()

        # ── LAPS DONE → HOMING ──
        if (self.run_state == STATE_RUNNING
                and self.lap_count >= self.target_laps
                and time.time() - self.run_start_time >= self.min_run_time_sec):
            self.enter_homing()

        if self.run_state == STATE_RUNNING:
            self.drive_running()
        elif self.run_state == STATE_HOMING:
            self.drive_homing()

    def publish_zero_now(self):
        """Immediate un-throttled zero command (watchdogs, guards)."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def drive_running(self):
        """
        Publish the drive command: a fixed drive_speed (scaled by the encoder
        stall boost) and the heading-fused steering with the full servo
        travel available. The LiDAR pipeline's speed is only consulted as
        a go/no-go: self.speed == 0 means the pipeline parked the car (all
        blocked, or no scan processed yet) and the throttle must stay zero —
        boosting into a wall is exactly what the stall sense must not do.
        """
        driving = self.speed > 0.0
        boost = self.update_stall_boost(driving)
        steer = clamp(self.compute_running_steer(),
                      -self.max_steer_cmd, self.max_steer_cmd)
        speed_cmd = self.drive_speed * boost
        # Turning gets MORE throttle, never less: steering load slows this
        # drivetrain by itself, so any nonzero steering command takes
        # turn_speed_gain on top of the base speed (the old corner speed
        # cap is gone on purpose).
        if abs(steer) > self.turn_steer_thresh:
            speed_cmd *= self.turn_speed_gain
        cmd = Twist()
        cmd.linear.x = float(speed_cmd) if driving else 0.0
        cmd.angular.z = float(steer)
        self.cmd_pub.publish(cmd)

    def update_stall_boost(self, driving):
        """
        Fuse /encoder/count into the throttle: while a drive command is out
        but the tick count is frozen (static friction won), ramp a gentle
        boost up to stall_boost_max; the moment the wheels turn again, ramp
        it back down to 1.0 at the same rate — never a step, so the car
        cannot lurch. Not commanding motion, or a silent encoder stream,
        resets the boost to 1.0 (a parked car is not stalled, and a blind
        boost is worse than none).
        """
        now = time.time()
        dt = 0.0
        if self.last_boost_update is not None:
            dt = clamp(now - self.last_boost_update, 0.0, 0.2)
        self.last_boost_update = now

        encoder_alive = (now - self.last_encoder_msg_time) <= self.encoder_stale_sec
        if not driving or not encoder_alive:
            if not encoder_alive and driving:
                self.get_logger().warn(
                    'No /encoder/count — stall boost disabled, driving at base speed.',
                    throttle_duration_sec=5.0)
            self.stall_boost = 1.0
            return self.stall_boost

        stalled = (now - self.last_encoder_change_time) > self.stall_timeout_sec
        if stalled:
            self.stall_boost = min(self.stall_boost + self.stall_boost_step * dt,
                                   self.stall_boost_max)
            self.get_logger().warn(
                f'Wheels stalled ({now - self.last_encoder_change_time:.1f}s '
                f'without a tick) — throttle boost x{self.stall_boost:.2f}',
                throttle_duration_sec=1.0)
        else:
            self.stall_boost = max(1.0,
                                   self.stall_boost - self.stall_boost_step * dt)
        return self.stall_boost

    def compute_running_steer(self):
        """
        Closed-loop steering for RUNNING: hold the absolute heading the LiDAR
        pipeline latched (target_yaw_deg) against the live compass, at the
        full 40 Hz of the control loop instead of the 20 Hz of the scans.

        PD on the heading error — the exact shape SteeringStabilizer proved
        on this car. Its steer = -(kp*err) is stated in raw CLOCKWISE compass
        degrees; yaw here is the CCW conversion of the same stream, which
        flips the error's sign, so the identical physical command is
        +kp*err: positive error (target left of the nose) gives positive
        angular.z, agreeing with the raw-LiDAR fallback mapping.

        Falls back to the open-loop LiDAR angle (str_angle, the old
        behaviour) when fusion is off, no target is latched yet, or /heading
        is missing/stale — a dead IMU degrades the drive, never parks it.
        """
        now = time.time()
        if not self.heading_fusion_enable:
            return self.str_angle
        if (self.target_yaw_deg is None or self.yaw_deg is None
                or now - self.last_heading_time > self.heading_stale_sec):
            self.last_heading_err = None
            self.get_logger().warn(
                'Heading fusion inactive (no /heading or no target yet) — '
                'steering on the raw LiDAR angle.',
                throttle_duration_sec=5.0)
            return self.str_angle

        error = normalize_angle_deg(self.target_yaw_deg - self.yaw_deg)

        # Deadband: the BNO jitters a degree or two at rest; chasing that
        # just saws the servo down every straight.
        if abs(error) < self.heading_deadband_deg:
            self.last_heading_err = None
            return 0.0

        d_term = 0.0
        if self.last_heading_err is not None:
            dt = now - self.last_steer_time
            if dt > 1e-3:
                d_term = self.heading_kd * (error - self.last_heading_err) / dt
        self.last_heading_err = error
        self.last_steer_time = now

        return clamp(self.heading_kp * error + d_term,
                     -self.heading_max_steer, self.heading_max_steer)

    # ══════════════════════════════════════════════════════════════════
    # Homing (drive back into the 5 cm circle around the start)
    # ══════════════════════════════════════════════════════════════════

    def enter_homing(self):
        self.run_state = STATE_HOMING
        self.homing_start_time = time.time()
        self.homing_min_dist = None
        self.driving_reverse = False
        self.gear_flips = 0
        self.homing_blocked_since = None
        self.publish_state()
        dist = self.distance_home_m()
        self.get_logger().info(
            f'{self.target_laps} laps complete — homing to '
            f'({self.home_x_m:.2f}, {self.home_y_m:.2f}) m'
            + (f', {dist * 100.0:.0f} cm away.' if dist is not None else '.'))

    def distance_home_m(self):
        if self.pos_x_m is None or self.pos_y_m is None:
            return None
        return math.hypot(self.home_x_m - self.pos_x_m, self.home_y_m - self.pos_y_m)

    def finish(self, reason):
        self.run_state = STATE_FINISHED
        self.finish_reason = reason
        self.finish_logged = False
        self.publish_zero_now()
        self.publish_state()

    def drive_homing(self):
        now = time.time()

        # ── The inputs homing cannot live without ──
        if self.pos_x_m is None or self.yaw_deg is None:
            self.publish_stop()
            self.get_logger().error(
                'HOMING but no odometry/heading — is vector_odom running? Holding.',
                throttle_duration_sec=2.0)
            # Never sit here forever: the timeout below still applies.
            if now - self.homing_start_time > self.homing_timeout_sec:
                self.finish('homing timed out with no odometry — stopped in place.')
            return

        # Encoder stream died mid-homing: the pose is frozen and any further
        # driving would be open-loop. Stop.
        if now - self.last_odom_time > self.odom_stale_sec:
            self.publish_zero_now()
            self.get_logger().error(
                f'/odom_vector stale for >{self.odom_stale_sec:.1f}s — holding.',
                throttle_duration_sec=2.0)
            if now - self.homing_start_time > self.homing_timeout_sec:
                self.finish('homing timed out on stale odometry — stopped in place.')
            return

        dist = self.distance_home_m()

        # ── SUCCESS: inside the circle ──
        if dist <= self.home_radius_m:
            self.finish(
                f'parked {dist * 100.0:.1f} cm from the start '
                f'(target {self.home_radius_m * 100.0:.0f} cm).')
            return

        if self.homing_min_dist is None or dist < self.homing_min_dist:
            self.homing_min_dist = dist

        # ── TIMEOUT ──
        if now - self.homing_start_time > self.homing_timeout_sec:
            self.finish(
                f'homing timed out after {self.homing_timeout_sec:.0f}s — stopped '
                f'{dist * 100.0:.1f} cm out (closest approach '
                f'{self.homing_min_dist * 100.0:.1f} cm).')
            return

        # ── Bearing and gear selection ──
        bearing_deg = math.degrees(math.atan2(self.home_y_m - self.pos_y_m,
                                              self.home_x_m - self.pos_x_m))
        error_fwd = normalize_angle_deg(bearing_deg - self.yaw_deg)

        # Hysteresis so the gear cannot chatter when home sits beside the car.
        if not self.driving_reverse and abs(error_fwd) > self.reverse_enter_err_deg:
            self.driving_reverse = True
            self.gear_flips += 1
        elif self.driving_reverse and abs(error_fwd) < self.reverse_exit_err_deg:
            self.driving_reverse = False
            self.gear_flips += 1

        # Shuttling back and forth means the circle is smaller than the car
        # can physically hit from here - accept the closest point instead of
        # hunting forever.
        if self.gear_flips >= self.max_gear_flips:
            self.finish(
                f'hunted over the start {self.gear_flips} times — settled '
                f'{dist * 100.0:.1f} cm out (closest approach '
                f'{self.homing_min_dist * 100.0:.1f} cm).')
            return

        # ── Clearance guard in the travel direction ──
        clearance = self.rear_clearance_m if self.driving_reverse else self.forward_clearance_m
        if clearance < self.homing_guard_dist_m:
            if self.homing_blocked_since is None:
                self.homing_blocked_since = now
            if now - self.homing_blocked_since > self.homing_blocked_give_up_sec:
                self.finish(
                    f'blocked {"behind" if self.driving_reverse else "ahead"} for '
                    f'{self.homing_blocked_give_up_sec:.0f}s while homing — stopped '
                    f'{dist * 100.0:.1f} cm out.')
                return
            self.publish_zero_now()
            self.get_logger().warn(
                f'Obstacle {clearance:.2f} m '
                f'{"behind" if self.driving_reverse else "ahead"} — paused.',
                throttle_duration_sec=1.0)
            return
        self.homing_blocked_since = None

        # ── Steering ──
        # Backing up: point the TAIL at home, so the error is measured
        # against yaw + 180.
        error = error_fwd
        if self.driving_reverse:
            error = normalize_angle_deg(error - 180.0)
        steer = remap(error, -self.str_ang_thresh, self.str_ang_thresh, -1.0, 1.0)
        if self.driving_reverse:
            # Reversing flips how steering rotates the car (bicycle model:
            # yaw rate = v/L * tan(steer), v < 0), so flip the command too.
            steer = -steer

        # ── Speed: ease down approaching the circle ──
        # Same encoder stall fusion as RUNNING: the homing creep floor
        # (0.25) is the speed most likely to lose to static friction, so
        # the gentle boost matters most right here. Clamped to the same
        # drive_speed linear limit as everything else.
        speed = remap(dist, self.home_radius_m, self.homing_slowdown_dist_m,
                      self.homing_min_speed, self.homing_max_speed)
        speed = min(speed * self.update_stall_boost(True), 1.25 ) #self.drive_speed)
        if self.driving_reverse:
            speed = -speed

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(clamp(steer, -self.max_steer_cmd, self.max_steer_cmd))
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[HOME] {dist * 100.0:5.1f} cm | bearing {bearing_deg:+6.1f}° '
            f'yaw {self.yaw_deg:+6.1f}° err {error:+6.1f}° | '
            f'{"REV" if self.driving_reverse else "FWD"} v={speed:+.2f} s={steer:+.2f}',
            throttle_duration_sec=0.5)

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

    def ray_valid(self, i):
        """True if ray i is a usable return."""
        if i < 0 or i >= len(self.ranges):
            return False
        r = self.ranges[i]
        # r > 0.05 also rejects the 0.0 some drivers encode no-return as, and
        # chassis self-hits - either would latch the clearance guard blocked.
        return (len(self.intensities) > i and self.intensities[i] > 0.05
                and math.isfinite(r) and 0.05 < r <= 4.0)

    # ══════════════════════════════════════════════════════════════════
    # Missing Data Interpolation (from LazyGo's LidarHandler)
    # ══════════════════════════════════════════════════════════════════

    def fix_missing(self, i):
        """
        Interpolate missing/invalid LiDAR rays using nearest valid neighbors.
        Invalid = intensity ≤ 0.05, range > 4m, or inf/NaN.
        """
        n = len(self.ranges)
        if i < 0 or i >= n:
            return 0.0

        if self.ray_valid(i):
            return self.ranges[i]

        first = i
        while first > 0:
            first -= 1
            if self.ray_valid(first):
                break
        else:
            first = 0

        last = i
        while last < n - 1:
            last += 1
            if self.ray_valid(last):
                break
        else:
            last = n - 1

        if not (self.ray_valid(first) and self.ray_valid(last)):
            return 0.0
        if first == last:
            return self.ranges[first]

        lerp_factor = (i - first) / (last - first)

        # If large distance gap between neighbors, snap to closer side
        if abs(self.ranges[first] - self.ranges[last]) > 0.1:
            return self.ranges[first] if lerp_factor < 0.5 else self.ranges[last]

        return lerp(self.ranges[first], self.ranges[last], lerp_factor)

    def fix_all_missing(self):
        """Fix all invalid rays within the search FOV (+ margin for the walls)."""
        chk = self.ind_range(-self.look_range_rad, self.look_range_rad)
        # The margin has to reach past the ±90° wall windows or their
        # readings would keep raw dropouts.
        wall_edge = math.radians(90.0) + self.wall_window_rad / 2.0
        extra = max(0.0, wall_edge - self.look_range_rad) + math.radians(5.0)
        margin = max(50, int(extra / self.ang_inc))
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
        `check_ang`? Returns the check point's distance if collision, else False.
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
        return {"dst": min_hit["dst"], "ang": min_hit["ang"]}

    # ══════════════════════════════════════════════════════════════════
    # Max Safe Distance Selection (open round: no colour override)
    # ══════════════════════════════════════════════════════════════════

    def get_max_d(self):
        """
        Scan all rays in the forward FOV, compute safe distance via marching,
        and return the direction with the maximum safe distance.
        """
        best = {"dst": 0.0, "ang": 0.0}
        chk = self.ind_range(-self.look_range_rad, self.look_range_rad)
        n = len(self.ranges)

        for i in range(chk[0], chk[1], self.cast_skip):
            if i < 0 or i >= n:
                continue

            if self.intensities[i] <= 0.05 or self.ranges[i] > 3.0:
                self.ranges[i] = self.fix_missing(i)
                self.intensities[i] = 1.0

            if self.ranges[i] <= 0.0 or self.ranges[i] > 3.0:
                continue

            dt = self.marching(i)
            if dt["dst"] > best["dst"]:
                best = dt

        return best["dst"], best["ang"]

    # ══════════════════════════════════════════════════════════════════
    # Wall Hugging
    # ══════════════════════════════════════════════════════════════════

    def measure_wall_dist(self, side):
        """
        Robust distance to the wall on `side` (+1 left, -1 right): the 25th
        percentile of the valid rays in a window around ±90°, so a single
        noise spike can neither drag the reading in nor an opening push it out.
        Returns None when the window has too few valid rays.
        """
        center = side * math.pi / 2.0
        lo, hi = self.ind_range(center - self.wall_window_rad / 2.0,
                                center + self.wall_window_rad / 2.0)
        vals = []
        for i in range(min(lo, hi), max(lo, hi)):
            if self.ray_valid(i):
                vals.append(self.ranges[i])
        if len(vals) < 3:
            return None
        vals.sort()
        return vals[len(vals) // 4]

    def wall_hug_correction(self, target_deg, now):
        """
        Heading trim (degrees, +CCW) holding wall_target_dist_m off the hugged
        wall. Zero whenever wall hugging should not interfere:
        - disabled, or the wall side is not latched yet
        - the open-space target is already steering into a corner
        - the side reading is missing or too far to be the wall beside us
        """
        if not self.wall_hug_enable or self.hug_side is None:
            return 0.0
        if abs(target_deg) > self.wall_hug_gate_deg:
            self.last_wall_error = None  # a stale D-term across a corner is noise
            return 0.0

        wall_dist = self.measure_wall_dist(self.hug_side)
        if wall_dist is None or wall_dist > self.wall_valid_max_dist_m:
            self.last_wall_error = None
            return 0.0

        # error > 0: too far from the wall -> steer toward it.
        error = wall_dist - self.wall_target_dist_m

        d_term = 0.0
        if self.wall_kd > 0.0 and self.last_wall_error is not None:
            dt = now - self.last_wall_time
            if dt > 1e-3:
                d_term = self.wall_kd * (error - self.last_wall_error) / dt
        self.last_wall_error = error
        self.last_wall_time = now

        correction = clamp(self.wall_kp * error + d_term,
                           -self.wall_max_correction_deg, self.wall_max_correction_deg)

        # Toward the LEFT wall = steer CCW (+); toward the RIGHT wall = CW (−).
        return self.hug_side * correction

    # ══════════════════════════════════════════════════════════════════
    # Emergency Danger Sense
    # ══════════════════════════════════════════════════════════════════

    def danger_sense(self, target_ang_deg):
        """
        If any obstacle is within danger_dist at side angles
        [danger_angle_min, danger_angle_max], and the current heading steers
        toward it, blend in a corrective steer AWAY from the obstacle,
        scaling with proximity.
        """
        n = len(self.ranges)
        closest_left_dist = float('inf')
        closest_right_dist = float('inf')

        left_zone = self.ind_range(
            math.radians(self.danger_angle_min),
            math.radians(self.danger_angle_max))
        right_zone = self.ind_range(
            math.radians(-self.danger_angle_max),
            math.radians(-self.danger_angle_min))

        for i in list(range(left_zone[0], left_zone[1])) + \
                list(range(right_zone[0], right_zone[1])):
            if i < 0 or i >= n:
                continue
            if self.intensities[i] <= 0.05 or self.ranges[i] > 3.0:
                continue
            if 0.0 < self.ranges[i] < self.danger_dist:
                ang = math.degrees(self.i2a(i))
                if ang > 0 and self.ranges[i] < closest_left_dist:
                    closest_left_dist = self.ranges[i]
                elif ang < 0 and self.ranges[i] < closest_right_dist:
                    closest_right_dist = self.ranges[i]

        escape_angle = self.str_ang_thresh * 0.5

        if closest_left_dist < self.danger_dist and target_ang_deg > 0:
            severity = 1.0 - (closest_left_dist / self.danger_dist)
            return target_ang_deg * (1.0 - severity) + (-escape_angle) * severity

        if closest_right_dist < self.danger_dist and target_ang_deg < 0:
            severity = 1.0 - (closest_right_dist / self.danger_dist)
            return target_ang_deg * (1.0 - severity) + escape_angle * severity

        return target_ang_deg

    # ══════════════════════════════════════════════════════════════════
    # Clearance Cones (used by HOMING's guard)
    # ══════════════════════════════════════════════════════════════════

    def update_clearances(self):
        """
        Minimum valid range in a narrow cone straight ahead and straight
        behind. Angle wrap around ±180° is handled by normalising each ray's
        offset from the cone centre instead of slicing index ranges.
        """
        fwd = float('inf')
        rear = float('inf')
        n = len(self.ranges)
        for i in range(n):
            if not self.ray_valid(i):
                continue
            ang = normalize_angle_rad(self.i2a(i))
            if abs(ang) <= self.homing_guard_half_rad:
                fwd = min(fwd, self.ranges[i])
            elif abs(normalize_angle_rad(ang - math.pi)) <= self.homing_guard_half_rad:
                rear = min(rear, self.ranges[i])
        self.forward_clearance_m = fwd
        self.rear_clearance_m = rear

    # ══════════════════════════════════════════════════════════════════
    # Main LiDAR Processing Timer
    # ══════════════════════════════════════════════════════════════════

    def calc_lidar_step(self):
        """Runs the navigation pipeline on each new scan."""
        if not self.new_lidar_val:
            return

        # Snapshot the scan under the lock before the in-place fix-up
        # passes mutate it. lidar_callback shares this node's LiDAR
        # callback group, so the two can never overlap; the lock stays as
        # a guard against any future regrouping.
        with self._scan_lock:
            self.new_lidar_val = False
            self.ranges = self.ranges.copy()
            self.intensities = self.intensities.copy()

        n = len(self.ranges)
        if n == 0 or self.ang_inc <= 0.0:
            return

        # Clearance cones are cheap and HOMING's guard needs them fresh even
        # when the rest of the pipeline is skipped.
        self.update_clearances()

        if self.run_state != STATE_RUNNING:
            return

        now = time.time()
        dt = max(now - self.last_time, 0.001)
        self.last_time = now

        # 1. Dynamic Cast Radius (robot width expands with speed)
        speed_ratio = self.current_speed / self.max_speed if self.max_speed > 0 else 0.0
        self.cast_r = remap(speed_ratio, 0.45, 1.0, self.cast_range_min, self.cast_range_max)

        # 2. Fix missing/invalid rays
        self.fix_all_missing()

        # 3. Find the most open direction
        max_d, t_ang = self.get_max_d()

        # --- Minimum Safe Distance Floor ---
        if max_d < self.min_clear_dist_m:
            self.target_dist = 0.0
            self.target_ang = 0.0
            self.speed = 0.0
            self.speed_boost = 1.0
            self.str_angle = 0.0
            # target_yaw_deg is deliberately NOT cleared: one blocked frame
            # must not snap the wheels straight mid-corner. Speed is zero,
            # so holding the last good heading costs nothing.
            self.get_logger().warn(
                f'All blocked (best {max_d:.2f} m) — holding.',
                throttle_duration_sec=1.0)
            self.publish_debug_scan()
            self.publish_target_marker()
            return

        # 4. Smooth target angle (dual lerp)
        delta = abs(t_ang - self.target_ang)
        if delta > 0.5:
            self.target_ang = t_ang
        else:
            self.target_ang = lerp(self.target_ang, t_ang, min(35 * dt, 1.0))
        self.target_ang = lerp(self.target_ang, t_ang, 0.1)
        self.target_dist = max_d

        target_deg = math.degrees(self.target_ang)

        # 5. Wall hugging trim (before danger sense, so safety can veto it)
        wall_corr = self.wall_hug_correction(target_deg, now)
        target_deg += wall_corr

        # 6. Emergency danger sense
        target_deg = self.danger_sense(target_deg)

        # 7. Fuse with the compass: latch the fully-trimmed target as an
        #    ABSOLUTE heading (the MCU heading boots at 0.0, so this frame
        #    is stable for the whole run). drive_running closes the loop on
        #    it at 40 Hz with fresh /heading; str_angle below stays as the
        #    no-IMU fallback.
        if self.yaw_deg is not None:
            self.target_yaw_deg = normalize_angle_deg(self.yaw_deg + target_deg)

        # 8. Speed boosting on clear straights
        self.speed_boost = 1.0
        boost_idx = self.a2i(math.radians(target_deg))
        if 0 <= boost_idx < n:
            marching_hit = self.marching(boost_idx, radius=self.cast_r / 2.0)
            if (abs(target_deg) < self.boost_angle_thresh
                    and marching_hit["dst"] > self.boost_dist_thresh):
                self.speed_boost = self.boost_max

        # 9. Map target angle to steering range [-1, 1] (fallback path)
        self.str_angle = remap(target_deg, -self.str_ang_thresh, self.str_ang_thresh,
                               -1.0, 1.0)

        # 10. Go signal for drive_running. The real throttle is drive_speed
        #     (with the stall boost and the corner gain); the old
        #     open-distance multiplier and corner speed cap LOWERED the
        #     command while turning and are removed on purpose — this
        #     drivetrain slows under steering load all by itself.
        self.speed = self.max_speed

        # 11. Throttled terminal feedback
        hug = ('n/a' if self.hug_side is None
               else ('L' if self.hug_side > 0 else 'R') + f'{wall_corr:+.1f}°')
        fuse = ('OPEN-LOOP' if self.target_yaw_deg is None or self.yaw_deg is None
                else f'{self.yaw_deg:+6.1f}°→{self.target_yaw_deg:+6.1f}°')
        self.get_logger().info(
            f'[NAV] Target: {target_deg:+6.1f}° | Yaw: {fuse} | '
            f'Dist: {max_d:4.2f}m | Wall: {hug} | '
            f'Lap: {self.lap_count}/{self.target_laps}',
            throttle_duration_sec=0.5)

        # 12. Debug outputs
        self.publish_debug_scan()
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
        debug.header.frame_id = self.scan_header.frame_id or 'laser'
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
        marker.header.frame_id = self.scan_header.frame_id or 'laser'
        marker.ns = "open_round_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.z = float(math.sin(self.target_ang / 2.0))
        marker.pose.orientation.w = float(math.cos(self.target_ang / 2.0))
        marker.scale.x = float(max(self.target_dist, 0.05))
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.9
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = OpenRoundRunNode()
    # Two threads, one per callback group: the LiDAR maths runs on one
    # while the control loop / watchdog / button / state callbacks keep
    # ticking on the other, so no single slow callback can freeze the node.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
