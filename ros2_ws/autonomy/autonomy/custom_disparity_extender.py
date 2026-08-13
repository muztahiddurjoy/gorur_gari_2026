"""
Custom disparity extender: tower detection + gap navigation between pillars.

Detection pipeline (per scan):
  falling/rising edge pairs -> asymmetric width gate -> dedup -> /tower_markers

Navigation pipeline (classic disparity extender, per scan):
  1. Build a nav view of the scan: no-return rays count as OPEN space at
     nav.max_range_m, dropouts are interpolated from valid neighbours, and
     everything is capped at nav.max_range_m.
  2. At every disparity (neighbouring rays differing by more than
     nav.disparity_threshold_m) smear the CLOSER distance sideways over the
     angle that (bot half-width + safety margin) subtends at that distance.
     A gap the bot cannot physically fit through simply disappears from the
     scan, so the target picker can never choose it.
  3. Steer at the farthest surviving ray inside the FOV, ties broken toward
     straight ahead. Speed scales with free distance and steering effort;
     a blocked forward stop-cone parks the car.

All physical geometry (bot width/length, margins, steering lock, speed
band, tower spec) comes from config/bot_config.yaml. The bot.* section is
shared by every node via the /** wildcard, so re-dimensioning the bot is a
config edit, not a code edit.

Topics:
  /scan                     in   sensor_msgs/LaserScan   the LiDAR fan
  /cmd_vel                  out  geometry_msgs/Twist     speed + steering
  /tower_markers            out  MarkerArray             red disk per tower
  /custom_disparity/target  out  MarkerArray             arrow + sphere at
                                                         the steering target

A beginner-friendly, block-by-block walkthrough of this whole file lives
in docs/custom_disparity_extender.md (Part 1 startup, Part 2 parameters,
Part 3 scan intake and safety, Part 4 navigation, Part 5 detection,
Part 6 measurement and markers).

Run:
  ros2 run autonomy custom_disparity_extender

bot_config.yaml is located and loaded automatically (see find_bot_config).
Passing an explicit --params-file overrides the automatic one:
  ros2 run autonomy custom_disparity_extender --ros-args \
      --params-file config/bot_config.yaml
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def find_bot_config():
    """Locate config/bot_config.yaml by walking up from this file.

    The node can execute from the source tree, build/autonomy (symlink
    install) or install/autonomy — all of them live somewhere under the
    workspace root, and the workspace root is the directory that contains
    config/bot_config.yaml. Walking parents finds the LIVE file the user
    edits, never a stale colcon copy.
    """
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'config' / 'bot_config.yaml'
        if candidate.is_file():
            return str(candidate)
    # Last resort: the documented way to run is from the workspace root.
    candidate = Path.cwd() / 'config' / 'bot_config.yaml'
    return str(candidate) if candidate.is_file() else None


class CustomDisparityExtender(Node):

    def __init__(self):
        """Declare every parameter, derive the working geometry, wire up I/O.

        The 25 parameters arrive in three groups (docs Part 2 covers each):
          bot.*      physical geometry shared by every node (/** in the YAML)
          nav.*      disparity-extender tuning (this node's own YAML section)
          top-level  detection tuning, topic names, and the enable_drive gate
        """
        super().__init__('custom_disparity_extender')

        p = self._param

        # ── Global bot geometry (bot_config.yaml, /** section) ────
        self.bot_width = float(p('bot.width_m', 0.21))
        self.bot_length = float(p('bot.length_m', 0.31))
        self.safety_margin = float(p('bot.safety_margin_m', 0.06))
        self.max_steer_rad = math.radians(float(p('bot.max_steer_deg', 60.0)))
        self.max_speed = float(p('bot.max_speed_mps', 0.6))
        self.min_speed = float(p('bot.min_speed_mps', 0.25))

        # Verify against: ros2 topic echo /scan --field header.frame_id
        # C1 drivers publish either 'laser' or 'laser_frame'; a mismatch
        # makes markers vanish silently.
        self.frame_id = str(p('bot.lidar.frame_id', 'laser'))

        # Heading of the lidar's 0-deg ray relative to bot-forward, same
        # convention as e_stop's lidar_yaw. A lidar mounted rotated (e.g.
        # cable-forward = 180) shifts every ray by this angle; without the
        # correction the FOV window, stop cone and steering target are all
        # computed on the wrong heading.
        self.lidar_yaw = math.radians(float(p('bot.lidar.yaw_deg', 0.0)))

        tower_w = float(p('bot.tower.width_m', 0.05))

        # Pillars sit min_separation apart centre-to-centre at the tightest.
        # Anything closer than this is the same pillar detected twice.
        self.min_tower_separation = float(p('bot.tower.min_separation_m', 0.10))

        # Derived from the geometry:
        #  - extend_radius is the half-corridor the bot needs to pass anything;
        #    it is THE quantity the disparity extension smears obstacles by.
        #  - tower width gate end-points: face-on vs corner-on (w * sqrt(2)).
        self.extend_radius = self.bot_width / 2.0 + self.safety_margin
        self.tower_w_min_face = tower_w * 100.0
        self.tower_w_max_diag = tower_w * 100.0 * math.sqrt(2.0)

        # ── Edge detection tuning ─────────────────────────────────
        # 0.06 m: a pillar seated 400 mm off the inner wall gives only a small
        # disparity at grazing incidence. 0.10 started missing those past 1.5 m.
        self.spike_threshold = float(p('spike_threshold_m', 0.06))
        self.spike_ref_dist = float(p('spike_ref_dist_m', 0.8))
        self.edge_stride = int(p('edge_stride', 6))
        self.fov_half_deg = float(p('detect_fov_half_deg', 90.0))

        # 3.0 cm floor: below that is one ray of noise at working range.
        # 25.0 cm ceiling: the widest legitimate object is the 200 mm parking
        # lot limitation; anything larger is wall.
        self.width_min_cm = float(p('width_min_cm', 3.0))
        self.width_max_cm = float(p('width_max_cm', 25.0))

        # Corridor is 1000 mm wide and the mat 3200 mm. Beyond 2 m a 5 cm
        # pillar subtends <= 2 rays, so the width estimate is meaningless.
        self.max_useful_range = float(p('max_useful_range_m', 2.0))

        # ── Navigation tuning ─────────────────────────────────────
        self.nav_fov_half = math.radians(float(p('nav.fov_half_deg', 80.0)))
        self.nav_max_range = float(p('nav.max_range_m', 3.0))
        self.disparity_thresh = float(p('nav.disparity_threshold_m', 0.25))
        self.stop_distance = float(p('nav.stop_distance_m', 0.35))
        self.stop_cone_half = math.radians(float(p('nav.stop_cone_half_deg', 20.0)))
        self.steer_speed_drop = float(p('nav.steer_speed_drop', 0.4))

        # Drive gate. Default OFF in code so a bare `ros2 run` without the
        # config file can never move the car; bot_config.yaml enables it.
        self.enable_drive = bool(p('enable_drive', True))
        self.wait_for_track_ready = bool(p('wait_for_track_ready', False))
        self.track_ready = not self.wait_for_track_ready

        scan_topic = str(p('scan_topic', '/scan'))
        cmd_topic = str(p('cmd_vel_topic', '/cmd_vel'))

        if self.wait_for_track_ready:
            from std_msgs.msg import Bool
            from rclpy.qos import DurabilityPolicy, QoSProfile
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.track_ready_sub = self.create_subscription(
                Bool, '/track_ready', self.track_ready_callback, latched)

        # ── LiDAR state (fallbacks until the first scan arrives) ──

        # 0.72 deg matches an RPLIDAR C1 at 10 Hz.
        self.min_ang = -math.pi
        self.max_ang = math.pi
        self.ang_inc = math.radians(0.72)
        self.ranges = np.empty(0, dtype=np.float32)
        self.valid = np.empty(0, dtype=bool)
        self.last_scan_time = 0.0

        # Per-scan yaw correction actually applied (index shift and the
        # angle it corresponds to). Zero until the first scan arrives.
        self.yaw_shift = 0
        self.yaw_applied = 0.0

        # ── I/O ───────────────────────────────────────────────────
        # sensor-data QoS (BEST_EFFORT), NOT the default depth-10 RELIABLE:
        # the C1 driver publishes BEST_EFFORT and a RELIABLE subscriber never
        # matches it — the callback simply never fires and no marker or
        # cmd_vel ever appears (same failure e_stop's FIX 2 documents).
        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.lidr_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/tower_markers', 10)
        self.target_pub = self.create_publisher(MarkerArray, '/custom_disparity/target', 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.vision_sub = self.create_subscription(
            Float32MultiArray, '/closest_obj', self.vision_callback, qos_profile_sensor_data)
        self.vision_colors = []

        # A silent /scan must never leave the MCU on a stale throttle.
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f'Custom disparity extender up | bot {self.bot_width:.2f}x'
            f'{self.bot_length:.2f} m | extend radius {self.extend_radius:.3f} m | '
            f'drive {"ENABLED" if self.enable_drive else "disabled"}'
        )

    def _param(self, name, default):
        """Declare with a default and read back the (possibly overridden) value."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def vision_callback(self, msg: Float32MultiArray):
        """Update the list of detected pillar colors from the vision node."""
        self.vision_colors = list(msg.data)

    # ──────────────────────────────────────────────────────────────
    # Index <-> angle helpers
    # ──────────────────────────────────────────────────────────────

    def a2i(self, angle: float) -> int:
        """Angle (radians) -> array index. round(), not int(): int() truncates
        toward zero, which is asymmetric either side of 0."""
        return int(round((angle - self.min_ang) / self.ang_inc))

    def i2a(self, index: int) -> float:
        """Array index -> angle (radians)."""
        return self.min_ang + index * self.ang_inc

    # ──────────────────────────────────────────────────────────────
    # Scan handling
    # ──────────────────────────────────────────────────────────────

    def lidr_callback(self, msg: LaserScan):
        """Per-scan entry point. Refresh the scan geometry, clean the ranges,
        then run both pipelines: detection (find_objects -> is_tower gate ->
        dedup -> /tower_markers) and navigation (drive_step)."""
        # Read geometry from the message every frame. Hardcoding min_ang = -pi
        # breaks a2i() on any LiDAR publishing 0..2pi: the index goes negative
        # and Python silently wraps to the end of the array.
        self.min_ang = msg.angle_min
        self.max_ang = msg.angle_max
        self.ang_inc = msg.angle_increment
        # Stamped before the sanity check below on purpose: even a malformed
        # scan proves the driver is alive, and alive-ness is all the watchdog
        # measures.
        self.last_scan_time = time.time()

        if self.ang_inc <= 0.0:
            self.get_logger().warn('angle_increment is non-positive; dropping scan.')
            return

        raw = np.array(msg.ranges, dtype=np.float32)
        if raw.size == 0:
            return

        # Rotate the scan into bot-forward heading. Rolling the (full-circle)
        # array by the yaw's index equivalent keeps a2i/i2a and every window
        # [lo, hi] contiguous — even a 180-deg mount stays a simple shift
        # instead of a window split across both ends of the array.
        self.yaw_shift = 0
        if self.lidar_yaw != 0.0:
            span = (self.max_ang - self.min_ang) + self.ang_inc
            if abs(span - 2.0 * math.pi) < 0.2:
                self.yaw_shift = int(round(self.lidar_yaw / self.ang_inc))
                raw = np.roll(raw, self.yaw_shift)
            else:
                self.get_logger().warn(
                    'bot.lidar.yaw_deg is set but the scan is not full-circle; '
                    'heading correction skipped.', throttle_duration_sec=5.0)
        self.yaw_applied = self.yaw_shift * self.ang_inc

        # ONE sentinel for every kind of invalid reading. Mapping nan -> inf and
        # inf -> 0 would turn "nothing in range" into "obstacle at 0 m" and
        # manufacture a disparity against the neighbouring ray.
        r = np.nan_to_num(raw, nan=0.0, posinf=0.0, neginf=0.0)

        upper = min(float(msg.range_max), self.max_useful_range)
        valid = (r > max(float(msg.range_min), 0.01)) & (r < upper)

        self.ranges = r
        self.valid = valid

        # Gate to tower-sized objects BEFORE deduplication, so a wall fragment
        # cannot occupy the separation radius and suppress a real pillar.
        objects = [o for o in self.find_objects() if self.is_tower(o)]
        objects.sort(key=lambda o: o["dist"])
        towers = self.deduplicate(objects)
        
        for i, t in enumerate(towers):
            if i < len(self.vision_colors):
                t["color"] = self.vision_colors[i]
            else:
                t["color"] = 0.0

        marker_array = MarkerArray()
        for i, o in enumerate(towers):
            marker_array.markers.append(self.make_marker(o, i))

        # Publish unconditionally: an empty array is meaningful information.
        self.marker_pub.publish(marker_array)

        if towers:
            target = towers[0]
            color_str = "Red" if target.get("color") == 1.0 else "Green" if target.get("color") == 2.0 else "Yellow"
            self.get_logger().info(
                f"{len(towers)} tower(s); closest at {target['dist']:.2f} m, "
                f"width {target['width_cm']:.1f} cm, "
                f"angle {target['angle_deg']:.1f} deg, "
                f"color {color_str}",
                throttle_duration_sec=0.5,
            )

        self.drive_step(msg, raw, towers)

    # ──────────────────────────────────────────────────────────────
    # Navigation: disparity extension over the raw scan
    # ──────────────────────────────────────────────────────────────

    def build_nav_ranges(self, msg: LaserScan, raw):
        """
        Nav view of the scan, distinct from the detection mask. `raw` is the
        already yaw-corrected float array from lidr_callback (re-reading
        msg.ranges here would silently drop the heading correction):

          - inf / beyond nav_max_range  -> OPEN space at nav_max_range.
            (Detection treats these as invalid; for driving, "no return"
            usually means "nothing there", and zeroing it would wall off
            every open corridor.)
          - nan / below range_min       -> dropout; interpolated from the
            nearest valid rays so one dead ray doesn't split a real gap.
        """
        if raw.size == 0:
            return None

        open_mask = np.isposinf(raw) | (raw >= self.nav_max_range)
        bad = ~open_mask & (np.isnan(raw) | (raw < max(float(msg.range_min), 0.01)))

        nav = raw.copy()
        nav[open_mask] = self.nav_max_range
        
        if bad.any():
            good = ~bad
            if not good.any():
                return None
            idx = np.arange(nav.size, dtype=np.float32)
            nav[bad] = np.interp(idx[bad], idx[good], nav[good])

        return np.minimum(nav, self.nav_max_range)

    def extend_disparities(self, nav):
        """
        Core of the disparity extender.

        At each step bigger than disparity_thresh between neighbouring rays,
        the closer surface has an exposed edge the bot could clip. Overwrite
        the far side of the edge with the CLOSER distance across the angle
        that extend_radius (half-width + margin) subtends at that distance:

            n_rays = atan(extend_radius / closer) / ang_inc

        After this pass every remaining far ray is guaranteed to be at least
        extend_radius from every detected edge, so "steer at the farthest
        ray" is collision-safe by construction — including between pillars:
        a pillar gap narrower than the bot is erased, a passable one stays.
        """
        d = nav.copy()
        n = d.size

        diffs = np.diff(nav)
        for i in np.flatnonzero(np.abs(diffs) > self.disparity_thresh):
            closer = float(min(nav[i], nav[i + 1]))
            if closer <= 0.0:
                continue

            n_ext = int(math.ceil(
                math.atan2(self.extend_radius, closer) / self.ang_inc))

            if nav[i] < nav[i + 1]:
                # Obstacle is on the low-index side: smear forward.
                j0, j1 = i + 1, min(n, i + 1 + n_ext)
            else:
                # Obstacle is on the high-index side: smear backward.
                j0, j1 = max(0, i + 1 - n_ext), i + 1

            seg = d[j0:j1]
            np.minimum(seg, closer, out=seg)

        return d

    def pick_target(self, d):
        """
        Farthest extended ray inside the nav FOV. Among rays within 95% of
        the best (the cap at nav_max_range creates whole plateaus of "best"),
        take the one closest to straight ahead — maximum clearance with
        minimum steering, and no oscillation between equal far rays.
        """
        lo = max(0, self.a2i(-self.nav_fov_half))
        hi = min(d.size - 1, self.a2i(self.nav_fov_half))
        if hi <= lo:
            return None

        window = d[lo:hi + 1]
        best = float(window.max())
        if best <= 0.0:
            return None

        cand = np.flatnonzero(window >= 0.95 * best) + lo
        centre = self.a2i(0.0)
        idx = int(cand[np.argmin(np.abs(cand - centre))])
        return idx, float(d[idx])

    def track_ready_callback(self, msg):
        if msg.data and not self.track_ready:
            self.track_ready = True
            self.get_logger().info('Received /track_ready from track_maker — starting drive!')

    def drive_step(self, msg: LaserScan, raw, towers=None):
        if not self.track_ready:
            self.publish_stop('waiting for track_maker (/track_ready)')
            return

        nav = self.build_nav_ranges(msg, raw)
        if nav is None:
            if self.enable_drive:
                self.publish_stop('no usable rays in scan')
        # Forward stop-cone: raw nav distances, not extended ones — the
        # extension is for steering choice, physical clearance is physical.
        # Compute clearance BEFORE trimming the nav array!
        c0 = max(0, self.a2i(-self.stop_cone_half))
        c1 = min(nav.size - 1, self.a2i(self.stop_cone_half))
        clearance = float(nav[c0:c1 + 1].min()) if c1 > c0 else 0.0

        # ── WRO Pillar Trimming ──
        # Trim the opposite side lidar values to force passing on the correct side
        if towers:
            closest_tower = towers[0]
            tower_idx = self.a2i(math.radians(closest_tower["angle_deg"]))
            color = closest_tower.get("color", 0.0)
            
            if color == 1.0:
                # RED -> Pass Right. Trim the LEFT side (indices > tower_idx)
                nav[tower_idx:] = 0.0
            elif color == 2.0:
                # GREEN -> Pass Left. Trim the RIGHT side (indices < tower_idx)
                nav[:tower_idx] = 0.0

        extended = self.extend_disparities(nav)
        pick = self.pick_target(extended)

        target_ang = None
        if pick is not None:
            idx, free_dist = pick
            target_ang = self.i2a(idx)
            # Marker back in the lidar's own convention so the arrow overlays
            # the raw /scan display in RViz regardless of mount yaw.
            self.publish_target_marker(target_ang - self.yaw_applied, free_dist)

        if not self.enable_drive:
            return

        if target_ang is None or clearance < self.stop_distance:
            self.publish_stop(f'blocked, clearance {clearance:.2f} m')
            return

        # REP-103: +angle = left = +angular.z. Normalised to [-1, 1] of the
        # physical lock, same convention the MCU bridge expects on /cmd_vel.
        # The clamp is reachable: the nav FOV allows targets out to +-80 deg
        # while the physical lock is only +-60 deg.
        steer = clamp(target_ang / self.max_steer_rad, -1.0, 1.0)

        open_frac = clamp(
            (free_dist - self.stop_distance)
            / max(self.nav_max_range - self.stop_distance, 1e-6),
            0.0, 1.0)
        speed = self.min_speed + (self.max_speed - self.min_speed) * open_frac
        speed *= 1.0 - self.steer_speed_drop * abs(steer)

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steer)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[NAV] {math.degrees(target_ang):+6.1f} deg | free {free_dist:.2f} m | '
            f'clear {clearance:.2f} m | v {speed:.2f} m/s',
            throttle_duration_sec=0.5,
        )

    def publish_stop(self, reason: str):
        """Publish an all-zero Twist (full stop) and say why, at most 1/s."""
        self.cmd_pub.publish(Twist())
        self.get_logger().warn(f'STOP — {reason}.', throttle_duration_sec=1.0)

    def watchdog(self):
        """Emergency stop if /scan goes quiet while we are allowed to drive."""
        if not self.enable_drive or not self.track_ready or self.last_scan_time == 0.0:
            return
        if time.time() - self.last_scan_time > 2.0:
            self.cmd_pub.publish(Twist())
            self.get_logger().error(
                'LiDAR silent > 2.0 s — emergency stop.',
                throttle_duration_sec=1.0)


    # ──────────────────────────────────────────────────────────────
    # Edge detection + width measurement
    # ──────────────────────────────────────────────────────────────

    def threshold_at(self, dist: float) -> float:
        """Scale the spike threshold with distance.

        At long range the natural range difference between adjacent rays on an
        obliquely-viewed wall approaches a fixed threshold, so the detector
        fires on flat surfaces. Growing the threshold with range keeps it
        sensitive up close without constant false positives far away.
        """
        return self.spike_threshold * max(1.0, dist / self.spike_ref_dist)

    def tower_width_gate(self, dist: float):
        """Return (min_cm, max_cm) accepted for a tower at this distance.

        Geometric band is [face, face * sqrt(2)] cm. Each edge is quantised by
        up to one beam step, and a stride > 1 adds further edge ambiguity, so
        the band is widened by a range-dependent slack.
        """
        ray_cm = dist * self.ang_inc * 100.0
        slack = 1.5 * ray_cm + 0.5 * (self.edge_stride - 1) * ray_cm
        return (self.tower_w_min_face - slack,
                self.tower_w_max_diag + slack)

    def is_tower(self, obj: dict) -> bool:
        lo, hi = self.tower_width_gate(obj["dist"])
        return lo <= obj["width_cm"] <= hi

    def find_objects(self):
        """
        Walk the FOV looking for falling-edge -> rising-edge pairs.

        Falling edge (range drops) = an object begins; push its first ray.
        Rising  edge (range jumps) = the object ends;  pop and measure.

        Using the SIGN is what makes this robust. abs() throws the direction
        away, and pairing elements (0,1),(2,3),... then assumes edges alternate
        perfectly starting with an entering edge. One clipped or missed edge
        shifts every later pair by one and you measure the GAP between two
        objects instead of an object.

        Stride compensation: comparing ranges[i] against ranges[i-stride] means
        the transition lies somewhere in (i-stride, i]. Both edges take the
        earliest possible boundary, so the two biases cancel in the width:
            p = i - stride + 1   (first ray on object)
            q = i - stride       (last ray on object)
        With stride == 1 these reduce to p = i and q = i - 1.
        """
        n = len(self.ranges)
        if n == 0:
            return []

        stride = max(1, self.edge_stride)

        half = math.radians(self.fov_half_deg)
        start = max(stride, self.a2i(-half))
        end = min(n - 1, self.a2i(half))
        if end <= start:
            return []

        stack = []
        objects = []

        for i in range(start, end):
            # Both samples must be real before their difference means anything.
            if not (self.valid[i] and self.valid[i - stride]):
                continue

            near = float(self.ranges[i])
            far = float(self.ranges[i - stride])
            slope = near - far

            thresh = self.threshold_at(min(near, far))

            if slope < -thresh:
                # Range dropped: object starts at the first ray on it.
                stack.append(i - stride + 1)

            elif slope > thresh:
                # Range jumped back out: object ended at the previous ray.
                if not stack:
                    # Rising edge with no matching fall: the object was already
                    # in view when the window opened. Skip rather than pair it
                    # with something unrelated.
                    continue

                p = stack.pop()
                q = i - stride
                if q < p:
                    continue

                obj = self.measure(p, q)
                if obj is not None:
                    objects.append(obj)

        return objects

    def deduplicate(self, objects):
        """
        Merge detections of the same physical tower.

        One tower can register more than once per scan: nested falling edges
        left on the stack, or a bad ray splitting a segment in two. Duplicates
        land within a few cm of each other in Cartesian space, while distinct
        towers are at least min_tower_separation apart. Input is sorted by
        distance, so the nearest (best-measured) detection wins.
        """
        kept = []
        for o in objects:
            a = math.radians(o["angle_deg"])
            x = o["dist"] * math.cos(a)
            y = o["dist"] * math.sin(a)
            if all(math.hypot(x - k["x"], y - k["y"]) >= self.min_tower_separation
                   for k in kept):
                o["x"] = x
                o["y"] = y
                kept.append(o)
        return kept

    def measure(self, p: int, q: int):
        """
        Measure the object occupying rays [p, q] inclusive.

        Distance comes from INSIDE the segment. At a falling edge, ranges[i-1]
        is the background behind the object and ranges[i] is the object itself;
        using the background meant a 5 cm pillar at 1 m in front of a wall at
        3 m measured ~15 cm, off by exactly the ratio of the two.

        Median rather than mean, so one bad ray inside the segment cannot drag
        the estimate.
        """
        seg = self.ranges[p:q + 1]
        seg_valid = self.valid[p:q + 1]
        seg = seg[seg_valid]

        if seg.size == 0:
            return None

        dist = float(np.median(seg))

        # (q - p + 1) rays, each covering one angular increment, is the object's
        # angular footprint. Using (q - p) undercounts by one increment:
        # negligible on a wide object, ~50% on a two-ray one.
        theta = (q - p + 1) * self.ang_inc

        # Arc length s = r * theta. For a body of width a at distance d the
        # subtended angle is ~ a/d, so s ~ a.
        width_cm = dist * theta * 100.0

        if not (self.width_min_cm < width_cm < self.width_max_cm):
            return None

        mid = (p + q) // 2

        return {
            "start": p,
            "end": q,
            "dist": dist,
            "angle_deg": math.degrees(self.i2a(mid)),
            "width_cm": width_cm,
        }

    # ──────────────────────────────────────────────────────────────
    # Visualization
    # ──────────────────────────────────────────────────────────────

    def make_marker(self, obj: dict, marker_id: int) -> Marker:
        """One flat yellow disk per detected tower, for RViz."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'towers'
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        # obj x/y are in bot-forward convention (yaw-corrected scan); the
        # marker lives in the lidar's frame, so rotate back by the applied
        # yaw to overlay the raw /scan display.
        ya = self.yaw_applied
        m.pose.position.x = obj['x'] * math.cos(ya) + obj['y'] * math.sin(ya)
        m.pose.position.y = -obj['x'] * math.sin(ya) + obj['y'] * math.cos(ya)
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0   # identity: no rotation needed for a flat disk

        # Draw the pillar at its true footprint rather than the measured
        # contour width, which inflates toward w*sqrt(2) when seen corner-on.
        diameter = self.tower_w_min_face / 100.0
        m.scale.x = diameter
        m.scale.y = diameter
        m.scale.z = 0.02             # thin: sits like a flat disk on the ground

        color_code = obj.get("color", 0.0)
        if color_code == 1.0:
            # Red
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
        elif color_code == 2.0:
            # Green
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
        else:
            # Yellow (LiDAR only, unknown color)
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            
        m.color.a = 0.8

        # Markers persist until overwritten or deleted. A short lifetime means
        # a tower that drops out of detection disappears instead of ghosting.
        m.lifetime.sec = 0
        m.lifetime.nanosec = int(0.3 * 1e9)   # 300 ms

        return m

    def publish_target_marker(self, angle: float, dist: float):
        """Arrow along the chosen heading + a sphere ON the point the bot is
        driving toward, so the intent is unambiguous in RViz."""
        stamp = self.get_clock().now().to_msg()

        arrow = Marker()
        arrow.header.frame_id = self.frame_id
        arrow.header.stamp = stamp
        arrow.ns = 'nav_target'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.z = math.sin(angle / 2.0)
        arrow.pose.orientation.w = math.cos(angle / 2.0)
        arrow.scale.x = float(dist)
        arrow.scale.y = 0.04
        arrow.scale.z = 0.04
        arrow.color.g = 1.0
        arrow.color.a = 0.9
        arrow.lifetime.nanosec = int(0.3 * 1e9)

        point = Marker()
        point.header.frame_id = self.frame_id
        point.header.stamp = stamp
        point.ns = 'nav_target'
        point.id = 1
        point.type = Marker.SPHERE
        point.action = Marker.ADD
        point.pose.position.x = dist * math.cos(angle)
        point.pose.position.y = dist * math.sin(angle)
        point.pose.position.z = 0.0
        point.pose.orientation.w = 1.0
        point.scale.x = 0.08
        point.scale.y = 0.08
        point.scale.z = 0.08
        point.color.g = 1.0
        point.color.b = 0.4
        point.color.a = 0.9
        point.lifetime.nanosec = int(0.3 * 1e9)

        arr = MarkerArray()
        arr.markers.append(arrow)
        arr.markers.append(point)
        self.target_pub.publish(arr)


def main(args=None):
    """Entry point: auto-wire bot_config.yaml, start the node, spin forever."""
    argv = list(sys.argv if args is None else args)

    # Auto-wire bot_config.yaml: unless the caller already supplied a params
    # file (launch file / manual --ros-args), find the workspace config and
    # inject it, so a bare `ros2 run` gets the same tuning as everything else.
    config_path = None
    if '--params-file' not in argv:
        config_path = find_bot_config()
        if config_path:
            argv += ['--ros-args', '--params-file', config_path]

    rclpy.init(args=argv)
    node = CustomDisparityExtender()

    if config_path:
        node.get_logger().info(f'Parameters auto-loaded from {config_path}')
    elif '--params-file' in argv:
        node.get_logger().info('Parameters from caller-supplied --params-file.')
    else:
        node.get_logger().warn(
            'bot_config.yaml NOT found — running on in-code defaults '
            '(drive stays disabled).')
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
