#!/usr/bin/env python3
"""
estop_node.py -- LiDAR safety bubble + automatic reverse recovery.

Assumes a 180 deg usable FOV: -90 deg (right) .. +90 deg (left), 0 = forward.
THERE IS NO REAR COVERAGE. See the warning above reverse_recovery().

Every change from the original is marked with a numbered "# FIX n:" comment.
"""

# FIX 1: original file had no indentation at all -- the class body, every
#        method body and main() were flush left. That is a hard SyntaxError,
#        nothing would have imported. Whole file re-indented.

import math

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data   # FIX 2: see subscription below
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry               # FIX 3: real reverse distance
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


# ==========================================================================
# Pure helpers -- no ROS types, so they can be unit tested offline
# ==========================================================================

def sanitize_ranges(ranges, range_min, range_max, min_valid, max_valid):
    """inf / nan / 0.0 / out-of-spec -> NaN.

    FIX 4: the original did
        np.nan_to_num(..., neginf=0, posinf=0)
    which maps "no return" to "obstacle touching the lidar". inf means the
    beam found NOTHING, which is the safest reading there is, not the most
    dangerous. With that mapping every unreturned beam sits at the origin,
    permanently inside the safety box, so the robot would latch stopped
    forever and never move. Invalid readings must be MASKED, not valued.
    Note np.nan_to_num also silently turns real NaN into 0.0 for the same
    reason -- both directions of that bug are fixed here.
    """
    r = np.asarray(ranges, dtype=np.float64).copy()
    lo = max(range_min, min_valid)
    hi = min(range_max, max_valid)
    bad = ~np.isfinite(r) | (r <= lo) | (r >= hi)
    r[bad] = np.nan
    return r


def median_filter_1d(r, k, circular=False):
    """Sliding median across neighbouring beams. k must be odd.

    Invalid beams are filled with +inf, NOT skipped. A no-return is evidence
    of "nothing there", so it has to vote in the median like any other far
    reading. np.nanmedian would ignore it instead, which lets a single noise
    speckle inside a window of no-returns survive and spread onto both its
    neighbours -- one bad beam becomes three. With +inf and odd k the median
    is always a real element, so inf never averages into a finite distance.

    circular=False here: with a 180 deg FOV the first and last beams are not
    neighbours, so the ends are padded rather than wrapped.
    """
    if k < 3:
        return r
    half = k // 2
    filled = np.where(np.isfinite(r), r, np.inf)
    if circular:
        ext = np.concatenate([filled[-half:], filled, filled[:half]])
    else:
        pad = np.full(half, np.inf)
        ext = np.concatenate([pad, filled, pad])
    out = np.median(sliding_window_view(ext, k), axis=1)
    out[~np.isfinite(out)] = np.nan
    return out


def to_xy(r, cos_a, sin_a, lidar_x, lidar_y):
    """Valid polar beams -> (x, y) points in base_link. x fwd, y left."""
    ok = np.isfinite(r)
    if not np.any(ok):
        return np.empty(0), np.empty(0)
    rv = r[ok]
    return rv * cos_a[ok] + lidar_x, rv * sin_a[ok] + lidar_y


def count_in_box(x, y, front, rear, half_width):
    """Points inside the axis-aligned safety box. A count, not an average.

    A mean over a sector is the wrong statistic for obstacle detection: a
    chair leg at 0.2 m in front of a wall at 4 m averages to ~3.5 m and
    vanishes. Counting occupancy is noise-robust without hiding small
    close objects.
    """
    if x.size == 0:
        return 0
    return int(np.count_nonzero(
        (x <= front) & (x >= -rear) & (np.abs(y) <= half_width)))


def escape_turn_sign(x, y, look_ahead, half_width):
    """Which way to swing the nose while reversing.

    Returns +1 to rotate CCW (nose swings left), -1 for CW (nose right).
    We turn the nose AWAY from whichever front quadrant is more crowded.
    """
    if x.size == 0:
        return 1.0
    fwd = (x > 0.0) & (x < look_ahead)
    left = int(np.count_nonzero(fwd & (y > half_width * 0.3)))
    right = int(np.count_nonzero(fwd & (y < -half_width * 0.3)))
    if left == right:
        return 1.0
    return -1.0 if left > right else 1.0


def braking_distance(speed, reaction_s, decel):
    v = abs(float(speed))
    if v < 1e-3:
        return 0.0
    return v * reaction_s + (v * v) / (2.0 * max(decel, 1e-3))


# ==========================================================================
# Node
# ==========================================================================

class EStopNode(Node):

    def __init__(self):
        super().__init__("estop_node")

        # ---------------- parameters ----------------
        self.declare_parameter('footprint_length', 0.32)
        self.declare_parameter('footprint_width', 0.21)
        self.declare_parameter('margin_front', 0.15)
        self.declare_parameter('margin_rear', 0.10)
        self.declare_parameter('margin_side', 0.06)
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)
        self.declare_parameter('reaction_time', 0.15)
        self.declare_parameter('decel', 1.2)
        self.declare_parameter('max_speed_expand', 0.6)
        self.declare_parameter('median_window', 3)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('history_len', 3)
        self.declare_parameter('trigger_hits', 2)
        self.declare_parameter('min_valid_range', 0.06)
        self.declare_parameter('max_valid_range', 12.0)
        self.declare_parameter('clear_scale', 1.35)
        self.declare_parameter('scan_timeout', 0.4)

        # ---- auto-reverse recovery ----
        self.declare_parameter('enable_auto_reverse', True)
        self.declare_parameter('reverse_speed', 0.10)        # m/s, keep small
        self.declare_parameter('reverse_turn_rate', 0.35)    # rad/s
        self.declare_parameter('max_reverse_distance', 0.25)  # m, HARD cap
        self.declare_parameter('max_reverse_time', 4.0)      # s, HARD cap
        self.declare_parameter('reverse_settle_s', 0.4)
        self.declare_parameter('require_rear_clear', False)  # gate on rear sensor

        g = lambda n: self.get_parameter(n).value
        self.L = float(g('footprint_length'))
        self.W = float(g('footprint_width'))
        self.m_front = float(g('margin_front'))
        self.m_rear = float(g('margin_rear'))
        self.m_side = float(g('margin_side'))
        self.lidar_x = float(g('lidar_x'))
        self.lidar_y = float(g('lidar_y'))
        self.lidar_yaw = float(g('lidar_yaw'))
        self.reaction = float(g('reaction_time'))
        self.decel = float(g('decel'))
        self.max_expand = float(g('max_speed_expand'))
        self.med_k = int(g('median_window')) | 1        # force odd
        self.min_points = int(g('min_points'))
        self.hist_len = int(g('history_len'))
        self.trigger_hits = int(g('trigger_hits'))
        self.min_valid = float(g('min_valid_range'))
        self.max_valid = float(g('max_valid_range'))
        self.clear_scale = float(g('clear_scale'))
        self.scan_timeout = float(g('scan_timeout'))
        self.rev_enable = bool(g('enable_auto_reverse'))
        self.rev_speed = abs(float(g('reverse_speed')))
        self.rev_turn = float(g('reverse_turn_rate'))
        self.rev_max_dist = float(g('max_reverse_distance'))
        self.rev_max_time = float(g('max_reverse_time'))
        self.rev_settle = float(g('reverse_settle_s'))
        self.require_rear_clear = bool(g('require_rear_clear'))

        # ---------------- state ----------------
        self.cmd_vel = Twist()
        # FIX 5: vel_status started False, i.e. "safe", before a single scan
        #        had arrived. Fail-safe means starting stopped and earning
        #        your way out of it.
        self.vel_status = True          # True = e-stop engaged
        self.reason = 'startup'
        self.hist = []                  # FIX 6: no debounce at all before
        self.last_scan_t = None         # FIX 7: no staleness watchdog before
        self.n_stop = 0
        self.n_clear = 0
        self.turn_sign = 1.0
        self.rear_clear = not self.require_rear_clear

        self._cos = None
        self._sin = None
        self._geom_key = None

        # recovery state machine
        self.rev_state = 'idle'         # idle | reversing | settle
        self.rev_start_t = None
        self.rev_dist = 0.0
        self.rev_settle_t = None
        self.odom_xy = None
        self.rev_start_xy = None
        self.last_tick_t = None

        # ---------------- interfaces ----------------
        # FIX 2: LaserScan was subscribed with plain depth-10 (RELIABLE) QoS.
        #        Most lidar drivers publish BEST_EFFORT, and a RELIABLE
        #        subscriber will not match a BEST_EFFORT publisher -- the
        #        callback simply never fires and the node looks "hung".
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.rear_sub = self.create_subscription(
            Bool, 'rear_clear', self.rear_callback, 10)

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, 'estop_status', 10)
        self.status_sub = self.create_subscription(
            Bool, 'estop_request', self.estop_status_request, 10)

        # FIX 8: 0.2 s = 5 Hz is far too slow to be an e-stop. At 0.6 m/s the
        #        robot travels 12 cm between ticks, and if your controller
        #        publishes cmd_vel at 20 Hz it wins 3 cycles out of 4 -- the
        #        stop would be a coin flip. 50 Hz.
        self.vel_timer = self.create_timer(0.02, self.vel_sender_timer)

        self.get_logger().info(
            f'estop up | footprint {self.L:.2f}x{self.W:.2f} | vote '
            f'{self.min_points} pts | auto-reverse '
            f'{"on" if self.rev_enable else "off"} '
            f'(max {self.rev_max_dist:.2f} m)')

    # ------------------------------------------------------------------
    def _update_geometry(self, msg, n):
        """Cache per-beam cos/sin. Recomputed only if scan geometry changes."""
        key = (n, round(msg.angle_min, 6), round(msg.angle_increment, 9))
        if key == self._geom_key:
            return
        a = msg.angle_min + np.arange(n) * msg.angle_increment + self.lidar_yaw
        self._cos = np.cos(a)
        self._sin = np.sin(a)
        self._geom_key = key
        self.get_logger().info(
            f'scan geometry: {n} beams, '
            f'{math.degrees(msg.angle_min):.0f}..'
            f'{math.degrees(msg.angle_min + n * msg.angle_increment):.0f} deg')

    def _boxes(self, speed):
        """(stop, clear) boxes as (front, rear, half_width)."""
        d = min(braking_distance(speed, self.reaction, self.decel),
                self.max_expand)
        fwd = d if speed >= 0.0 else 0.0
        stop = (self.L / 2 + self.m_front + fwd,
                self.L / 2 + self.m_rear,
                self.W / 2 + self.m_side)
        s = self.clear_scale
        clear = (self.L / 2 + self.m_front * s,
                 self.L / 2 + self.m_rear * s,
                 self.W / 2 + self.m_side * s)
        return stop, clear

    # ------------------------------------------------------------------
    def lidar_callback(self, scan_msg: LaserScan):
        """FIX 9: the original body was

               ranges = np.nan_to_num(np.array(scan_msg), ...)
               return

           Two bugs. np.array(scan_msg) wraps the LaserScan MESSAGE OBJECT,
           not scan_msg.ranges -- you get a 0-d object array, and any
           arithmetic on it raises. And the result was assigned to a local
           that was immediately discarded, so the callback did nothing.
        """
        n = len(scan_msg.ranges)
        if n == 0:
            return
        self._update_geometry(scan_msg, n)
        self.last_scan_t = self.get_clock().now()

        # 1. mask invalid returns (never map them to a distance)
        r = sanitize_ranges(scan_msg.ranges, scan_msg.range_min,
                            scan_msg.range_max, self.min_valid, self.max_valid)
        # 2. kill isolated speckle
        r = median_filter_1d(r, self.med_k, circular=False)
        # 3. polar -> cartesian in base_link
        x, y = to_xy(r, self._cos, self._sin, self.lidar_x, self.lidar_y)

        # 4. spatial vote inside the speed-scaled box
        stop_box, clear_box = self._boxes(self.cmd_vel.linear.x)
        self.n_stop = count_in_box(x, y, *stop_box)
        self.n_clear = count_in_box(x, y, *clear_box)
        self.turn_sign = escape_turn_sign(x, y, clear_box[0] * 2.0,
                                          clear_box[2])

        # 5. temporal debounce, then latch
        self.hist.append(self.n_stop >= self.min_points)
        if len(self.hist) > self.hist_len:
            self.hist.pop(0)

        if not self.vel_status:
            if sum(self.hist) >= self.trigger_hits:
                self.engage(f'{self.n_stop} pts in bubble')
        elif self.n_clear == 0 and self.reason != 'manual':
            # front is genuinely clear -- recovery (if any) has done its job
            if self.rev_state == 'idle':
                self.release()

    def odom_callback(self, msg: Odometry):
        """FIX 3: reverse distance measured from odom instead of guessed from
        elapsed time. You already publish nav_msgs/Odometry from the encoders,
        so use it -- integrating a commanded velocity assumes the wheels
        actually turned, which is exactly what you cannot assume when you are
        pinned against something."""
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def rear_callback(self, msg: Bool):
        """Optional gate from a rear sonar or bumper. See reverse warning."""
        self.rear_clear = bool(msg.data)

    def estop_status_request(self, request: Bool):
        """FIX 10: the original body was the bare expression

               self.vel_status

           which evaluates the attribute and throws the value away. It never
           assigned anything, so the estop_request topic did nothing at all.

           FIX 11: an external request may always ENGAGE, but may only CLEAR
           when the lidar agrees the box is empty. Letting a remote topic
           clear a stop while an obstacle is still 5 cm from the bumper
           defeats the point of the node.
        """
        if request.data:
            self.engage('manual')
        else:
            if self.n_clear == 0 and self.last_scan_t is not None:
                self.release()
            else:
                self.get_logger().warn(
                    f'manual clear refused: {self.n_clear} pts still in box')

    # ------------------------------------------------------------------
    def reverse_recovery(self, now, dt):
        """Back the car out of a stop until the front box is clear.

        ############ WARNING -- THIS DRIVES INTO A BLIND SPOT ############
        The lidar covers -90..+90 deg. Nothing behind the car is observed,
        ever. This manoeuvre is therefore an unverified move into unknown
        space, and no amount of code makes it safe. The mitigations here are
        all about limiting the damage: crawl speed, a hard distance cap
        (max_reverse_distance, default 0.25 m -- less than one car length),
        a hard time cap, and an optional rear_clear gate you should wire to
        a rear sonar or a bumper switch before you trust this on anything
        but a clear floor. Do not raise the caps to "make it work better".
        ##################################################################

        Returns a Twist to publish.
        """
        stop = Twist()

        if not self.rev_enable:
            return stop
        # never reverse on stale data -- we would be blind in both directions
        if self.last_scan_t is None:
            return stop
        age = (now - self.last_scan_t).nanoseconds * 1e-9
        if age > self.scan_timeout:
            return stop
        if self.require_rear_clear and not self.rear_clear:
            self.get_logger().warn('reverse blocked: rear not clear',
                                   throttle_duration_sec=2.0)
            return stop

        # ---- idle: decide whether to start ----
        if self.rev_state == 'idle':
            if self.reason == 'manual' or self.n_clear == 0:
                return stop            # nothing to escape from
            self.rev_state = 'reversing'
            self.rev_start_t = now
            self.rev_dist = 0.0
            self.rev_start_xy = self.odom_xy
            self.get_logger().warn(
                f'auto-reverse: backing up, nose swinging '
                f'{"left" if self.turn_sign > 0 else "right"}')

        # ---- reversing ----
        if self.rev_state == 'reversing':
            # distance travelled: odom if we have it, else integrate command
            if self.odom_xy is not None and self.rev_start_xy is not None:
                self.rev_dist = math.hypot(
                    self.odom_xy[0] - self.rev_start_xy[0],
                    self.odom_xy[1] - self.rev_start_xy[1])
            else:
                self.rev_dist += self.rev_speed * dt

            elapsed = (now - self.rev_start_t).nanoseconds * 1e-9

            if self.n_clear == 0:
                self._end_reverse(now, 'front clear')
                return stop
            if self.rev_dist >= self.rev_max_dist:
                self._end_reverse(now, f'distance cap {self.rev_dist:.2f} m')
                return stop
            if elapsed >= self.rev_max_time:
                self._end_reverse(now, 'time cap')
                return stop

            cmd = Twist()
            cmd.linear.x = -self.rev_speed
            cmd.angular.z = self.turn_sign * self.rev_turn
            return cmd

        # ---- settle: sit still briefly so odometry and the scan agree ----
        if self.rev_state == 'settle':
            if (now - self.rev_settle_t).nanoseconds * 1e-9 >= self.rev_settle:
                self.rev_state = 'idle'
            return stop

        return stop

    def _end_reverse(self, now, why):
        self.get_logger().info(f'auto-reverse finished: {why}')
        self.rev_state = 'settle'
        self.rev_settle_t = now

    # ------------------------------------------------------------------
    def vel_sender_timer(self):
        now = self.get_clock().now()
        dt = 0.02
        if self.last_tick_t is not None:
            dt = max((now - self.last_tick_t).nanoseconds * 1e-9, 1e-3)
        self.last_tick_t = now

        # FIX 7: watchdog. A silent lidar is a danger state, not a green light.
        if self.last_scan_t is None:
            self.engage('no scan yet')
        elif (now - self.last_scan_t).nanoseconds * 1e-9 > self.scan_timeout:
            self.engage('scan stale')

        # FIX 12: status_pub was created and then never published to.
        self.status_pub.publish(Bool(data=self.vel_status))

        if not self.vel_status:
            self.rev_state = 'idle'
            return

        cmd = self.reverse_recovery(now, dt)

        # FIX 13: the original set only linear.x = 0.0 and reused a member
        #         Twist, leaving angular.z at whatever it last held. A stop
        #         command that still spins is not a stop. Build a fresh Twist
        #         and zero every field.
        self.cmd_vel = cmd
        self.vel_pub.publish(cmd)

    # ------------------------------------------------------------------
    def engage(self, reason):
        if not self.vel_status:
            self.get_logger().warn(f'E-STOP engaged: {reason}')
        self.vel_status = True
        self.reason = reason

    def release(self):
        if self.vel_status:
            self.get_logger().info('E-STOP cleared')
        self.vel_status = False
        self.reason = ''
        self.hist.clear()
        self.rev_state = 'idle'


# FIX 14: `return;` with a trailing semicolon at the end of three methods.
#         Legal Python, but the semicolon is C habit and the bare return at
#         the end of a function does nothing. Removed.


def main(args=None):
    rclpy.init(args=args)
    estop_node = EStopNode()
    # FIX 15: no KeyboardInterrupt handling and no rclpy.shutdown(). Ctrl-C
    #         dumped a traceback and left the context uninitialised.
    try:
        rclpy.spin(estop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # FIX 16: last thing this node ever does is command a stop.
        estop_node.vel_pub.publish(Twist())
        estop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()