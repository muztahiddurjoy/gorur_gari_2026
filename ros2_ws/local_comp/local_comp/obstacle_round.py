import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker

from local_comp.config_utils import load_default_params


class ObstacleRound(Node):

    def __init__(self):
        super().__init__('obstacle_round')
        self.get_logger().info('ObstacleRound node has been started.')

        self.scan = None
        cfg = load_default_params(self, 'obstacle_round.yaml', 'obstacle_round')

        # ── Parameter Declarations with Defaults ──
        self.declare_parameter('fov_min_deg', cfg.get('fov_min_deg', -85.0))
        self.declare_parameter('fov_max_deg', cfg.get('fov_max_deg', 85.0))
        self.declare_parameter('car_radius', cfg.get('car_radius', 0.12))
        self.declare_parameter('disparity_thresh', cfg.get('disparity_thresh', 0.12))
        self.declare_parameter('max_range', cfg.get('max_range', 1.15))
        self.declare_parameter('stop_dist', cfg.get('stop_dist', 0.40))
        self.declare_parameter('wall_safe_dist', cfg.get('wall_safe_dist', 0.24))
        self.declare_parameter('heading_fusion_enable', cfg.get('heading_fusion_enable', True))
        self.declare_parameter('heading_clockwise', cfg.get('heading_clockwise', True))
        self.declare_parameter('heading_kp', cfg.get('heading_kp', 0.030))
        self.declare_parameter('heading_kd', cfg.get('heading_kd', 0.003))
        self.declare_parameter('heading_deadband_deg', cfg.get('heading_deadband_deg', 1.5))
        self.declare_parameter('heading_stale_sec', cfg.get('heading_stale_sec', 0.5))
        self.declare_parameter('max_corridor_deg', cfg.get('max_corridor_deg', 75.0))
        self.declare_parameter('corridor_align_weight', cfg.get('corridor_align_weight', 0.30))
        self.declare_parameter('finish_max_yaw_deg', cfg.get('finish_max_yaw_deg', 38.0))
        self.declare_parameter('steer_alpha', cfg.get('steer_alpha', 0.75))
        self.declare_parameter('speed_straight', cfg.get('speed_straight', 1.0))
        self.declare_parameter('speed_turn', cfg.get('speed_turn', 1.25))
        self.declare_parameter('max_steer_deg', cfg.get('max_steer_deg', 40.0))
        # Press -> first drive command. Set from config/round_select.yaml by
        # gorur_round.launch.py; the firmware's LED countdown must match it.
        self.declare_parameter('start_delay_sec', cfg.get('start_delay_sec', 5.0))

        # ── Read Parameters ──
        self.fov_min_deg = float(self.get_parameter('fov_min_deg').value)
        self.fov_max_deg = float(self.get_parameter('fov_max_deg').value)
        self.car_radius = float(self.get_parameter('car_radius').value)
        self.disparity_thresh = float(self.get_parameter('disparity_thresh').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.wall_safe_dist = float(self.get_parameter('wall_safe_dist').value)
        self.heading_fusion_enable = bool(self.get_parameter('heading_fusion_enable').value)
        self.heading_clockwise = bool(self.get_parameter('heading_clockwise').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_kd = float(self.get_parameter('heading_kd').value)
        self.heading_deadband_deg = float(self.get_parameter('heading_deadband_deg').value)
        self.heading_stale_sec = float(self.get_parameter('heading_stale_sec').value)
        self.max_corridor_rad = math.radians(float(self.get_parameter('max_corridor_deg').value))
        self.corridor_align_weight = float(self.get_parameter('corridor_align_weight').value)
        self.finish_max_yaw_deg = float(self.get_parameter('finish_max_yaw_deg').value)
        self.steer_alpha = float(self.get_parameter('steer_alpha').value)
        self.speed_straight = float(self.get_parameter('speed_straight').value)
        self.speed_turn = float(self.get_parameter('speed_turn').value)
        self.max_steer_rad = math.radians(float(self.get_parameter('max_steer_deg').value))
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        if not self.start_delay_sec > 0.0:
            raise ValueError(f'start_delay_sec must be > 0, got {self.start_delay_sec}')

        self.scan_angles = np.array([])   # radians; sorted right to left
        self.raw_ranges = np.array([])    # metres
        self.scan_ranges = np.array([])   # metres
        self.new_scan_ready = False
        self.last_target_angle = 0.0
        self.last_target_dist = 0.0

        # IMU corridor heading (+CCW deg to match scan_angles & cmd_vel.angular.z)
        self.prev_raw_heading = None
        self.last_heading_time = None
        self.yaw_deg = 0.0
        self.yaw_rate_dps = 0.0
        self.target_yaw_deg = 0.0
        self.prev_steer = 0.0

        self.state = 'STANDBY'
        self.start_timer = None         # one-shot, ends COUNTDOWN
        self.button_was_down = False    # press edge detection
        self.lidar_last_timestamp = None
        self.vel = Twist()

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/button_status', self.button_callback, 10)
        self.create_subscription(Float32, '/heading', self.heading_callback, 10)
        self.create_subscription(
            Odometry, '/gorubot/ground_truth', self.ground_truth_callback, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(Marker, '/obstacle_round/target_marker', 10)
        self.path_pub = self.create_publisher(Path, '/bot_path', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/bot_path_marker', 10)

        # Persistent path history for rviz2
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.last_path_pose = None

        self.create_timer(0.025, self.obstacle_control_loop)

    

    def button_callback(self, msg):
        # Act on the press edge only: the MCU keeps sending True while the
        # button is held, and its LED countdown starts on the edge as well.
        pressed = bool(msg.data)
        rising = pressed and not self.button_was_down
        self.button_was_down = pressed
        if not rising or self.state == 'RUNNING':
            return
        # A press mid-countdown restarts it, exactly like the LED does.
        restarted = self.state == 'COUNTDOWN'
        self.state = 'COUNTDOWN'
        self.reset_imu_run_state()
        self.arm_start_timer()
        self.get_logger().info(
            f'Button pressed{" again, countdown restarted" if restarted else ""}! '
            f'Starting Obstacle Round in {self.start_delay_sec:.1f}s :::::')

    def reset_imu_run_state(self):
        """Zero relative corridor yaw at launch without resetting raw heading."""
        self.yaw_deg = 0.0
        self.yaw_rate_dps = 0.0
        self.target_yaw_deg = 0.0
        self.prev_steer = 0.0

    def arm_start_timer(self):
        """(Re)start the one-shot timer that ends the countdown."""
        if self.start_timer is not None:
            self.destroy_timer(self.start_timer)
        self.start_timer = self.create_timer(self.start_delay_sec, self.start_run)

    def start_run(self):
        """Fire once, start_delay_sec after the press, and drive at once."""
        self.start_timer.cancel()  # one shot
        if self.state != 'COUNTDOWN':
            return
        self.reset_imu_run_state()
        self.new_scan_ready = self.scan_ranges.size > 0
        self.state = 'RUNNING'
        self.get_logger().info(
            f'{self.start_delay_sec:.1f}s elapsed! Navigating obstacle corridor.')
        # First drive command now, not on the next 40 Hz tick.
        self.obstacle_control_loop()

    def heading_callback(self, msg: Float32):
        """Track unwrapped corridor yaw (+CCW deg) and gyro yaw rate from /heading."""
        raw = float(msg.data)
        now = time.monotonic()

        if self.prev_raw_heading is not None:
            # Wrap delta into (-180, +180] so 359 -> 1 deg is +2 deg, not -358
            raw_delta = (raw - self.prev_raw_heading + 180.0) % 360.0 - 180.0
            if abs(raw_delta) < 45.0:  # Reject impossible IMU spike glitches
                yaw_delta = -raw_delta if self.heading_clockwise else raw_delta
                if self.state == 'RUNNING':
                    self.yaw_deg += yaw_delta

                if self.last_heading_time is not None:
                    dt_imu = now - self.last_heading_time
                    if 0.005 < dt_imu < 0.25:
                        inst_rate = yaw_delta / dt_imu
                        self.yaw_rate_dps = 0.6 * inst_rate + 0.4 * self.yaw_rate_dps
            else:
                self.get_logger().warn(
                    f'IMU jump ({raw_delta:+.1f} deg) ignored.',
                    throttle_duration_sec=1.0)

        self.prev_raw_heading = raw
        self.last_heading_time = now

    def imu_is_fresh(self) -> bool:
        """Return True when IMU fusion is enabled and /heading arrived recently."""
        if not self.heading_fusion_enable or self.last_heading_time is None:
            return False
        return (time.monotonic() - self.last_heading_time) <= self.heading_stale_sec

    def scan_callback(self, msg):
        self.lidar_last_timestamp = int(time.time() * 1000)
        self.scan = msg
        self.scan_angles, self.raw_ranges = self.filter_scan(msg)
        if self.raw_ranges.size > 0:
            self.scan_ranges = self.extend_disparities(
                self.raw_ranges, msg.angle_increment)
            self.new_scan_ready = True

    def filter_scan(self, msg):
        raw = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(raw)) * msg.angle_increment
        angles = (angles + math.pi) % (2.0 * math.pi) - math.pi

        in_fov = ((angles >= math.radians(self.fov_min_deg))
                  & (angles <= math.radians(self.fov_max_deg)))
        fov_angles = angles[in_fov]
        fov_ranges = raw[in_fov].copy()

        # Sort from Right to Left
        order = np.argsort(fov_angles)
        fov_angles = fov_angles[order]
        fov_ranges = fov_ranges[order]

        # +inf or far rays as open path
        open_space = np.isposinf(fov_ranges) | (fov_ranges >= self.max_range)
        fov_ranges[open_space] = self.max_range

        # Clean nan or < 0.05m min range
        bad = ~open_space & (np.isnan(fov_ranges) | (fov_ranges < max(msg.range_min, 0.05)))
        if bad.any() and (~bad).any():
            idx = np.arange(len(fov_ranges))
            fov_ranges[bad] = np.interp(idx[bad], idx[~bad], fov_ranges[~bad])

        return fov_angles, fov_ranges

    def extend_disparities(self, ranges, angle_inc):
        smeared = ranges.copy()
        n = len(ranges)

        for i in range(n - 1):
            r_curr = ranges[i]
            r_next = ranges[i + 1]
            diff = r_next - r_curr

            if abs(diff) > self.disparity_thresh:
                closer_dist = min(r_curr, r_next)
                if closer_dist <= 0.05:
                    continue

                # How many C1 rays cover our car_radius at closer_dist (cap flank wrap)
                angle_to_cover = min(
                    math.atan2(self.car_radius, closer_dist),
                    math.radians(40.0),
                )
                n_rays = int(math.ceil(angle_to_cover / angle_inc))

                if diff > 0:  # r_curr is closer ->  toward left
                    end_idx = min(n, i + 1 + n_rays)
                    smeared[i + 1 : end_idx] = np.minimum(
                        smeared[i + 1 : end_idx], closer_dist)
                else:         # r_next is closer -> toward right
                    start_idx = max(0, i + 1 - n_rays)
                    smeared[start_idx : i + 1] = np.minimum(
                        smeared[start_idx : i + 1], closer_dist)

        return smeared

    def pick_target(self, imu_ok: bool):
        """Pick safest corridor-aligned open path from disparity-extended scan."""
        if self.scan_ranges.size == 0 or float(np.max(self.scan_ranges)) <= 0.0:
            return 0.0, 0.0

        ranges = self.scan_ranges
        angles = self.scan_angles

        if imu_ok:
            corridor_angles = angles + math.radians(self.yaw_deg)
            valid_mask = np.abs(corridor_angles) <= self.max_corridor_rad
        else:
            corridor_angles = angles
            valid_mask = np.abs(angles) <= math.radians(70.0)

        masked = np.where(valid_mask, ranges, 0.0)
        if float(np.max(masked)) <= 0.05:
            # Emergency recovery if forward corridor cone is completely blocked
            masked = ranges
            valid_mask = np.ones_like(angles, dtype=bool)

        # Smooth with a 15-ray kernel so tiny pinholes are ignored
        kernel_size = min(15, len(masked))
        smoothed = np.convolve(
            masked, np.ones(kernel_size) / kernel_size, mode='same')

        if imu_ok:
            # Prefer paths aligned down the corridor (+X), avoiding corner dives
            align = 1.0 - self.corridor_align_weight * np.minimum(
                1.0, np.abs(corridor_angles) / (math.pi / 2.0))
            scores = np.where(valid_mask, smoothed * align, 0.0)
        else:
            scores = np.where(valid_mask, smoothed, 0.0)

        peak = float(np.max(scores))
        if peak <= 0.0:
            return 0.0, 0.0

        candidates = np.flatnonzero(scores >= 0.92 * peak)
        splits = np.where(np.diff(candidates) > 1)[0] + 1
        gaps = np.split(candidates, splits)

        def gap_score(gap):
            mean_phi = float(np.mean(np.abs(corridor_angles[gap])))
            return len(gap) * (1.0 - 0.15 * min(1.0, mean_phi / (math.pi / 2.0)))

        best_gap = max(gaps, key=gap_score)

        # Trim edge rays inside the winning gap so we stay clear of obstacles,
        # then blend the safe gap midpoint with the straightest interior ray.
        margin = len(best_gap) // 4
        interior = (
            best_gap[margin : len(best_gap) - margin]
            if len(best_gap) >= 4 else best_gap
        )
        mid_pos = len(best_gap) // 2
        if imu_ok and len(best_gap) >= 4:
            rel_aligned = int(np.argmin(np.abs(corridor_angles[interior])))
            aligned_pos = margin + rel_aligned
            best_idx = int(best_gap[int(round(0.6 * mid_pos + 0.4 * aligned_pos))])
        else:
            best_idx = int(best_gap[mid_pos])

        return float(angles[best_idx]), float(ranges[best_idx])

    def is_finish_wall_ahead(self, imu_ok: bool, front_clear: float) -> bool:
        """
        Confirm the car faces the end wall (finish line), not a side wall.

        With IMU, gate on corridor heading and project forward-corridor rays
        along the +X corridor axis so even slightly angled arrivals stop reliably.
        """
        if imu_ok:
            if abs(self.yaw_deg) > self.finish_max_yaw_deg:
                return False

            corridor_angles = self.scan_angles + math.radians(self.yaw_deg)
            end_cone = np.abs(corridor_angles) <= math.radians(35.0)
            if not np.any(end_cone):
                return False

            x_ahead = self.raw_ranges[end_cone] * np.cos(corridor_angles[end_cone])
            return (float(np.min(x_ahead)) <= self.stop_dist
                    and float(np.max(x_ahead)) <= self.stop_dist + 0.12)

        wide_mask = np.abs(self.scan_angles) <= math.radians(55.0)
        wide_raw_max = float(np.max(self.raw_ranges[wide_mask])) if np.any(wide_mask) else 0.0
        return front_clear <= self.stop_dist and wide_raw_max <= 0.45
    

    # NOT NECESSARY CODE ___ ONLY USED TO DRAW PATH MARKER IN RVIZ2
    def ground_truth_callback(self, msg: Odometry):
            # Accumulate robot path trace
            pos = msg.pose.pose.position
            if self.last_path_pose is None:
                dist = 1.0
            else:
                dx = pos.x - self.last_path_pose.x
                dy = pos.y - self.last_path_pose.y
                dist = math.hypot(dx, dy)
    
            # Add point every 2 cm moved
            if dist >= 0.02:
                self.last_path_pose = pos
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.header.frame_id = 'odom'
                pose_stamped.pose = msg.pose.pose
                self.path_msg.header.stamp = msg.header.stamp
                self.path_msg.poses.append(pose_stamped)
                self.path_pub.publish(self.path_msg)
                self.publish_path_marker()
    
    # NOT NECESSARY CODE ___ ONLY USED TO DRAW PATH MARKER IN RVIZ2
    def publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'bot_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # 2cm wide visible line
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 1.0   # Cyan trail
        marker.color.a = 0.95
        marker.points = [p.pose.position for p in self.path_msg.poses]
        self.path_marker_pub.publish(marker)

    # NOT NECESSARY CODE ___ ONLY USED TO DRAW PATH MARKER IN RVIZ2
    def publish_target_marker(self, target_angle, target_dist):
        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacle_target'
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = float(math.sin(target_angle / 2.0))
        marker.pose.orientation.w = float(math.cos(target_angle / 2.0))
        marker.scale.x = float(max(target_dist, 0.2))
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.r = 1.0
        marker.color.g = 0.84
        marker.color.b = 0.0   # Gold arrow
        marker.color.a = 0.95
        self.target_pub.publish(marker)

    def obstacle_control_loop(self):
        # STANDBY, FINISHED, or COUNTDOWN (start_run() ends that one)
        if self.state != 'RUNNING':
            return

        # Watchdog: Stop immediately if LiDAR disconnects > 1.2s
        now_ms = int(time.time() * 1000)
        if self.lidar_last_timestamp is None or (now_ms - self.lidar_last_timestamp) > 1200:
            self.get_logger().warn('LIDAR data stale! Emergency stop.', throttle_duration_sec=1.0)
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            return

        if self.state == 'RUNNING' and self.scan_ranges.size > 0:
            imu_ok = self.imu_is_fresh()

            # 1. Check in front (-15 to +15 deg)
            front_mask = np.abs(self.scan_angles) <= math.radians(15.0)
            front_clear = float(np.min(self.raw_ranges[front_mask])) if np.any(front_mask) else 0.0

            # Finish Line Detection (IMU-guarded against side-wall false triggers)
            if self.is_finish_wall_ahead(imu_ok, front_clear):
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)
                self.state = 'FINISHED'
                self.get_logger().info(
                    f'FINISHED! Stopped on finish line '
                    f'(front {front_clear:.2f}m, yaw={self.yaw_deg:+.1f} deg).')
                return

            # Latch a new corridor target heading only when a fresh LiDAR scan arrives
            if self.new_scan_ready:
                self.new_scan_ready = False
                target_angle, target_dist = self.pick_target(imu_ok)
                self.last_target_angle = target_angle
                self.last_target_dist = target_dist
                if imu_ok:
                    desired_yaw = self.yaw_deg + math.degrees(target_angle)
                    limit_deg = math.degrees(self.max_corridor_rad)
                    self.target_yaw_deg = float(np.clip(desired_yaw, -limit_deg, limit_deg))

            # Side-bumper repulsion (50..85 deg avoids fighting ahead-diagonal slalom arcs)
            right_mask = (
                (self.scan_angles >= math.radians(-85.0))
                & (self.scan_angles <= math.radians(-50.0))
            )
            left_mask = (
                (self.scan_angles >= math.radians(50.0))
                & (self.scan_angles <= math.radians(85.0))
            )
            right_dist = float(np.min(self.raw_ranges[right_mask])) if np.any(right_mask) else 1.0
            left_dist = float(np.min(self.raw_ranges[left_mask])) if np.any(left_mask) else 1.0

            wall_push = 0.0
            if right_dist < self.wall_safe_dist:
                wall_push += (self.wall_safe_dist - right_dist) * 1.8  # push left
            if left_dist < self.wall_safe_dist:
                wall_push -= (self.wall_safe_dist - left_dist) * 1.8   # push right

            if imu_ok:
                # Closed-loop 40 Hz PD heading hold + derivative-on-measurement damping
                yaw_error = self.target_yaw_deg - self.yaw_deg
                if abs(yaw_error) <= self.heading_deadband_deg:
                    eff_error = 0.0
                else:
                    eff_error = math.copysign(
                        abs(yaw_error) - self.heading_deadband_deg, yaw_error)
                base_steer = (self.heading_kp * eff_error) - (self.heading_kd * self.yaw_rate_dps)
                marker_angle = math.radians(self.target_yaw_deg - self.yaw_deg)
                mode_tag = 'IMU'
            else:
                # Open-loop LiDAR fallback when /heading is unavailable
                base_steer = self.last_target_angle / self.max_steer_rad
                marker_angle = self.last_target_angle
                mode_tag = 'LIDAR-ONLY'

            self.publish_target_marker(marker_angle, self.last_target_dist)

            raw_steer = float(np.clip(base_steer + wall_push, -1.0, 1.0))
            steer = float(
                self.steer_alpha * raw_steer + (1.0 - self.steer_alpha) * self.prev_steer)
            self.prev_steer = steer

            # Turning throttle boost compensates for Ackermann tyre scrub
            speed = self.speed_straight + abs(steer) * (self.speed_turn - self.speed_straight)

            self.vel.linear.x = float(speed)
            self.vel.angular.z = float(steer)
            self.pub.publish(self.vel)

            self.get_logger().info(
                f'[{mode_tag}] yaw={self.yaw_deg:+5.1f}->{self.target_yaw_deg:+5.1f} deg '
                f'(rate={self.yaw_rate_dps:+5.0f} d/s) | Front: {front_clear:.2f}m | '
                f'L/R: {left_dist:.2f}/{right_dist:.2f}m | v={speed:.2f} steer={steer:+.2f}',
                throttle_duration_sec=0.4)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRound()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
