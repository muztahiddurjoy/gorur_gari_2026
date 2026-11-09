import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleRound(Node):

    def __init__(self):
        super().__init__('obstacle_round')
        self.get_logger().info('ObstacleRound node has been started.')

        self.scan = None

        self.fov_min_deg = -70.0
        self.fov_max_deg = 70.0
        self.scan_angles = np.array([])   # radians; sorted -70 to left +70
        self.raw_ranges = np.array([])    # metres
        self.scan_ranges = np.array([])   # metres

        self.state = 'STANDBY'
        self.countdown_start = None
        self.lidar_last_timestamp = None
        self.vel = Twist()

        # BOT CONFIG
        self.car_radius = 0.12        # 0.06m half-width + 0.06m safety margin
        self.disparity_thresh = 0.12  # minimum jump between rays to count as an edge (m)
        self.max_range = 1.15         # 1.15m lookahead (next 2-3 pillars) for timely swerving
        self.stop_dist = 0.40         # finish-line end-wall stop distance (m)
        self.wall_safe_dist = 0.24    # side-wall distance (m)

        # Drive Config 
        self.speed_straight = 1.0    # speed m/s
        self.speed_turn = 1.25        # cornering boost m/s
        self.max_steer_rad = math.radians(40.0)

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/button_status', self.button_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.obstacle_control_loop)

    def button_callback(self, msg):
        if msg.data and self.state == 'STANDBY':
            self.state = 'COUNTDOWN'
            self.countdown_start = time.time()
            self.get_logger().info('Button pressed! Starting Obstacle Round in 5s :::::')

    def scan_callback(self, msg):
        self.lidar_last_timestamp = int(time.time() * 1000)
        self.scan = msg
        self.scan_angles, self.raw_ranges = self.filter_scan(msg)
        if self.raw_ranges.size > 0:
            self.scan_ranges = self.extend_disparities(
                self.raw_ranges, msg.angle_increment)

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

                # How many C1 rays cover our car_radius at closer_dist
                angle_to_cover = math.atan2(self.car_radius, closer_dist)
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

    def pick_target(self):
        """Find the center of the deepest open corridor gap."""
        best_dist = float(np.max(self.scan_ranges))
        if best_dist <= 0.0:
            return 0.0, 0.0

        # Smooth with a 15-ray so that tiny turns are ignored
        # and the center of the open corridor naturally is the peak
        kernel_size = min(15, len(self.scan_ranges))
        smoothed = np.convolve(
            self.scan_ranges, np.ones(kernel_size) / kernel_size, mode='same')

        peak = float(np.max(smoothed))
        candidates = np.flatnonzero(smoothed >= 0.92 * peak)
        best_idx = int(candidates[len(candidates) // 2])

        return float(self.scan_angles[best_idx]), float(self.scan_ranges[best_idx])

    def obstacle_control_loop(self):
        if self.state in ('STANDBY', 'FINISHED'):
            return

        if self.state == 'COUNTDOWN':
            if (time.time() - self.countdown_start) < 5.0:
                return
            self.state = 'RUNNING'
            self.get_logger().info('5s elapsed! Navigating obstacle corridor.')

        # Watchdog: Stop immediately if LiDAR disconnects > 1.2s
        now_ms = int(time.time() * 1000)
        if self.lidar_last_timestamp is None or (now_ms - self.lidar_last_timestamp) > 1200:
            self.get_logger().warn('LIDAR data stale! Emergency stop.', throttle_duration_sec=1.0)
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            return

        if self.state == 'RUNNING' and self.scan_ranges.size > 0:
            # 1. Check in front (-15 to +15 deg)
            front_mask = np.abs(self.scan_angles) <= math.radians(15.0)
            front_clear = float(np.min(self.raw_ranges[front_mask])) if np.any(front_mask) else 0.0
            wide_mask = np.abs(self.scan_angles) <= math.radians(55.0)
            wide_raw_max = float(np.max(self.raw_ranges[wide_mask])) if np.any(wide_mask) else 0.0

            # Finish Line Detection
            # front clearance <= 0.28m AND no raw ray in [-55,+55] deg sees past 0.45m
            if front_clear <= self.stop_dist and wide_raw_max <= 0.45:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)
                self.state = 'FINISHED'
                self.get_logger().info(
                    f'FINISHED! Stopped on finish line (end wall at {front_clear:.2f} m).')
                return

            # Pick target angle at the center of the largest open path
            target_angle, target_dist = self.pick_target()

            # Side-wall repulsion
            right_mask = (self.scan_angles >= math.radians(-70.0)) & (self.scan_angles <= math.radians(-25.0))
            left_mask = (self.scan_angles >= math.radians(25.0)) & (self.scan_angles <= math.radians(70.0))
            right_dist = float(np.min(self.raw_ranges[right_mask])) if np.any(right_mask) else 1.0
            left_dist = float(np.min(self.raw_ranges[left_mask])) if np.any(left_mask) else 1.0

            wall_push = 0.0
            if right_dist < self.wall_safe_dist:
                wall_push += (self.wall_safe_dist - right_dist) * 2.5  # push left 
            if left_dist < self.wall_safe_dist:
                wall_push -= (self.wall_safe_dist - left_dist) * 2.5   # push right

            steer = np.clip((target_angle / self.max_steer_rad) + wall_push, -1.0, 1.0)
            # Turning throttle increase as steer increases
            speed = self.speed_straight + abs(steer) * (self.speed_turn - self.speed_straight)

            self.vel.linear.x = float(speed)
            self.vel.angular.z = float(steer)
            self.pub.publish(self.vel)

            self.get_logger().info(
                f'Target: {math.degrees(target_angle):+5.1f} deg ({target_dist:.2f}m) | '
                f'Front: {front_clear:.2f}m | L/R: {left_dist:.2f}/{right_dist:.2f}m | '
                f'v={speed:.2f} steer={steer:+.2f}',
                throttle_duration_sec=0.4)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRound()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
