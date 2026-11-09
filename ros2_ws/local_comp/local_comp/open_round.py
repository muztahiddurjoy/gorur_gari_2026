import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan
import math
import time

import numpy as np
from geometry_msgs.msg import Twist

class OpenRound(Node):
    
    def __init__(self):
        super().__init__('open_round')
        self.get_logger().info('OpenRound node has been started.')
        self.scan = None

        # FOV -100 to +100 deg
        self.fov_min_deg = -100.0
        self.fov_max_deg = 100.0
        self.max_range = 2.50             # for infinity replacement

        self.car_radius = 0.18            # cm corner 
        self.disparity_thresh = 0.12      # (m)

        ## STEERING CONFIGURATION
        self.wall_safe_dist = 0.35        # Side wall buffer (m)
        self.speed_straight = 1.2        # Straight path speed
        self.speed_turn = 1.5            # Cornering speed (ms^-1)
        self.max_steer_rad = math.radians(30.0)
        self.target_laps_yaw = 1055.0     # 12 corners -> 1080 deg - 25 deg safety reduction = 1055
        self.start_front_dist = None      # distance of front wall logged at start

        self.scan_angles = np.array([])   # radians, sorted -100 to +100
        self.raw_ranges = np.array([])    # metres
        self.scan_ranges = np.array([])   # metres

        self.state = 'STANDBY'
        self.countdown_start = None
        self.lidar_last_timestamp = None
        self.vel = Twist()

        self.prev_raw_heading = None
        self.total_yaw_deg = 0.0        

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/button_status', self.button_callback, 10)
        self.create_subscription(Float32, '/heading', self.heading_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.open_control_loop)

                
    def button_callback(self, msg):
        if msg.data and self.state == 'STANDBY':
            self.state = 'COUNTDOWN'
            self.countdown_start = time.time()
            self.total_yaw_deg = 0.0
            self.get_logger().info('Button pressed! Starting Open Round in 5.0s:::::')

    def heading_callback(self, msg):
        raw = float(msg.data)
        if self.prev_raw_heading is not None:
            # Fix: 359 to 1 deg is +2 deg, not -358 deg
            delta = (raw - self.prev_raw_heading + 180.0) % 360.0 - 180.0
            if abs(delta) < 45.0:  # Ignore IMU glitches
                self.total_yaw_deg += delta
        self.prev_raw_heading = raw

    def scan_callback(self, msg):
        self.lidar_last_timestamp = int(time.time() * 1000)
        self.scan = msg
        self.scan_angles, self.raw_ranges = self.filter_scan(msg)
        if self.raw_ranges.size > 0:
            self.scan_ranges = self.extend_disparities(self.raw_ranges, msg.angle_increment)

    def filter_scan(self, msg):
        """[-100, +100] deg; inf capped at max_range."""
        raw = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(raw)) * msg.angle_increment
        angles = (angles + math.pi) % (2.0 * math.pi) - math.pi

        in_fov = ((angles >= math.radians(self.fov_min_deg))
                  & (angles <= math.radians(self.fov_max_deg)))
        fov_angles = angles[in_fov]
        fov_ranges = raw[in_fov].copy()

        order = np.argsort(fov_angles)
        fov_angles = fov_angles[order]
        fov_ranges = fov_ranges[order]

        open_space = np.isposinf(fov_ranges) | (fov_ranges >= self.max_range)
        fov_ranges[open_space] = self.max_range

        bad = ~open_space & (np.isnan(fov_ranges) | (fov_ranges < max(msg.range_min, 0.05)))
        if bad.any() and (~bad).any():
            idx = np.arange(len(fov_ranges))
            fov_ranges[bad] = np.interp(idx[bad], idx[~bad], fov_ranges[~bad])

        return fov_angles, fov_ranges

    def extend_disparities(self, ranges, angle_inc):
        # Disparity Extender Algorithm for Open Round
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

                angle_to_cover = math.atan2(self.car_radius, closer_dist)
                n_rays = int(math.ceil(angle_to_cover / angle_inc))

                if diff > 0:  # r_curr is closer -> left corner
                    end_idx = min(n, i + 1 + n_rays)
                    smeared[i + 1 : end_idx] = np.minimum(
                        smeared[i + 1 : end_idx], closer_dist)
                else:         # r_next is closer -> right corner
                    start_idx = max(0, i + 1 - n_rays)
                    smeared[start_idx : i + 1] = np.minimum(
                        smeared[start_idx : i + 1], closer_dist)

        return smeared

    def pick_target(self):
        """Find the center angle and distance of the largest open path"""
        if float(np.max(self.scan_ranges)) <= 0.0:
            return 0.0, 0.0

        kernel_size = min(15, len(self.scan_ranges))
        smoothed = np.convolve(self.scan_ranges, np.ones(kernel_size) / kernel_size, mode='same')
        peak = float(np.max(smoothed))
        candidates = np.flatnonzero(smoothed >= 0.92 * peak)
        best_idx = int(candidates[len(candidates) // 2])

        return float(self.scan_angles[best_idx]), float(self.scan_ranges[best_idx])

    def open_control_loop(self):
        if self.state in ('STANDBY', 'FINISHED'):
            return

        front_clear = 2.50
        if self.raw_ranges.size > 0:
            front_mask = np.abs(self.scan_angles) <= math.radians(15.0)
            if np.any(front_mask):
                front_clear = float(np.min(self.raw_ranges[front_mask]))

        if self.state == 'COUNTDOWN':
            if (time.time() - self.countdown_start) < 5.0:
                return
            self.total_yaw_deg = 0.0
            # Log distance to front wall normalized
            self.start_front_dist = float(np.clip(front_clear, 0.60, 1.60))
            self.state = 'RUNNING'
            self.get_logger().info(f'RUNNING (starting front wall dist: {self.start_front_dist:.2f}m).')

        now_ms = int(time.time() * 1000)
        if self.lidar_last_timestamp is None or (now_ms - self.lidar_last_timestamp) > 1200:
            self.get_logger().warn('LIDAR data not receiving. Stopping.', throttle_duration_sec=1.0)
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            return

        if self.state == 'RUNNING' and self.scan_ranges.size > 0:
            if abs(self.total_yaw_deg) >= self.target_laps_yaw and front_clear <= self.start_front_dist:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)
                self.state = 'FINISHED'
                self.get_logger().info(
                    f'FINISHED 3 LAPS! Yaw={self.total_yaw_deg:+.1f} deg, '
                    f'stopped on starting side (front wall {front_clear:.2f}m).')
                return

            # Trying to keep the BOT in the middle (no wall hugging)
            target_angle, target_dist = self.pick_target()

            right_mask = (self.scan_angles >= math.radians(-80.0)) & (self.scan_angles <= math.radians(-25.0))
            left_mask = (self.scan_angles >= math.radians(25.0)) & (self.scan_angles <= math.radians(80.0))
            right_dist = float(np.min(self.raw_ranges[right_mask])) if np.any(right_mask) else 1.0
            left_dist = float(np.min(self.raw_ranges[left_mask])) if np.any(left_mask) else 1.0

            wall_push = 0.0
            if right_dist < self.wall_safe_dist:
                wall_push += (self.wall_safe_dist - right_dist) * 2.5   # push left
            if left_dist < self.wall_safe_dist:
                wall_push -= (self.wall_safe_dist - left_dist) * 2.5    # push right

            steer = float(np.clip((target_angle / self.max_steer_rad) + wall_push, -1.0, 1.0))
            speed = self.speed_straight + abs(steer) * (self.speed_turn - self.speed_straight)

            self.vel.linear.x = float(speed)
            self.vel.angular.z = steer
            self.pub.publish(self.vel)

            laps_done = abs(self.total_yaw_deg) / 360.0
            self.get_logger().info(
                f'Lap {laps_done:.2f}/3.00 ({self.total_yaw_deg:+6.1f}°) | '
                f'Target: {math.degrees(target_angle):+5.1f}° | '
                f'Front: {front_clear:.2f}m | L/R: {left_dist:.2f}/{right_dist:.2f}m | '
                f'steer={steer:+.2f}',
                throttle_duration_sec=0.4) 

def main(args=None):
    rclpy.init(args=args)
    node = OpenRound()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()