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

        self.declare_parameter('direction_latch_deg', 35.0)
        self.declare_parameter('steer_alpha', 0.45)  # 0.0 = frozen, 1.0 = raw | NOT SO MUCH SENSITIVE

        self.lap_direction = None  # None -> 'CW' or 'CCW' after first turn
        self.prev_steer = 0.0
        #===========================#

        self.declare_parameter('fov_min_deg', -100.0)
        self.declare_parameter('fov_max_deg', 100.0)
        self.declare_parameter('max_range', 2.50)
        self.declare_parameter('car_radius', 0.18)
        self.declare_parameter('disparity_thresh', 0.12)
        self.declare_parameter('wall_safe_dist', 0.35)
        
        self.declare_parameter('target_laps_yaw', 1080.0) # (360*3=1080)

        # Steering controls
        self.declare_parameter('speed_straight', 1.20)
        self.declare_parameter('speed_turn', 1.0)           
        self.declare_parameter('max_steer_deg', 22.0)       
        self.declare_parameter('turn_entry_dist', 0.30)
        self.declare_parameter('center_kp', 1.8)             # Proportional gain for straight centering

        # ── Read Parameters ──
        self.fov_min_deg = float(self.get_parameter('fov_min_deg').value)
        self.fov_max_deg = float(self.get_parameter('fov_max_deg').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.car_radius = float(self.get_parameter('car_radius').value)
        self.disparity_thresh = float(self.get_parameter('disparity_thresh').value)
        self.wall_safe_dist = float(self.get_parameter('wall_safe_dist').value)
        self.speed_straight = float(self.get_parameter('speed_straight').value)
        self.speed_turn = float(self.get_parameter('speed_turn').value)
        self.max_steer_rad = math.radians(float(self.get_parameter('max_steer_deg').value))

        self.direction_latch_deg = float(self.get_parameter('direction_latch_deg').value)
        self.steer_alpha = float(self.get_parameter('steer_alpha').value)
        self.turn_entry_dist = float(self.get_parameter('turn_entry_dist').value)
        self.center_kp = float(self.get_parameter('center_kp').value)


        self.target_laps_yaw = float(self.get_parameter('target_laps_yaw').value)
        self.start_front_dist = None

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
        if msg.data and self.state in ('STANDBY', 'FINISHED'):
            self.state = 'COUNTDOWN'
            self.countdown_start = time.time()
            self.total_yaw_deg = 0.0
            self.prev_steer = 0.0
            self.get_logger().info('Button pressed! Starting Open Round in 5.0s:::::')
            self.lap_direction = None

    def heading_callback(self, msg):
        raw = float(msg.data)
        if self.prev_raw_heading is not None:
            # Fix: 359 to 1 deg is +2 deg, not -358 deg
            delta = (raw - self.prev_raw_heading + 180.0) % 360.0 - 180.0
            if abs(delta) < 45.0:  # Ignore IMU glitches
                self.total_yaw_deg += delta
        #experimental
        if (self.state == 'RUNNING'
                and self.lap_direction is None
                and abs(self.total_yaw_deg) >= self.direction_latch_deg):
            self.lap_direction = 'CW' if self.total_yaw_deg > 0.0 else 'CCW'
            self.get_logger().info(f'Direction latched: {self.lap_direction} (yaw={self.total_yaw_deg:+.1f}°)')
        #=====================#  
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
        """Find the center angle and distance of the largest allowed open path."""
        if self.lap_direction == 'CW':
            valid_mask = (self.scan_angles >= math.radians(-100.0)) & (self.scan_angles <= math.radians(15.0))
        elif self.lap_direction == 'CCW':
            valid_mask = (self.scan_angles >= math.radians(-15.0)) & (self.scan_angles <= math.radians(100.0))
        else:
            valid_mask = (self.scan_angles >= math.radians(-75.0)) & (self.scan_angles <= math.radians(75.0))

        masked_ranges = self.scan_ranges.copy()
        masked_ranges[~valid_mask] = 0.0

        if float(np.max(masked_ranges)) <= 0.0:
            return 0.0, 0.0

        kernel_size = min(15, len(masked_ranges))
        smoothed = np.convolve(masked_ranges, np.ones(kernel_size) / kernel_size, mode='same')
        peak = float(np.max(smoothed))
        candidates = np.flatnonzero(smoothed >= 0.92 * peak)
        best_idx = int(candidates[len(candidates) // 2])

        return float(self.scan_angles[best_idx]), float(self.scan_ranges[best_idx])
    
    def measure_side_wall(self, angle_center_deg, window_deg=20.0):
        """Return the 25th-percentile distance in a window around a side angle."""
        half_w = window_deg / 2.0
        mask = (self.scan_angles >= math.radians(angle_center_deg - half_w)) & \
               (self.scan_angles <= math.radians(angle_center_deg + half_w))
        if not np.any(mask):
            return 1.5
        vals = self.raw_ranges[mask]
        valid = vals[vals > 0.05]
        if valid.size == 0:
            return 1.5
        return float(np.percentile(valid, 25))
    
    
    
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
            #### Experimental
            # 1. Measure side walls using our 25th-percentile helper function (±80° to ±100°)
            left_dist = self.measure_side_wall(75.0, window_deg=30.0)
            right_dist = self.measure_side_wall(-75.0, window_deg=30.0)
            target_angle, _ = self.pick_target()

            # 2. Always-on inner/outer wall safety push
            wall_push = 0.0
            if right_dist < self.wall_safe_dist:
                wall_push += (self.wall_safe_dist - right_dist) * 2.2   # push left (+)
            if left_dist < self.wall_safe_dist:
                wall_push -= (self.wall_safe_dist - left_dist) * 2.2    # push right (-)

            is_turning = (front_clear <= self.turn_entry_dist)

            if not is_turning:
                # ── STRAIGHT MODE ──
                outer_is_right = (self.lap_direction != 'CW')

                if left_dist < 0.85 and right_dist < 0.85:
                    error = left_dist - right_dist
                elif outer_is_right and right_dist < 0.85:
                    error = (0.50 - right_dist) * 1.5
                elif (not outer_is_right) and left_dist < 0.85:
                    error = (left_dist - 0.50) * 1.5
                else:
                    error = 0.0

                # Filter the target_angle to ±20° so the car runs straight after a turn
                # stopping the next, too turn early
                straight_align = np.clip(target_angle, math.radians(-20.0), math.radians(20.0)) / self.max_steer_rad
                center_trim = np.clip(+self.center_kp * error, -0.45, 0.45)

                target_steer = np.clip(0.55 * straight_align + center_trim + wall_push, -0.75, 0.75)
                target_speed = self.speed_straight

            else:
                # ── CORNER MODE ──
                norm_angle = np.clip(target_angle / self.max_steer_rad, -1.0, 1.0)
                # 1.35 exponent: softer than linear at turn entry, strong enough mid-corner
                curve_steer = math.copysign(abs(norm_angle) ** 1.35, norm_angle)
                target_steer = np.clip(curve_steer + wall_push, -0.85, 0.85)
                target_speed = self.speed_turn

            # 3. Smooth steering transition
            steer = float(self.steer_alpha * target_steer + (1.0 - self.steer_alpha) * self.prev_steer)
            self.prev_steer = steer
            speed = float(target_speed)

            self.vel.linear.x = float(speed)
            self.vel.angular.z = steer
            self.pub.publish(self.vel)
            
            
            
            mode_str = 'CORNER' if is_turning else 'STRAIGHT'
            laps_done = abs(self.total_yaw_deg) / 360.0
            self.get_logger().info(
                f'[{mode_str}] Lap {laps_done:.2f}/3.00 ({self.total_yaw_deg:+6.1f}°) | '
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