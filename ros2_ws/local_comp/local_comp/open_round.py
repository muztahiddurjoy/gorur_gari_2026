import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
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
        # Field of view kept from each scan, in degrees (0 = straight ahead).
        self.fov_min_deg = -135.0
        self.fov_max_deg = 135.0
        self.scan_angles = np.array([])   # radians, within the FOV
        self.scan_ranges = np.array([])   # metres, valid readings only
        self.button_pressed = False
        self.lidar_last_timestamp = None
        self.vel = Twist()
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool,'/button_status',self.button_callback,10)
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)

                
    def button_callback(self,msg):
        if msg.data and not self.button_pressed:
            self.button_pressed = True
            self.get_logger().info('Button pressed. Starting the open round process.')
            self.start_open_round()

    def scan_callback(self, msg):
        self.lidar_last_timestamp = int(time.time() * 1000) 
        self.scan = msg
        self.scan_angles, self.scan_ranges = self.filter_scan(msg)

    def filter_scan(self, msg):
        """Return (angles, ranges) for valid readings between -135 and 135 deg."""
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        # Wrap to [-pi, pi) so scans published as 0..2pi work too.
        angles = (angles + math.pi) % (2.0 * math.pi) - math.pi

        in_fov = ((angles >= math.radians(self.fov_min_deg))
                  & (angles <= math.radians(self.fov_max_deg)))
        valid = (np.isfinite(ranges)
                 & (ranges >= msg.range_min)
                 & (ranges <= msg.range_max))
        keep = in_fov & valid
        return angles[keep], ranges[keep]

    def start_open_round(self):
        self.get_logger().info('Starting the timer')
        # Add your logic here to start the open round process
        time.sleep(5)  # Simulating a delay for the open round process
        self.create_timer(0.1, self.open_round_loop)
        

    def open_round_loop(self):
        self.get_logger().info('Open round process is running.')
        if self.scan is not None:
            self.vel.linear.x = 0.0  # Move forward at a constant speed
            self.vel.angular.z = 0.0  # No rotation
        
        if self.lidar_last_timestamp is not None and (int(time.time() * 1000) - self.lidar_last_timestamp) > 1200:
            self.get_logger().warn('LIDAR data is stale. Stopping the robot.')
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.

        if self.scan_ranges.size > 0:
            i = int(np.argmin(self.scan_ranges))
            self.get_logger().info(
                f'{self.scan_ranges.size} points in FOV, closest '
                f'{self.scan_ranges[i]:.2f} m at '
                f'{math.degrees(self.scan_angles[i]):.1f} deg')
        
            
            
        self.pub.publish(self.vel)
        

def main(args=None):
    rclpy.init(args=args)
    node = OpenRound()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()