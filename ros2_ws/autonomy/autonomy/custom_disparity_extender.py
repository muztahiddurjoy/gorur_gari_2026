from cv2 import line

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np 

class CustomDisparityExtender(Node):
    def __init__(self):
        super().__init__('disparity_checker_test')
        self.sub = self.create_subscription(LaserScan,'/scan',self.lidr_callback, 10)
        self.min_ang = -math.pi
        self.max_ang = math.pi
        self.ang_inc = math.radians(0.5)
        self.ranges = []
        self.intensities = []
        self.spike_threshold = 0.1
        self.get_logger().info(f"the angle for 0th index will be {math.degrees(self.i2a(0))} and for last index will be {math.degrees(self.i2a(719))}")
        
    def lidr_callback(self, msg:LaserScan):
        self.ranges = np.array(msg.ranges, dtype=np.float32)
        self.intensities = np.array(msg.intensities, dtype=np.float32)
        self.ranges = np.nan_to_num(self.ranges, nan=math.inf, posinf=0, neginf=0)
        self.ang_inc = msg.angle_increment
        start = self.a2i(-math.radians(20))
        end = self.a2i(math.radians(20))

        for i in range (start, end):
            if self.ranges[i] > 0.01 and abs(self.ranges[i]-self.ranges[i+1]) > self.spike_threshold:
                self.get_logger().info(f"Spike detected at angle: {math.degrees(self.i2a(i))}")

        # line = ", ".join(
        #     f"{self.ranges[i]:.2f}" if self.ranges[i] != math.inf else "inf"
        #     for i in range(start, end)
        # )

        # self.get_logger().info(line)
        #self.get_logger().info(f"Received LaserScan with {len(self.ranges)} ranges and {len(self.intensities)} intensities and angle increment {self.ang_inc}")

    def a2i(self, angle:float):
        return int((angle - self.min_ang) / self.ang_inc)

    def i2a(self, index:int):
        return self.min_ang + index * self.ang_inc

def main(args=None):
    rclpy.init(args=args)
    node = CustomDisparityExtender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()