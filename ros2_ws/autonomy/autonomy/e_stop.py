import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np


class EStopNode(Node):
    def __init__(self):
        super().__init__("estop_node")
        self.cmd_vel = Twist()
        self.vel_status = False
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.status_pub = self.create_publisher(Bool,'estop_status',10)
        self.status_sub = self.create_subscription(Bool,'estop_request',self.estop_status_request, 10)
        self.vel_timer = self.create_timer(0.2, self.vel_sender_timer)

    def lidar_callback(self,scan_msg:LaserScan):
        ranges = np.nan_to_num(np.array(scan_msg),neginf=0,posinf=0)
        return;

    def estop_status_request(self,request:Bool):
        self.vel_status
        return;

    def vel_sender_timer(self):
        if self.vel_status == False:
            return
        self.cmd_vel.linear.x = 0.0
        self.vel_pub.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    estop_node = EStopNode()
    rclpy.spin(estop_node)
    estop_node.destroy_node()


if __name__ == '__main__':
    main()
