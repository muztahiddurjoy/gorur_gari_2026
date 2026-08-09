import rclpy
from rclpy.node import Node
from math import pi, sin, cos
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Vector3,Twist

class BoxLapTestNode(Node):
    def __init__(self):
        super().__init__('box_lap_test')
        self.get_logger().info('Box Lap Test node initialized.')
        self.sub = self.create_subscription(Vector3, 'odom', self.odom_callback, 10)
        self.heading = self.create_subscription(Float32, 'heading', self.heading_callback, 10)
        self.box_h = 95/100  # metres, 95cm box
        self.box_w = 50/100  # metres, 65cm box

        self.car_h = 0.32
        self.car_w = 0.21
        self.heading = 0.0
        self.expected_heading = 0.0
        self.heading_tolerance = 2  # degrees (error below is computed in degrees)
        self.vel_msg = Twist()
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
    def odom_callback(self, msg:Vector3):
       # self.get_logger().info(f'odom: x={msg.x:.3f} y={msg.y:.3f} distance={msg.z:.3f}')
        
        if(msg.x < self.box_h+(10/100) and msg.y < self.box_w+(10/100)):
            self.expected_heading = 0.0
            self.vel_msg.linear.x = 1.5
            self.get_logger().info("on step 1")
        elif(msg.x > self.box_h and msg.y < self.box_w):
            self.expected_heading = 90.0
            self.vel_msg.linear.x = 1.5
            self.get_logger().info("on step 2")
        elif(msg.y > self.box_w and msg.x > self.car_h+(12/100)):
            self.expected_heading = 180.0
            self.vel_msg.linear.x = 1.5
            self.get_logger().info("on step 3")
        elif(msg.x < self.car_h and msg.y > self.box_w+(10/100)):
            self.expected_heading = 270.0
            self.vel_msg.linear.x = 1.5
            self.get_logger().info("on step 4")

        # elif(msg.x > self.box_h+(12/100)):
        #    # self.expected_heading = 180.0
        #     self.vel_msg.linear.x = 1.01
        # elif(msg.x < self.box_w+(12/100)):
        #     self.expected_heading = 270.0
        #     self.vel_msg.linear.x = 1.01
        else:
            self.expected_heading = 0.0
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
        # self.get_logger().info(f'cmd_vel: linear.x={self.vel_msg.linear.x:.3f} angular.z={self.vel_msg.angular.z:.3f}')
        self.stabilize_steer()
        self.pub.publish(self.vel_msg)

    def heading_callback(self, msg:Float32):
        self.heading = msg.data
       # self.get_logger().info(f'heading: {self.heading:.3f}')

    def stabilize_steer(self):
        # signed error, positive = need to turn heading up toward expected
        error = self.expected_heading - self.heading
        # wrap to [-180, 180] so 350° vs 10° reads as +20°, not -340°
        error = (error + 180) % 360 - 180

        if abs(error) < self.heading_tolerance:
            self.vel_msg.angular.z = 0.0
            return

        kp = 0.03  # tune this: angular.z output per degree of heading error
        angular_z = kp * error

        max_angular_z = 0.3  # cap so stabilization doesn't yank full steering lock
        angular_z = max(-max_angular_z, min(max_angular_z, angular_z))

        self.vel_msg.angular.z = -angular_z
        
def main(args=None):
    rclpy.init(args=args)
    node = BoxLapTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()