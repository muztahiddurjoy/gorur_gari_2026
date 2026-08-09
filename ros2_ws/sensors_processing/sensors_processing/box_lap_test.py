import rclpy
from rclpy.node import Node
from math import pi, sin, cos
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Vector3, Twist
from sensors_processing.steering_stabilizer import SteeringStabilizer

class BoxLapTestNode(Node):
    def __init__(self):
        super().__init__('box_lap_test')
        self.get_logger().info('Box Lap Test node initialized.')
        self.sub = self.create_subscription(Vector3, 'odom', self.odom_callback, 10)
        self.heading_sub = self.create_subscription(Float32, 'heading', self.heading_callback, 10)
        self.box_h = 95/100  # metres, 95cm box
        self.box_w = 50/100  # metres, 65cm box

        self.car_h = 0.32
        self.car_w = 0.21
        self.heading = 0.0
        self.expected_heading = 0.0
        self.heading_tolerance = 2  # degrees (error below is computed in degrees)
        self.vel_msg = Twist()
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Steering Stabilizer interface
        self.stabilizer = SteeringStabilizer(kp=0.03, max_steer=0.3, tolerance_deg=2.0)
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
        # Calculate stabilized steering output
        self.vel_msg.angular.z = self.stabilizer.compute_steering(
            current_heading_deg=self.heading,
            target_heading_deg=self.expected_heading
        )
        self.pub.publish(self.vel_msg)

    def heading_callback(self, msg:Float32):
        self.heading = msg.data
        
def main(args=None):
    rclpy.init(args=args)
    node = BoxLapTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()