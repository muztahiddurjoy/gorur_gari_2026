import rclpy
from rclpy.node import Node
from math import pi, sin, cos
from std_msgs.msg import Float32,Int32

class VectorOdometryNode(Node):
    def __init__(self):
        super().__init__('vector_odometry')
        self.get_logger().info('Vector Odometry node initialized.')
        self.tick_sub = self.create_subscription(Int32, 'encoder/count', self.tick_callback, 10)
        self.heading_sub = self.create_subscription(Float32, 'heading', self.heading_callback, 10)
        self.current_tick = 0;
        self.current_yaw = 0.0;
        # odom states
        self.last_local_x = 0.0
        self.last_local_y = 0.0

        self.last_global_x = 0.0
        self.last_global_y = 0.0


        self.wheel_diameter = 65
        self.counts_per_rev = 1000
        self.gear_ratio = 1.0
    
        

    def tick_callback(self, msg: Int32):
        self.get_logger().info(f'Received tick: {msg.data}')
        self.last_local_x = self.current_tick
        self.current_tick = msg.data
        del_tick = self.current_tick - self.last_local_x
        del_distance = del_tick * ((pi * self.wheel_diameter) / (self.counts_per_rev* self.gear_ratio))
        
    def heading_callback(self, msg: Float32):
        self.current_yaw = int(msg.data)*(pi/180.0)  # Convert degrees to radians
        self.get_logger().info(f'Received yaw: {self.current_yaw} radians')

def main(args=None):
    rclpy.init(args=args)
    vector_odom_node = VectorOdometryNode()
    try:
        rclpy.spin(vector_odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        vector_odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()