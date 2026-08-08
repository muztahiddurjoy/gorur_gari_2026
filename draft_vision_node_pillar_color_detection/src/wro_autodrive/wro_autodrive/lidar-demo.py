import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import math

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("LidarProcessor node started. Waiting for messages on topic '/scan'...")

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_points = len(ranges)
        if num_points == 0:
            return

        clean_ranges = [x if not (math.isinf(x) or math.isnan(x)) else msg.range_max for x in ranges]
        
        # Calculate dynamic indices based on actual scan length
        def deg_to_idx(deg):
            return int((deg % 360) / 360.0 * num_points)

        front_indices = [deg_to_idx(d) for d in range(-15, 15)]
        left_indices = [deg_to_idx(d) for d in range(75, 105)]
        right_indices = [deg_to_idx(d) for d in range(255, 285)]

        front_dist = min([clean_ranges[i] for i in front_indices if i < num_points])
        left_dist = min([clean_ranges[i] for i in left_indices if i < num_points])
        right_dist = min([clean_ranges[i] for i in right_indices if i < num_points])

        print(f"Front: {front_dist:.2f}m | Left: {left_dist:.2f}m | Right: {right_dist:.2f}m")
        raise SystemExit(0)

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()