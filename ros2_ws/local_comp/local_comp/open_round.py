import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
class OpenRound(Node):
    def __init__(self):
        super().__init__('open_round')
        self.get_logger().info('OpenRound node has been started.')
        self.create_subscription(Bool,'/button_status',self.button_callback,10)
        
    def button_callback(self,msg):
        if msg.data:
            self.get_logger().info('Button pressed. Starting the open round process.')
            self.start_open_round()

    def start_open_round(self):
        self.get_logger().info('Starting the timer')
        # Add your logic here to start the open round process


def main(args=None):
    rclpy.init(args=args)
    node = OpenRound()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()