import rclpy
from rclpy.node import Node
from pymavlink import mavutil

class MCUBridgeNode(Node):
    def __init__(self):
        super().__init__('mcu_bridge')
        self.port = '/dev/ttyUSB0'  # Update this to your MCU's serial port
        self.baudrate = 115200
        self.get_logger().info(f'Connecting to MCU on {self.port} at {self.baudrate} baud.')

        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baudrate)
            self.get_logger().info('Successfully connected to MCU.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MCU: {e}')
            rclpy.shutdown()
            return
        self.timer = self.create_timer(0.1, self.send_heartbeat)  # Send heartbeat every 0.1 seconds

    def send_heartbeat(self):
        msg = self.master.recv_match(blocking=False)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                self.get_logger().info('Received heartbeat from MCU.')
            if msg.get_type() == 'COMMAND_ACK':
                self.get_logger().info(f'Received command acknowledgment: {msg}')
            else:
                self.get_logger().info(f'Received message: {msg}')
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, 0
            )
            self.get_logger().info('Heartbeat sent to MCU.')
        except Exception as e:
            self.get_logger().error(f'Failed to send heartbeat: {e}')

def main(args=None):
    rclpy.init(args=args)
    mcu_bridge_node = MCUBridgeNode()
    try:
        rclpy.spin(mcu_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        mcu_bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()