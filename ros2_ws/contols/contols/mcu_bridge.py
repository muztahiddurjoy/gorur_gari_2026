import geometry_msgs
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import MAVLink_gorur_gari_mcu_to_ros2_msg_message
from geometry_msgs.msg import Twist

class MCUBridgeNode(Node):
    def __init__(self):
        super().__init__('mcu_bridge')
        self.port = '/dev/ttyUSB0' 
        self.baudrate = 115200
        self.get_logger().info(f'Connecting to MCU on {self.port} at {self.baudrate} baud.')
        self.mcu_connected = False;
        
        try:
            self.master = gorur_gari_mavlink_msg.MAVLink(self.port, self.baudrate)
            self.get_logger().info('Successfully connected to MCU.')
            self.mcu_connected = True
            self.cmd_vel = self.create_subscription(Twist,'/cmd_vel', self.handle_cmd_vel, 10)
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MCU: {e}')
            rclpy.shutdown()
            return
        self.timer = self.create_timer(0.1, self.send_heartbeat)  # Send heartbeat every 0.1 seconds
        
    def handle_cmd_vel(self,msg:Twist):
        try:
            if not self.mcu_connected: #handle the case where the MCU is not connected
                self.get_logger().error("MCU is not connected. cannot send command.")
                return;
            throttle = msg.linear.x
            steering = msg.angular.z
            self.get_logger().info(f'Sending cmd_vel to MCU: throttle={throttle}, steering={steering}')
            self.master.gorur_gari_serial_msg_send(throttle, steering)
        except Exception as e:
            self.get_logger().error(f'Error in handle_cmd_vel: {e}')
            return
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