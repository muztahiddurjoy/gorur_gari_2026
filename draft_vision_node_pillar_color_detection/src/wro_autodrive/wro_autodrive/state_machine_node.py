#!/usr/bin/env python3
"""
state_machine_node.py - Finite State Machine (FSM) Node for WRO 2026 Future Engineers

Tracks match state machine: START -> LAP_1 -> LAP_2 -> LAP_3 -> PARKING.
Processes fused environmental perception data and outputs path planning targets to drive controller.
"""

from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from wro_autodrive.msg import ObstacleArray


class VehicleState(Enum):
    """FSM States according to WRO 2026 Competition Rules."""
    START = "START"
    LAP_1 = "LAP_1"
    LAP_2 = "LAP_2"
    LAP_3 = "LAP_3"
    PARKING = "PARKING"
    STOPPED = "STOPPED"


class StateMachineNode(Node):
    """
    ROS 2 Finite State Machine node controlling high-level vehicle behavior and mission sequence.
    """

    def __init__(self) -> None:
        super().__init__('state_machine_node')

        # --- Parameter Declarations ---
        self.declare_parameter('fused_obstacles_topic', '/fused/obstacles')
        self.declare_parameter('goal_topic', '/planner/drive_goal')
        self.declare_parameter('state_topic', '/robot/state')
        self.declare_parameter('target_laps', 3)
        self.declare_parameter('fsm_loop_rate_hz', 20.0)

        fused_topic = self.get_parameter('fused_obstacles_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        loop_rate = self.get_parameter('fsm_loop_rate_hz').get_parameter_value().double_value

        # --- Internal State Initialization ---
        self.current_state = VehicleState.START
        self.lap_count = 0
        self.orange_lines_passed = 0

        # --- Callback Groups ---
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        # --- Publishers & Subscribers ---
        self.goal_pub = self.create_publisher(
            Twist,
            goal_topic,
            qos_profile=10
        )

        self.state_pub = self.create_publisher(
            String,
            state_topic,
            qos_profile=10
        )

        self.fused_sub = self.create_subscription(
            ObstacleArray,
            fused_topic,
            self._fused_obstacles_callback,
            qos_profile=10,
            callback_group=self.sub_cb_group
        )

        # Periodic FSM evaluation loop timer
        timer_period = 1.0 / loop_rate
        self.fsm_timer = self.create_timer(
            timer_period,
            self._fsm_loop,
            callback_group=self.timer_cb_group
        )

        self.get_logger().info(f'StateMachineNode initialized in state: {self.current_state.value}')

    def _fused_obstacles_callback(self, msg: ObstacleArray) -> None:
        """
        Callback processing incoming obstacle data for navigation decision making.
        """
        # Count pillars or track position relative to red/green pillars
        red_count = sum(1 for obs in msg.obstacles if obs.color == 'red')
        green_count = sum(1 for obs in msg.obstacles if obs.color == 'green')

        self.get_logger().debug(f'FSM perception input: Red={red_count}, Green={green_count}')

    def _fsm_loop(self) -> None:
        """
        Periodic state transition & action execution loop.
        """
        # Publish current state for monitoring/telemetry
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)

        goal_cmd = Twist()

        # --- FSM State Handler Blueprint ---
        if self.current_state == VehicleState.START:
            self.get_logger().info('Transitioning from START -> LAP_1')
            self._transition_to(VehicleState.LAP_1)

        elif self.current_state == VehicleState.LAP_1:
            goal_cmd.linear.x = 0.5  # Target linear speed m/s
            goal_cmd.angular.z = 0.0 # Target steering angle / curvature
            
            # Example lap completion logic:
            # if lap_finished_event:
            #     self._transition_to(VehicleState.LAP_2)

        elif self.current_state == VehicleState.LAP_2:
            goal_cmd.linear.x = 0.55
            # if lap_finished_event:
            #     self._transition_to(VehicleState.LAP_3)

        elif self.current_state == VehicleState.LAP_3:
            goal_cmd.linear.x = 0.55
            # if lap_finished_event:
            #     self._transition_to(VehicleState.PARKING)

        elif self.current_state == VehicleState.PARKING:
            goal_cmd.linear.x = 0.2
            goal_cmd.angular.z = 0.3 # Maneuver into parking bay
            # if parked:
            #     self._transition_to(VehicleState.STOPPED)

        elif self.current_state == VehicleState.STOPPED:
            goal_cmd.linear.x = 0.0
            goal_cmd.angular.z = 0.0

        # Publish target goal command for drive controller node
        self.goal_pub.publish(goal_cmd)

    def _transition_to(self, new_state: VehicleState) -> None:
        """
        State transition logger & handler.
        """
        self.get_logger().info(f'FSM State Transition: {self.current_state.value} -> {new_state.value}')
        self.current_state = new_state


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('StateMachineNode stopping via KeyboardInterrupt.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
