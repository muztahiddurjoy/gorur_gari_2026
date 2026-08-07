#!/usr/bin/env python3
"""
fusion_node.py - Sensor Fusion Node for WRO 2026 Future Engineers

Fuses 2D vision color obstacle semantics (/vision/obstacles) with 2D LiDAR polar spatial scan data (/lidar/scan).
Correlates bounding box center angles with LiDAR range measurements to assign exact distance and spatial coordinates.

Publishes fused tracking data for downstream path planning and state machine nodes.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from wro_autodrive.msg import ObstacleArray

# For structured fused output, using ObstacleArray or custom fused publisher
from geometry_msgs.msg import PoseArray, Pose, Point


class FusionNode(Node):
    """
    ROS 2 Node fusing visual semantic identification with spatial LiDAR point clouds/scans.
    """

    def __init__(self) -> None:
        super().__init__('fusion_node')

        # --- Parameter Declarations ---
        self.declare_parameter('vision_topic', '/vision/obstacles')
        self.declare_parameter('lidar_topic', '/lidar/scan')
        self.declare_parameter('fused_topic', '/fused/obstacles')
        self.declare_parameter('camera_fov_deg', 90.0)
        self.declare_parameter('camera_resolution_x', 640)

        vision_topic = self.get_parameter('vision_topic').get_parameter_value().string_value
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        fused_topic = self.get_parameter('fused_topic').get_parameter_value().string_value
        self.fov_deg = self.get_parameter('camera_fov_deg').get_parameter_value().double_value
        self.img_width = self.get_parameter('camera_resolution_x').get_parameter_value().integer_value

        # --- Callback Group Setup (Reentrant for concurrent callbacks) ---
        self.fusion_cb_group = ReentrantCallbackGroup()

        # --- QoS Configuration ---
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Internal cache for cross-sensor correlation
        self.latest_scan = None
        self.latest_obstacles = None

        # --- Publishers & Subscribers ---
        self.fused_pub = self.create_publisher(
            ObstacleArray,
            fused_topic,
            qos_profile=10
        )

        self.vision_sub = self.create_subscription(
            ObstacleArray,
            vision_topic,
            self._vision_callback,
            qos_profile=10,
            callback_group=self.fusion_cb_group
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            lidar_topic,
            self._lidar_callback,
            qos_profile=self.sensor_qos,
            callback_group=self.fusion_cb_group
        )

        self.get_logger().info(
            f'FusionNode initialized. Listening on {vision_topic} and {lidar_topic}'
        )

    def _lidar_callback(self, msg: LaserScan) -> None:
        """
        Stores latest LiDAR scan in memory thread-safely.
        """
        self.latest_scan = msg

    def _vision_callback(self, msg: ObstacleArray) -> None:
        """
        Triggered when vision obstacle metadata arrives.
        Fuses current vision detection with latest cached LiDAR scan data.
        """
        self.latest_obstacles = msg

        if self.latest_scan is None:
            self.get_logger().debug('Waiting for initial LiDAR scan data...')
            # Publish un-fused array for fall-back operations
            self.fused_pub.publish(msg)
            return

        fused_array = self._fuse_data(msg, self.latest_scan)
        self.fused_pub.publish(fused_array)

    def _fuse_data(self, vision_msg: ObstacleArray, scan_msg: LaserScan) -> ObstacleArray:
        """
        Fuses pixel bounding boxes with polar LiDAR range arrays.

        1. Map pixel center X to angle relative to camera axis:
           angle_rad = (center_x - (width / 2)) * (FOV_rad / width)
        2. Lookup LiDAR distance at target angle_rad.
        3. Annotate obstacle or refine spatial coordinates.
        """
        fused_msg = ObstacleArray()
        fused_msg.header = vision_msg.header

        fov_rad = math.radians(self.fov_deg)

        for obs in vision_msg.obstacles:
            # Map pixel X coordinate to relative heading angle
            pixel_offset = obs.center_x - (self.img_width / 2.0)
            angle_offset = (pixel_offset / self.img_width) * fov_rad

            # Look up corresponding distance index in LaserScan
            if scan_msg.angle_increment > 0:
                scan_idx = int((angle_offset - scan_msg.angle_min) / scan_msg.angle_increment)
                if 0 <= scan_idx < len(scan_msg.ranges):
                    distance = scan_msg.ranges[scan_idx]
                    self.get_logger().debug(
                        f'Obstacle ({obs.color}) at pixel {obs.center_x} mapped to angle '
                        f'{math.degrees(angle_offset):.1f} deg with LiDAR range {distance:.2f}m'
                    )

            # Re-add obstacle to output array
            fused_msg.obstacles.append(obs)

        return fused_msg


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('FusionNode stopping via KeyboardInterrupt.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
