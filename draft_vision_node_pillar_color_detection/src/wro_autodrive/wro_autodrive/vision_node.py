#!/usr/bin/env python3
"""
vision_node.py - OpenCV Vision Processing Node for WRO 2026 Future Engineers

Subscribes to raw camera image stream, performs color mask filtering (Red & Green traffic pillars),
and publishes identified obstacles to /vision/obstacles.

Designed with non-blocking callback groups and sensor-optimized QoS for low latency.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from wro_autodrive.msg import Obstacle, ObstacleArray

# Import cv_bridge safely if installed, or fallback gracefully
try:
    from cv_bridge import CvBridge, CvBridgeError
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

import os
import sys

# Ensure local script directory is on sys.path for direct python3 invocation
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    from wro_autodrive.pillar_detector import PillarDetector
except ModuleNotFoundError:
    from pillar_detector import PillarDetector


class VisionNode(Node):
    """
    ROS 2 Vision Node responsible for color segmentation and obstacle feature extraction.
    """

    def __init__(self) -> None:
        super().__init__('vision_node')
        
        # --- Parameter Declarations ---
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('obstacles_topic', '/vision/obstacles')
        self.declare_parameter('min_contour_area', 300)
        self.declare_parameter('roi_top_crop', 0.5)
        self.declare_parameter('publish_debug_image', True)
        
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        obstacles_topic = self.get_parameter('obstacles_topic').get_parameter_value().string_value
        min_area = self.get_parameter('min_contour_area').get_parameter_value().integer_value
        roi_crop = self.get_parameter('roi_top_crop').get_parameter_value().double_value
        self.publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value

        # --- Callback Group Setup (Non-Blocking Concurrency) ---
        # Dedicated callback group so camera callbacks run independently without blocking other timers/subscriptions
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
        
        # --- Real-Time Sensor QoS Profile ---
        # Best-effort, volatile durability with depth 1 to prioritize low latency over delivery guarantees
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- OpenCV Pillar Detector (standalone, no ROS coupling) ---
        self.detector = PillarDetector(
            min_area=min_area,
            roi_top_crop=roi_crop,
        )

        # --- Publishers & Subscribers ---
        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None

        self.obstacle_pub = self.create_publisher(
            ObstacleArray,
            obstacles_topic,
            qos_profile=10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/vision/debug_image',
            qos_profile=self.sensor_qos
        )

        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self._image_callback,
            qos_profile=self.sensor_qos,
            callback_group=self.sensor_cb_group
        )

        self.get_logger().info(
            f'VisionNode initialized. Subscribed to {camera_topic}, publishing to {obstacles_topic}'
        )

    def _image_callback(self, msg: Image) -> None:
        """
        Callback triggered when a new camera frame is received.
        Processes frame using OpenCV HSV color masking via PillarDetector.
        """
        if not CV_BRIDGE_AVAILABLE:
            self.get_logger().warn_once('cv_bridge is not available! Cannot convert ROS Image to OpenCV format.')
            return

        try:
            # Convert ROS Image message to OpenCV BGR frame
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image frame: {str(e)}')
            return

        # Run the standalone OpenCV detection pipeline
        detections = self.detector.detect(cv_image)

        # Convert detection dicts to ROS 2 Obstacle messages
        obstacle_msgs = []
        for det in detections:
            obs = Obstacle()
            obs.color = det['color']
            obs.center_x = det['centroid_x']
            obs.width = det['bbox'][2]  # w from (x, y, w, h)
            obstacle_msgs.append(obs)

        # Construct and publish ObstacleArray message
        obstacle_array_msg = ObstacleArray()
        obstacle_array_msg.header = msg.header
        obstacle_array_msg.obstacles = obstacle_msgs

        self.obstacle_pub.publish(obstacle_array_msg)
        self.get_logger().debug(f'Published {len(obstacle_msgs)} obstacles.')

        # Publish visual debug frame if enabled
        if self.publish_debug and self.bridge:
            debug_frame = self.detector.draw_detections(cv_image, detections)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish debug image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    # Utilize MultiThreadedExecutor for scalable concurrency
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('VisionNode stopping via KeyboardInterrupt.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
