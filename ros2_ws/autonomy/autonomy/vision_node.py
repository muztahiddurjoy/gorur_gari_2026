#!/usr/bin/env python3
"""
vision_node.py - OpenCV Vision Processing Node for WRO 2026 Future Engineers

Subscribes to raw camera image stream, performs color mask filtering (Red & Green traffic pillars),
and publishes identified obstacles to /vision/obstacles.

Designed with non-blocking callback groups and sensor-optimized QoS for low latency.
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

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
    from autonomy.pillar_detector import PillarDetector
except ModuleNotFoundError:
    from pillar_detector import PillarDetector


class VisionNode(Node):
    """
    ROS 2 Vision Node responsible for color segmentation and obstacle feature extraction.
    """

    def __init__(self) -> None:
        super().__init__('vision_node')
        
        self.declare_parameter('efficient_mode', True)         # Toggle CPU optimization vs Full Realtime Debug mode
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('obstacles_topic', '/closest_obj')
        self.declare_parameter('min_contour_area', 250)
        self.declare_parameter('roi_top_crop', 0.40)
        self.declare_parameter('publish_debug_image', True)  # Auto-overridden to True when efficient_mode is False
        self.declare_parameter('process_every_n_frames', 2)   # Throttle processing when efficient_mode is True
        self.declare_parameter('processing_width', 320)       # Downscale resolution when efficient_mode is True
        self.declare_parameter('processing_height', 240)
        self.declare_parameter('camera_hfov_deg', 60.0)       # Camera Horizontal FOV for angular matching
        
        self.declare_parameter('min_aspect_ratio', 1.0)
        self.declare_parameter('max_aspect_ratio', 3.5)
        self.declare_parameter('min_solidity', 0.40)

        # --- HSV Parameter Declarations ---
        self.declare_parameter('green_lower', [35, 80, 0])
        self.declare_parameter('green_upper', [85, 255, 255])
        self.declare_parameter('red_lower_1', [0, 95, 50])
        self.declare_parameter('red_upper_1', [12, 255, 255])
        self.declare_parameter('red_lower_2', [165, 95, 50])
        self.declare_parameter('red_upper_2', [180, 255, 255])
        
        self.efficient_mode = bool(self.get_parameter('efficient_mode').value)
        camera_topic = self.get_parameter('camera_topic').value
        obstacles_topic = self.get_parameter('obstacles_topic').value
        self.camera_hfov_deg = self.get_parameter('camera_hfov_deg').value
        min_area = self.get_parameter('min_contour_area').get_parameter_value().integer_value
        roi_crop = self.get_parameter('roi_top_crop').get_parameter_value().double_value
        min_ar = float(self.get_parameter('min_aspect_ratio').value)
        max_ar = float(self.get_parameter('max_aspect_ratio').value)
        min_sol = float(self.get_parameter('min_solidity').value)

        # When efficient_mode is False, auto-enable debug image for real-time visualization
        if not self.efficient_mode:
            self.publish_debug = True
        else:
            self.publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value

        self.frame_skip_n = int(self.get_parameter('process_every_n_frames').value)
        self.target_w = int(self.get_parameter('processing_width').value)
        self.target_h = int(self.get_parameter('processing_height').value)

        self.frame_counter = 0

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

        # --- OpenCV Pillar Detector ---
        import numpy as np
        self.detector = PillarDetector(
            min_area=min_area,
            roi_top_crop=roi_crop,
            min_aspect_ratio=min_ar,
            max_aspect_ratio=max_ar,
            min_solidity=min_sol,
            green_lower=np.array(self.get_parameter('green_lower').value, dtype=np.uint8),
            green_upper=np.array(self.get_parameter('green_upper').value, dtype=np.uint8),
            red_lower_1=np.array(self.get_parameter('red_lower_1').value, dtype=np.uint8),
            red_upper_1=np.array(self.get_parameter('red_upper_1').value, dtype=np.uint8),
            red_lower_2=np.array(self.get_parameter('red_lower_2').value, dtype=np.uint8),
            red_upper_2=np.array(self.get_parameter('red_upper_2').value, dtype=np.uint8),
        )

        # --- Publishers & Subscribers ---
        # ── Publishers ────────────────────────────────────────────────
        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None

        self.obstacle_pub = self.create_publisher(
            Float32MultiArray,
            obstacles_topic,
            qos_profile=self.sensor_qos
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
        # 1. Frame Skipping / Throttling Optimization (Only active when efficient_mode is True)
        self.frame_counter += 1
        if self.efficient_mode and (self.frame_counter % max(1, self.frame_skip_n) != 0):
            return

        if not CV_BRIDGE_AVAILABLE:
            self.get_logger().warn_once('cv_bridge is not available! Cannot convert ROS Image to OpenCV format.')
            return

        try:
            # Convert ROS Image message to OpenCV BGR frame
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image frame: {str(e)}')
            return

        # 2. Resolution Downscaling / Hard-Limiting Optimization (Only active when efficient_mode is True)
        h, w, _ = cv_image.shape
        if self.efficient_mode and (w > self.target_w or h > self.target_h):
            cv_image = cv2.resize(cv_image, (self.target_w, self.target_h), interpolation=cv2.INTER_NEAREST)

        # Run the standalone OpenCV detection pipeline
        detections = self.detector.detect(cv_image)

        # Send only the color codes of detected towers, sorted by closest (largest area) to furthest
        sorted_dets = sorted(detections, key=lambda d: d['bbox'][2] * d['bbox'][3], reverse=True)
        
        msg_out = Float32MultiArray()
        msg_out.data = [float(1.0 if det['color'] == 'red' else 2.0) for det in sorted_dets]

        self.obstacle_pub.publish(msg_out)
        self.get_logger().debug(f'Published tower colors: {msg_out.data}')

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
