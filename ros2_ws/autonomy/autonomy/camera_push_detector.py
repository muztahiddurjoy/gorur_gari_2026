#!/usr/bin/env python3
"""
camera_push_detector.py — ROS 2 Vision Node for Bounding-Box-Based Vehicle Push Guidance

This node processes incoming camera frames, detects obstacle/pillar bounding boxes,
and determines whether the robot should be directed to steer ("push") LEFT, RIGHT, or stay CENTER.

WRO 2026 Rules & Avoidance Logic:
1. Color Rule:
   - Green Pillar -> Must pass on LEFT  -> Output: PUSH_LEFT
   - Red Pillar   -> Must pass on RIGHT -> Output: PUSH_RIGHT
2. Spatial Bounding Box Offset:
   - Obstacle on Left side of frame  -> Output: PUSH_RIGHT (steer away from left obstacle)
   - Obstacle on Right side of frame -> Output: PUSH_LEFT  (steer away from right obstacle)

Topics Published:
- /vision/push_direction (std_msgs/String): "PUSH_LEFT", "PUSH_RIGHT", or "KEEP_CENTER"
- /vision/push_debug_image (sensor_msgs/Image): Annotated image with bounding boxes & direction overlay
"""

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False


class CameraPushDetector(Node):

    def __init__(self):
        super().__init__('camera_push_detector')

        # ── Parameter Declarations ────────────────────────────────────
        self.declare_parameter('efficient_mode', True)         # Toggle CPU optimization vs Full Realtime Debug mode
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('push_direction_topic', '/vision/push_direction')
        self.declare_parameter('debug_image_topic', '/vision/push_debug_image')
        self.declare_parameter('min_contour_area', 100)
        self.declare_parameter('deadzone_px', 20)
        self.declare_parameter('publish_debug_image', False)  # Auto-overridden to True when efficient_mode is False
        self.declare_parameter('process_every_n_frames', 2)   # Throttle processing when efficient_mode is True
        self.declare_parameter('processing_width', 320)       # Downscale resolution when efficient_mode is True
        self.declare_parameter('processing_height', 240)

        # HSV Thresholds for Red and Green Pillars
        self.declare_parameter('green_lower', [35, 80, 50])
        self.declare_parameter('green_upper', [85, 255, 255])
        self.declare_parameter('red_lower_1', [0, 95, 50])
        self.declare_parameter('red_upper_1', [10, 255, 255])
        self.declare_parameter('red_lower_2', [165, 95, 50])
        self.declare_parameter('red_upper_2', [180, 255, 255])

        # Load parameters
        self.efficient_mode = bool(self.get_parameter('efficient_mode').value)
        camera_topic = self.get_parameter('camera_topic').value
        push_dir_topic = self.get_parameter('push_direction_topic').value
        debug_img_topic = self.get_parameter('debug_image_topic').value
        self.min_area = int(self.get_parameter('min_contour_area').value)
        self.deadzone_px = int(self.get_parameter('deadzone_px').value)
        
        # When efficient_mode is False, auto-enable debug image for real-time visualization
        if not self.efficient_mode:
            self.publish_debug = True
        else:
            self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.frame_skip_n = int(self.get_parameter('process_every_n_frames').value)
        self.target_w = int(self.get_parameter('processing_width').value)
        self.target_h = int(self.get_parameter('processing_height').value)

        self.frame_counter = 0

        # Initialize HSV arrays
        self.green_lower = np.array(self.get_parameter('green_lower').value, dtype=np.uint8)
        self.green_upper = np.array(self.get_parameter('green_upper').value, dtype=np.uint8)
        self.red_lower_1 = np.array(self.get_parameter('red_lower_1').value, dtype=np.uint8)
        self.red_upper_1 = np.array(self.get_parameter('red_upper_1').value, dtype=np.uint8)
        self.red_lower_2 = np.array(self.get_parameter('red_lower_2').value, dtype=np.uint8)
        self.red_upper_2 = np.array(self.get_parameter('red_upper_2').value, dtype=np.uint8)

        # ── QoS & CV Bridge ───────────────────────────────────────────
        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Publishers & Subscriptions ────────────────────────────────
        self.direction_pub = self.create_publisher(String, push_dir_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_img_topic, qos_profile=sensor_qos)

        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            qos_profile=sensor_qos
        )

        self.get_logger().info(
            f'[CameraPushDetector] Node started | EfficientMode: {self.efficient_mode} | '
            f'Res: {self.target_w if self.efficient_mode else "Full"}x{self.target_h if self.efficient_mode else "Full"} | '
            f'Skip: {"1/"+str(self.frame_skip_n) if self.efficient_mode else "None (Realtime)"} | '
            f'DebugImg: {self.publish_debug}'
        )

    def image_callback(self, msg: Image):
        # 1. Frame Skipping / Throttling Optimization (Only when efficient_mode is True)
        self.frame_counter += 1
        if self.efficient_mode and (self.frame_counter % max(1, self.frame_skip_n) != 0):
            return

        if not self.bridge:
            self.get_logger().warn_once('cv_bridge is not installed! Cannot process image.')
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion error: {e}')
            return

        # 2. Resolution Downscaling / Hard-Limiting (Only when efficient_mode is True)
        h, w, _ = cv_img.shape
        if self.efficient_mode and (w > self.target_w or h > self.target_h):
            cv_img = cv2.resize(cv_img, (self.target_w, self.target_h), interpolation=cv2.INTER_NEAREST)
            h, w, _ = cv_img.shape

        frame_center_x = w // 2

        # ── Color Masking & Bounding Box Extraction ───────────────────
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # Mask Green
        mask_green = cv2.inRange(hsv, self.green_lower, self.green_upper)
        # Mask Red (combining 2 HSV ranges)
        mask_red1 = cv2.inRange(hsv, self.red_lower_1, self.red_upper_1)
        mask_red2 = cv2.inRange(hsv, self.red_lower_2, self.red_upper_2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Morphological noise removal
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        detections = []

        # Find Green Contours
        contours_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours_g:
            area = cv2.contourArea(c)
            if area >= self.min_area:
                x, y, bw, bh = cv2.boundingRect(c)
                detections.append({
                    'color': 'GREEN',
                    'bbox': (x, y, bw, bh),
                    'area': area,
                    'cx': x + bw // 2,
                    'cy': y + bh // 2
                })

        # Find Red Contours
        contours_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours_r:
            area = cv2.contourArea(c)
            if area >= self.min_area:
                x, y, bw, bh = cv2.boundingRect(c)
                detections.append({
                    'color': 'RED',
                    'bbox': (x, y, bw, bh),
                    'area': area,
                    'cx': x + bw // 2,
                    'cy': y + bh // 2
                })

        # ── Guidance Decision Logic ──────────────────────────────────
        push_direction = "KEEP_CENTER"

        if detections:
            # Sort detections by area (largest/closest first)
            detections.sort(key=lambda d: d['area'], reverse=True)
            primary = detections[0]
            cx = primary['cx']
            color = primary['color']

            # Decision based on WRO Rules + Bounding Box Placement
            if color == 'RED':
                # Red Pillar -> Pass Right -> PUSH_RIGHT
                push_direction = "PUSH_RIGHT"
            elif color == 'GREEN':
                # Green Pillar -> Pass Left -> PUSH_LEFT
                push_direction = "PUSH_LEFT"
            else:
                # Generic obstacle avoidance based on X position offset
                if cx < (frame_center_x - self.deadzone_px):
                    push_direction = "PUSH_RIGHT"
                elif cx > (frame_center_x + self.deadzone_px):
                    push_direction = "PUSH_LEFT"
                else:
                    push_direction = "KEEP_CENTER"

        # Publish Decision
        dir_msg = String()
        dir_msg.data = push_direction
        self.direction_pub.publish(dir_msg)

        # ── Debug Image Visualization ────────────────────────────────
        if self.publish_debug:
            debug_img = cv_img.copy()

            # Draw center line and deadzone
            cv2.line(debug_img, (frame_center_x, 0), (frame_center_x, h), (255, 255, 255), 2)
            cv2.line(debug_img, (frame_center_x - self.deadzone_px, 0), (frame_center_x - self.deadzone_px, h), (200, 200, 200), 1, cv2.LINE_AA)
            cv2.line(debug_img, (frame_center_x + self.deadzone_px, 0), (frame_center_x + self.deadzone_px, h), (200, 200, 200), 1, cv2.LINE_AA)

            # Draw detections and bounding boxes
            for det in detections:
                x, y, bw, bh = det['bbox']
                color_bgr = (0, 255, 0) if det['color'] == 'GREEN' else (0, 0, 255)
                
                # Draw bounding box
                cv2.rectangle(debug_img, (x, y), (x + bw, y + bh), color_bgr, 3)
                # Draw centroid circle
                cv2.circle(debug_img, (det['cx'], det['cy']), 5, (255, 255, 0), -1)
                # Draw label
                label = f"{det['color']} {det['area']:.0f}px"
                cv2.putText(debug_img, label, (x, max(y - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

            # Overlay Direction Guidance Banner
            banner_color = (0, 255, 255)  # Yellow default
            if push_direction == "PUSH_LEFT":
                banner_color = (255, 100, 0)
                cv2.arrowedLine(debug_img, (frame_center_x + 60, 50), (frame_center_x - 60, 50), banner_color, 4, tipLength=0.3)
            elif push_direction == "PUSH_RIGHT":
                banner_color = (0, 100, 255)
                cv2.arrowedLine(debug_img, (frame_center_x - 60, 50), (frame_center_x + 60, 50), banner_color, 4, tipLength=0.3)

            cv2.putText(debug_img, f"GUIDANCE: {push_direction}", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, banner_color, 3)

            # Publish Debug Frame
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPushDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
