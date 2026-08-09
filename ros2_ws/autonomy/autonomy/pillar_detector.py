"""
pillar_detector.py - High-Performance Traffic Pillar Detector for WRO 2026 Future Engineers

Standalone OpenCV module for detecting Red and Green traffic pillars (50x50x100mm)
from a BGR camera frame. Designed for low-latency execution on embedded SBCs (RPi 4/5).

Usage:
    from wro_autodrive.pillar_detector import PillarDetector

    detector = PillarDetector()
    results = detector.detect(bgr_frame)
    # results: [{'color': 'red', 'bbox': (x, y, w, h), 'centroid_x': int}, ...]
"""

import cv2
import numpy as np


class PillarDetector:
    """
    Detects Red and Green WRO traffic pillars via HSV color segmentation.

    All HSV thresholds are exposed as class attributes for easy runtime
    tuning or ROS 2 parameter injection without modifying source code.
    """

    # ------------------------------------------------------------------ #
    # HSV Threshold Bounds (OpenCV scale: H 0-180, S 0-255, V 0-255)
    # ------------------------------------------------------------------ #
    # Derived from target RGB values with generous tolerance bands
    # to handle variable arena lighting conditions.
    #
    # Green Pillar — Target RGB(68, 214, 44) → HSV center ≈ (56, 202, 214)
    GREEN_LOWER = np.array([35, 80, 50], dtype=np.uint8)
    GREEN_UPPER = np.array([85, 255, 255], dtype=np.uint8)

    # Red Pillar — Target RGB(238, 39, 55) → HSV center ≈ (178, 213, 238)
    # Saturation threshold (95) catches indoor plastic/printed blocks while rejecting skin (S < 80)
    RED_LOWER_1 = np.array([0, 95, 50], dtype=np.uint8)
    RED_UPPER_1 = np.array([12, 255, 255], dtype=np.uint8)
    RED_LOWER_2 = np.array([165, 95, 50], dtype=np.uint8)
    RED_UPPER_2 = np.array([180, 255, 255], dtype=np.uint8)

    # ------------------------------------------------------------------ #
    # Contour Filtering Parameters
    # ------------------------------------------------------------------ #
    MIN_CONTOUR_AREA = 250

    # Aspect Ratio (Height / Width): 0.4 - 4.5 handles any orientation of blocks
    MIN_ASPECT_RATIO = 0.4
    MAX_ASPECT_RATIO = 4.5

    # Solidity (Contour Area / Bounding Box Area): 0.4 allows hollowed/LEGO-style blocks
    MIN_SOLIDITY = 0.40

    # Gaussian blur kernel size (must be odd)
    BLUR_KERNEL = (5, 5)

    # Fraction of frame height to crop from top (0.0 – 1.0)
    ROI_TOP_CROP = 0.4

    # Morphological kernel for mask cleanup (erosion + dilation)
    MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def __init__(
        self,
        min_area: int = None,
        roi_top_crop: float = None,
        blur_kernel: tuple = None,
    ) -> None:
        """
        Optionally override class-level defaults at instantiation.

        :param min_area:      Override MIN_CONTOUR_AREA threshold.
        :param roi_top_crop:  Override ROI_TOP_CROP fraction (0.0–1.0).
        :param blur_kernel:   Override BLUR_KERNEL size tuple, e.g. (3, 3).
        """
        if min_area is not None:
            self.MIN_CONTOUR_AREA = min_area
        if roi_top_crop is not None:
            self.ROI_TOP_CROP = roi_top_crop
        if blur_kernel is not None:
            self.BLUR_KERNEL = blur_kernel

    # ================================================================== #
    #                        PUBLIC API                                   #
    # ================================================================== #

    def detect(self, bgr_frame: np.ndarray) -> list:
        """
        Main entry point. Processes a single BGR camera frame and returns
        a list of detected pillar obstacles.

        :param bgr_frame: Raw BGR image (numpy array, shape HxWx3).
        :returns: List of dicts, each containing:
                  - 'color':      str ('red' or 'green')
                  - 'bbox':       tuple (x, y, w, h) in full-frame coords
                  - 'centroid_x': int, x-center of bounding box
        """
        # Step 1: Preprocessing — ROI crop + Gaussian blur
        roi, y_offset = self._preprocess(bgr_frame)

        # Step 2: Convert ROI to HSV color space (single conversion, reused)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Step 3: Generate color masks
        green_mask = self._create_green_mask(hsv)
        red_mask = self._create_red_mask(hsv)

        # Step 4: Extract and filter contours from each mask
        detections = []
        detections.extend(self._extract_contours(green_mask, 'green', y_offset))
        detections.extend(self._extract_contours(red_mask, 'red', y_offset))

        return detections

    def draw_detections(self, bgr_frame: np.ndarray, detections: list) -> np.ndarray:
        """
        Visual debugging helper: Draws bounding boxes, ROI line, and centroids onto a frame copy.

        :param bgr_frame: Original BGR image.
        :param detections: Output list from detect().
        :returns: Annotated BGR frame copy.
        """
        annotated = bgr_frame.copy()
        h, w = annotated.shape[:2]

        # Draw ROI cutoff line (yellow dashed)
        roi_y = int(h * self.ROI_TOP_CROP)
        cv2.line(annotated, (0, roi_y), (w, roi_y), (0, 255, 255), 2)
        cv2.putText(annotated, "ROI CROP LINE", (10, roi_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Draw each detection
        for det in detections:
            color_name = det['color']
            x, y, bw, bh = det['bbox']
            cx = det['centroid_x']

            # Choose box color based on detection label
            box_color = (0, 255, 0) if color_name == 'green' else (0, 0, 255)

            # Draw bounding box
            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), box_color, 2)

            # Draw centroid dot and vertical guide line
            cv2.circle(annotated, (cx, y + bh // 2), 4, (255, 255, 255), -1)
            cv2.line(annotated, (cx, y), (cx, y + bh), (255, 255, 255), 1)

            # Draw text label
            label = f"{color_name.upper()} x:{cx} w:{bw}"
            cv2.putText(annotated, label, (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

        return annotated

    # ================================================================== #
    #                      INTERNAL PIPELINE STAGES                       #
    # ================================================================== #

    def _preprocess(self, frame: np.ndarray) -> tuple:
        """
        Applies ROI cropping and Gaussian blur to reduce processing area
        and suppress high-frequency sensor noise.

        :param frame: Full BGR frame.
        :returns: (roi_frame, y_offset) where y_offset is the pixel row
                  where the ROI starts (for coordinate remapping).
        """
        h = frame.shape[0]

        # Crop: keep only the bottom portion of the frame
        y_start = int(h * self.ROI_TOP_CROP)
        roi = frame[y_start:, :]

        # Gaussian blur — suppress CCD/CMOS noise; odd kernel enforced
        roi = cv2.GaussianBlur(roi, self.BLUR_KERNEL, 0)

        return roi, y_start

    def _create_green_mask(self, hsv: np.ndarray) -> np.ndarray:
        """
        Generates a binary mask isolating green hue regions.
        Applies morphological open (erode→dilate) to remove speckle noise.

        :param hsv: HSV image (ROI).
        :returns: Cleaned binary mask (uint8, 0 or 255).
        """
        mask = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_UPPER)

        # Morphological open: erode removes salt noise, dilate restores edges
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.MORPH_KERNEL, iterations=1)

        return mask

    def _create_red_mask(self, hsv: np.ndarray) -> np.ndarray:
        """
        Generates a binary mask isolating red hue regions.
        Red wraps around H=0/180 in OpenCV HSV, so we OR two sub-range masks.

        :param hsv: HSV image (ROI).
        :returns: Cleaned binary mask (uint8, 0 or 255).
        """
        # Low-end red hues (0–10)
        mask_low = cv2.inRange(hsv, self.RED_LOWER_1, self.RED_UPPER_1)

        # High-end red hues (170–180)
        mask_high = cv2.inRange(hsv, self.RED_LOWER_2, self.RED_UPPER_2)

        # Combine both ranges with bitwise OR
        mask = cv2.bitwise_or(mask_low, mask_high)

        # Morphological open to clean up noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.MORPH_KERNEL, iterations=1)

        return mask

    def _extract_contours(
        self,
        mask: np.ndarray,
        color_label: str,
        y_offset: int,
    ) -> list:
        """
        Finds contours in a binary mask, filters by minimum area, and
        computes bounding boxes + centroids.

        Coordinates are remapped to full-frame space by adding y_offset
        to the bounding box Y coordinate.

        :param mask:        Binary mask (uint8).
        :param color_label: 'red' or 'green' tag for output dicts.
        :param y_offset:    Pixel offset from ROI crop to restore full-frame Y.
        :returns: List of detection dicts.
        """
        # RETR_EXTERNAL: only outermost contours (ignore internal holes)
        # CHAIN_APPROX_SIMPLE: compress contour points to save memory
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        results = []

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # 1. Reject noise blobs below minimum area threshold
            if area < self.MIN_CONTOUR_AREA:
                continue

            # Compute axis-aligned bounding box
            x, y, w, h = cv2.boundingRect(cnt)

            # 2. Aspect Ratio check (Height / Width)
            aspect_ratio = float(h) / float(w) if w > 0 else 0.0
            if aspect_ratio < self.MIN_ASPECT_RATIO or aspect_ratio > self.MAX_ASPECT_RATIO:
                continue

            # 3. Solidity check (Contour Area / Bounding Box Area)
            box_area = float(w * h)
            solidity = float(area) / box_area if box_area > 0 else 0.0
            if solidity < self.MIN_SOLIDITY:
                continue

            # Remap Y coordinate back to full-frame space
            y_full = y + y_offset

            # Compute X centroid of the bounding box
            centroid_x = x + (w // 2)

            results.append({
                'color': color_label,
                'bbox': (x, y_full, w, h),
                'centroid_x': centroid_x,
                'aspect_ratio': aspect_ratio,
                'solidity': solidity,
            })

        return results
