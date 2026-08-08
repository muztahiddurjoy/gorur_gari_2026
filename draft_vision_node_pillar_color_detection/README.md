# OpenCV Pillar Detection & Draft Vision Pipeline (`wro-2026`)

> **Draft / Reference Folder**: This directory serves as a standalone draft and reference prototype for OpenCV traffic pillar detection (Red/Green 50x50x100mm blocks), HSV threshold tuning tools, and initial vision pipeline prototypes for the WRO 2026 Future Engineers competition.

---

## 📂 Key Components & File Guide

### 1. Core Vision Modules (`src/wro_autodrive/wro_autodrive/`)
- **`pillar_detector.py`**: Standalone OpenCV module for HSV segmentation, Gaussian blur filtering, morphological ops, aspect ratio, and solidity contour filtering for Red and Green WRO pillars.
- **`vision_node.py`**: ROS 2 node wrapping `pillar_detector.py` to process camera frames (`/camera/image_raw`) and publish obstacle array messages (`/vision/obstacles`).
- **`tune_hsv.py`**: Interactive GUI and click-to-inspect tool for real-time HSV color threshold tuning under local lighting conditions.
- **`fusion_node.py`**: Prototype sensor fusion node combining camera vision detections with LiDAR scan ranges.
- **`drive_controller_node.py`**: Prototype drive controller.
- **`state_machine_node.py`**: Draft high-level state machine for managing challenge states.

### 2. Custom ROS 2 Message Definitions (`src/wro_autodrive/msg/`)
- `Obstacle.msg`: Represents individual detected pillars (`string color`, `int32 center_x`, `int32 width`).
- `ObstacleArray.msg`: Headered array of `Obstacle` messages.

### 3. Test & Launch Assets
- `test_images/sample_wro_track.jpg`: Sample camera frame for testing HSV segmentation offline.
- `launch/wro_launch.py`: Prototype launch file.

---

## 🚀 How to Reuse in Main Stack (`ros2_ws`)

When ready to integrate full vision processing into `gorur_gari_2026`:
1. Port `pillar_detector.py` into `ros2_ws/sensors_processing/sensors_processing/`.
2. Add camera node subscription and publish nearest pillar color (`"R"` or `"G"`) to `/closest_obj` for `disparity_extender.py`.
