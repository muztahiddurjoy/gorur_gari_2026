# Gorur Gari 2026 - ROS 2 Workspace Quick Start

---

## 🚗 Core Autonomous Driving & LiDAR

### 1. Run Disparity Extender Navigation Node
```bash
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
ros2 run sensors_processing disparity_extender --ros-args --params-file config/disparity_extender_params.yaml
```

### 2. RViz2 Visualization
```bash
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
rviz2 -d config/disparity_view.rviz
```

### 3. RPLidar C1 / A1 Launch
```bash
ros2 launch rplidar_ros rplidar_c1_launch.py
```

---

## 📷 Vision Node & Debugging Guide

### 1. Camera Hardware Setup (Disable Auto-Focus)
```bash
# Set camera parameters (adjust /dev/video2 to your camera port)
v4l2-ctl -d /dev/video2 --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d /dev/video2 --set-ctrl=focus_absolute=0
```

### 2. Launch USB Camera Node
```bash
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera -p video_device:=/dev/video2
```

### 3. Run Vision Processing Node
```bash
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash

# Run in default mode (publishes pillar detection to /closest_obj)
ros2 run autonomy vision_node --ros-args --params-file config/vision_params.yaml

# Run in Full Debug Realtime Mode (forces debug image publishing)
ros2 run autonomy vision_node --ros-args --params-file config/vision_params.yaml -p efficient_mode:=false
```

---

## 🔍 Vision Debugging & Tuning Tools

### 1. Monitor Detected Pillar Color
```bash
# Echo the closest object output ('R' for Red, 'G' for Green, 'N' for None)
ros2 topic echo /closest_obj
```

### 2. View Debug Image Stream (Processed Mask & Bounding Boxes)
```bash
# Open RQT Image Viewer to inspect /vision/debug_image
ros2 run rqt_image_view rqt_image_view /vision/debug_image
```

### 3. Interactive HSV Color Threshold Tuner
```bash
# Option A: Run via ROS 2 inside ros2_ws
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
ros2 run autonomy tune_hsv

# Option B: Direct Python execution from ros2_ws
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
python3 autonomy/autonomy/tune_hsv.py --webcam 2

# Option C: Direct Python execution from repository root
cd ~/Documents/GitHub/gorur_gari_2026
python3 draft_vision_node_pillar_color_detection/src/wro_autodrive/wro_autodrive/tune_hsv.py --webcam 2
```


#### RUN Rviz for open round
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d config/open_round_view.rviz


#### RUN open round for debug 
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run autonomy open_round_run --ros-args \
  -p require_button_start:=false \
  -p enable_auto_steering:=false

### Custom disparity extender
##### Set enable_drive to false for safe bench testing
ros2 run autonomy custom_disparity_extender --ros-args \
    --params-file config/bot_config.yaml \
    -p enable_drive:=false

