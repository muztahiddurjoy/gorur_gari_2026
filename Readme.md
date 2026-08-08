# NOTE!!! Draft README, messy shits. I will fix it later

# Running the disparsity algorithm node
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
ros2 run sensors_processing disparity_extender --ros-args --params-file config/disparity_extender_params.yaml

# Running RViz2 Visualization
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
rviz2 -d config/disparity_view.rviz

# Running the RPLidar Node
ros2 launch rplidar_ros rplidar_c1_launch.py

# Vision WebCam
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
ros2 run sensors_processing vision_node --ros-args --params-file config/vision_params.yaml -p camera_topic:=/camera/image_raw


# Start ros2 cam
ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera -p video_device:=/dev/video0

# TuningHSV
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws/sensors_processing/sensors_processing
python3 tune_hsv.py --webcam 2

# Turn off auto focus
v4l2-ctl -d /dev/video2 --set-ctrl=focus_absolute=0

