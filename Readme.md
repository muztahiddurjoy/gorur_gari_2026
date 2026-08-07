# NOTE!!! Draft README, messy shits. I will fix it later

# Running the disparsity algorithm node
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
ros2 run sensors_processing disparity_extender --ros-args --params-file config/disparity_extender_params.yaml

# Running RViz2 Visualization
cd ~/Documents/GitHub/gorur_gari_2026/ros2_ws
source install/setup.bash
rviz2 -d config/disparity_view.rviz
