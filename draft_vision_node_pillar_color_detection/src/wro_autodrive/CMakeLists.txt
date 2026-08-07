cmake_minimum_required(VERSION 3.8)
project(wro_autodrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find build dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate custom ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  DEPENDENCIES std_msgs
)

# Install Python Node executables and modules into lib/wro_autodrive
install(PROGRAMS
  wro_autodrive/vision_node.py
  wro_autodrive/fusion_node.py
  wro_autodrive/state_machine_node.py
  wro_autodrive/drive_controller_node.py
  wro_autodrive/tune_hsv.py
  wro_autodrive/pillar_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
