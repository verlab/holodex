cmake_minimum_required(VERSION 3.16)
project(yolo_seg_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  /usr/include
)

add_executable(yolo_seg_node
  src/yolo_seg_node.cpp
)

target_link_libraries(yolo_seg_node
  ${OpenCV_LIBS}
  /lib/libVIPhal.so
  /lib/libNBGlinker.so
  pthread
  dl
)

ament_target_dependencies(yolo_seg_node
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS
  yolo_seg_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

ament_package()
