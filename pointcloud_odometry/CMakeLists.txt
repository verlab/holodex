cmake_minimum_required(VERSION 3.16)
project(pointcloud_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# Configurar small_gicp embarcado (third_party)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/small_gicp/CMakeLists.txt")
  set(BUILD_HELPER ON CACHE BOOL "" FORCE)
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
  add_subdirectory(third_party/small_gicp)
  if(TARGET small_gicp AND NOT TARGET small_gicp::small_gicp)
    add_library(small_gicp::small_gicp ALIAS small_gicp)
  endif()
else()
  find_package(small_gicp REQUIRED)
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 1. Componente de Filtragem e Decimação 2D
add_library(pointcloud_filter_component SHARED
  src/pointcloud_filter_component.cpp
)
target_include_directories(pointcloud_filter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pointcloud_filter_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)
target_link_libraries(pointcloud_filter_component
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_nodes(pointcloud_filter_component "pointcloud_odometry::PointcloudFilterComponent")

# 2. Componente de Odometria small_gicp Frame-to-Submap
add_library(icp_odometry_component SHARED
  src/icp_odometry_component.cpp
)
target_include_directories(icp_odometry_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(icp_odometry_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)
target_link_libraries(icp_odometry_component
  small_gicp
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)
rclcpp_components_register_nodes(icp_odometry_component "pointcloud_odometry::IcpOdometryComponent")

# Executáveis standalone opcionais
add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
target_link_libraries(pointcloud_filter_node pointcloud_filter_component)
ament_target_dependencies(pointcloud_filter_node rclcpp)

add_executable(icp_odometry_node src/icp_odometry_node.cpp)
target_link_libraries(icp_odometry_node icp_odometry_component)
ament_target_dependencies(icp_odometry_node rclcpp)

# Teste standalone da logica de recuperacao (dead-reckoning + porta adaptativa)
add_executable(test_recovery_utils test/test_recovery_utils.cpp)
target_link_libraries(test_recovery_utils Eigen3::Eigen)

# Instalação
install(TARGETS
  pointcloud_filter_component
  icp_odometry_component
  pointcloud_filter_node
  icp_odometry_node
  test_recovery_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pipeline_orchestrator.py
  scripts/save_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
