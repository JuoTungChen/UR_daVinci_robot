cmake_minimum_required(VERSION 3.8)
project(mops_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(range-v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mops_msgs REQUIRED)
find_package(mops_common REQUIRED)

add_library(mops_control SHARED src/mops_control.cpp)
target_compile_features(mops_control PUBLIC cxx_std_17)
ament_target_dependencies(mops_control
  rclcpp rclcpp_components urdf geometry_msgs sensor_msgs mops_msgs
  Eigen3 range-v3 orocos_kdl urdfdom_headers kdl_parser mops_common)
target_compile_options(mops_control PRIVATE -Wall -Wextra)
rclcpp_components_register_nodes(mops_control "mops_control::MopsControlNode")

install(
  TARGETS mops_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
