cmake_minimum_required(VERSION 3.8)
project(mops_state_publisher_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mops_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mops_state_publisher src/mops_state_publisher.cpp)
target_compile_features(mops_state_publisher PRIVATE cxx_std_17)
target_compile_options(mops_state_publisher PRIVATE -Wall -Wextra)
ament_target_dependencies(mops_state_publisher
  rclcpp urdf kdl_parser orocos_kdl tf2_ros geometry_msgs sensor_msgs mops_msgs std_msgs)
add_executable(mops_remapper src/mops_remapper.cpp)
target_compile_features(mops_remapper PRIVATE cxx_std_17)
target_compile_options(mops_remapper PRIVATE -Wall -Wextra)
ament_target_dependencies(mops_remapper
  rclcpp sensor_msgs)

install(
  TARGETS mops_state_publisher mops_remapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
