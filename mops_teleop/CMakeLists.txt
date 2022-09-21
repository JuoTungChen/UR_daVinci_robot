cmake_minimum_required(VERSION 3.8)
project(mops_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mops_common REQUIRED)
find_package(mops_msgs REQUIRED)
find_package(touch_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop Eigen3 tf2_ros geometry_msgs mops_msgs std_msgs touch_msgs mops_common)
target_compile_features(teleop PRIVATE cxx_std_17)
target_compile_options(teleop PRIVATE -Wall -Wextra)

set(CMAKE_AUTOMOC ON)
add_executable(foot_control src/footcontrol.cpp)
ament_target_dependencies(foot_control rclcpp std_msgs)
target_link_libraries(foot_control Qt5::Core Qt5::Widgets)
target_compile_features(foot_control PRIVATE cxx_std_17)
target_compile_options(foot_control PRIVATE -Wall -Wextra)

install(
  TARGETS foot_control teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
