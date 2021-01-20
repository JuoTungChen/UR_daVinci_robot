cmake_minimum_required(VERSION 3.1)
project(ursurg_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2_ros
  touch_msgs
  ursurg_common
  ursurg_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

catkin_package()

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop Eigen3::Eigen ${catkin_LIBRARIES})
target_include_directories(teleop PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_options(teleop PRIVATE -Wall -Wextra)

add_executable(foot_control src/footcontrol.cpp)
target_link_libraries(foot_control Qt5::Core Qt5::Widgets ${catkin_LIBRARIES})
target_include_directories(foot_control PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_options(foot_control PRIVATE -Wall -Wextra)
