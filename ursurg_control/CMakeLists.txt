cmake_minimum_required(VERSION 3.1)
project(ursurg_control)

find_package(catkin REQUIRED COMPONENTS roscpp ur_control ur_control_msgs)
find_package(RobWork REQUIRED)

catkin_package()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ROBWORK_INCLUDE_DIRS}
)

add_executable(ursurg_control
  src/ursurg_control.cpp
)

target_link_libraries(ursurg_control
  ${ROBWORK_LIBRARIES}
  ${catkin_LIBRARIES}
)
