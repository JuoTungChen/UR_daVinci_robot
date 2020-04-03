cmake_minimum_required(VERSION 3.1)
project(ursurg_state_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp ur_control)
find_package(RobWork REQUIRED)
find_package(ur_rtde 1.0 REQUIRED)

catkin_package()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ROBWORK_INCLUDE_DIRS}
)

add_executable(inv_kin_node
  src/inv_kin_node.cpp
  src/workcellcontainer.cpp
  src/devicekinematics.cpp
)

target_link_libraries(
  inv_kin_node
  ur_rtde::rtde
  ${ROBWORK_LIBRARIES}
  ${catkin_LIBRARIES}
)
