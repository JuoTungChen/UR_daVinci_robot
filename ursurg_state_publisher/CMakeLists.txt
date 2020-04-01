cmake_minimum_required(VERSION 2.8.3)
project(ur_surgical_platform)

set(CMAKE_BUILD_TYPE "Debug")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
#  ur_robot_driver
#  actionlib
#  actionlib_msgs
)
SET(RW_ROOT "$ENV{RW_ROOT}")
FIND_PACKAGE(RobWork REQUIRED)
find_package(ur_rtde REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS actionlib_msgs
#  DEPENDS system_lib
)


include_directories(
# include
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
  ${catkin_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)

target_link_libraries(inv_kin_node PRIVATE ur_rtde::rtde)


