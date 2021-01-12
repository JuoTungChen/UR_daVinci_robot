cmake_minimum_required(VERSION 3.1)
project(ursurg_state_publisher)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
  COMPONENTS
    kdl_parser
    roscpp
    rosconsole
    rostime
    sensor_msgs
    tf2_ros
    tf2_kdl
    ursurg_common
)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package()

add_executable(ursurg_state_publisher src/ursurg_state_publisher_node.cpp)
target_link_libraries(ursurg_state_publisher ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(ursurg_state_publisher PRIVATE ${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_compile_options(ursurg_state_publisher PRIVATE -Wall -Wextra)
