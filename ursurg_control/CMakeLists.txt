cmake_minimum_required(VERSION 3.1)
project(ursurg_control)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  ursurg_common
  ursurg_msgs
)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(range-v3 REQUIRED)

catkin_package()

add_executable(ursurg_control src/ursurg_control.cpp)
target_link_libraries(ursurg_control
  range-v3::range-v3
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(ursurg_control
  PRIVATE
    ${orocos_kdl_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_compile_options(ursurg_control PRIVATE -Wall -Wextra)
