cmake_minimum_required(VERSION 3.1)
project(ursurg_control)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  ursurg_common
)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

include_directories(
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ursurg_control
  src/ursurg_control.cpp
)

if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  target_compile_options(ursurg_control PRIVATE -Wall)
endif()

target_link_libraries(ursurg_control
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
  ${catkin_LIBRARIES}
)
