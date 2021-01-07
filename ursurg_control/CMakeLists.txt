cmake_minimum_required(VERSION 3.1)
project(ursurg_control)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  ursurg_common
  ursurg_msgs
)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

add_executable(ursurg_control src/ursurg_control.cpp)
target_link_libraries(ursurg_control
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

add_executable(ursurg_control_rcm src/ursurg_control_rcm.cpp)
target_link_libraries(ursurg_control_rcm
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(ursurg_control_rcm
  PRIVATE
    ${orocos_kdl_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_compile_options(ursurg_control_rcm PRIVATE -Wall -Wextra)
