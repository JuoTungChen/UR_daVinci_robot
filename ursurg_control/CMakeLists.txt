cmake_minimum_required(VERSION 3.1)
project(ursurg_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ursurg_common
  ursurg_robwork_model
)
find_package(RobWork REQUIRED PATHS "$ENV{RW_ROOT}")

catkin_package()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

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
