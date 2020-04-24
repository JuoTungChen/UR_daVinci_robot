cmake_minimum_required(VERSION 3.1)
project(ursurg_teleop)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  ur_control
  ursurg_common
)

find_package(Geomagic REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

catkin_package()

include_directories(
  ${Geomagic_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  add_compile_options(-Wall -Wextra)
endif()

add_executable(ursurg_teleop src/ursurg_teleop.cpp src/gui.cpp src/robot.cpp src/haptics.cpp src/hd++.cpp)
target_link_libraries(ursurg_teleop
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Eigen3::Eigen
  ${Geomagic_LIBRARIES}
  ${catkin_LIBRARIES}
)
