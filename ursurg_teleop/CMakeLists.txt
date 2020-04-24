cmake_minimum_required(VERSION 3.1)
project(ursurg_teleop)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Use C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

find_package(Geomagic REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(ur_rtde REQUIRED)

catkin_package()

include_directories(
  ${Geomagic_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  add_compile_options(-Wall -Wextra)
endif()

add_executable(haptics_differential src/node.cpp src/hd++.cpp)
target_link_libraries(haptics_differential
  Eigen3::Eigen
  ${Geomagic_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(footpedal src/footpedal.cpp)
target_link_libraries(footpedal ${catkin_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets)

add_executable(ur_teleop src/ur_teleop.cpp)
target_link_libraries(ur_teleop
  ur_rtde::rtde
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)
