cmake_minimum_required(VERSION 3.19)
project(mops_common)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_library(mops_common INTERFACE
  include/linalg.hpp
  include/mathutility.hpp
  include/prettyprint.hpp
  include/conversions/conversions.hpp
  include/conversions/eigen.hpp
  include/conversions/kdl.hpp
)
ament_target_dependencies(mops_common INTERFACE geometry_msgs orocos_kdl Eigen3 Boost)
target_include_directories(mops_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_mops_common HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs orocos_kdl Eigen3 Boost)

install(DIRECTORY include/ DESTINATION include/mops_common)
install(
  TARGETS mops_common
  EXPORT export_mops_common
)

ament_package()
