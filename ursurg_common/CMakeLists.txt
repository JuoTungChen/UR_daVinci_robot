cmake_minimum_required(VERSION 3.1)
project(ursurg_common)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  #CATKIN_DEPENDS
  DEPENDS Boost
  INCLUDE_DIRS include
  CFG_EXTRAS cmake_modules-extras.cmake
)

file(GLOB_RECURSE header_files "include/*[.h|.hpp]")
add_custom_target(header_target SOURCES ${header_files})

# Install headers
install(
  DIRECTORY include/ursurg_common
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install cmake files
install(
  DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)
