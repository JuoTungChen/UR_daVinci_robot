cmake_minimum_required(VERSION 3.8)
project(mops_description)

find_package(ament_cmake REQUIRED)
find_package(ur_description REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
