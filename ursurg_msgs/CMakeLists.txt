cmake_minimum_required(VERSION 3.1)
project(ursurg_msgs)

find_package(catkin REQUIRED COMPONENTS
  # actionlib
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    PoseGraspStamped.msg
)

add_service_files(
  DIRECTORY srv
  FILES SetRCM.srv
)

# add_action_files(
#   DIRECTORY action
#   FILES
#     x.action
# )

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
