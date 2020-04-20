cmake_minimum_required(VERSION 2.8.3)
project(ursurg_msgs)

find_package(catkin REQUIRED COMPONENTS
  # actionlib
  message_generation
  # std_msgs
)

# add_message_files(
#   DIRECTORY msg
#   FILES
#     x.msg
# )

add_service_files(
  DIRECTORY srv
  FILES SetFloat64.srv
)

# add_action_files(
#   DIRECTORY action
#   FILES
#     x.action
# )

generate_messages(
  # DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
