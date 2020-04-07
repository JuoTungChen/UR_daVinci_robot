cmake_minimum_required(VERSION 2.8.3)
project(eua_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  std_msgs
  sensor_msgs
  control_msgs
  rospy
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES SetTrajectoryLimits.srv SetCalibrationOffset.srv
)

catkin_python_setup()
generate_messages()
catkin_package()
