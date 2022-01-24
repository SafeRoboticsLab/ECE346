cmake_minimum_required(VERSION 3.0.2)
project(rc_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
find_package(Boost REQUIRED)


add_message_files(
  DIRECTORY msg
  FILES RCControl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost
)
