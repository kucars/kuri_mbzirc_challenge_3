cmake_minimum_required(VERSION 2.8.3)
project(kuri_object_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  DIRECTORY srv
  FILES
  Object2Track.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime message_generation
)
