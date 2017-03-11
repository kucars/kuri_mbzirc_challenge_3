cmake_minimum_required(VERSION 2.8.3)
project(kuri_system_coordinator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  sensor_msgs
  image_transport
  cmake_modules
  tf message_filters
  nav_msgs
  message_generation
  actionlib_msgs
)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(kuri_msgs REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy actionlib_msgs sensor_msgs nav_msgs kuri_msgs
#  DEPENDS system_lib
)


include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
#TODO: Fix this geo2
add_library(geo2 src/geo.cpp)

add_executable(solo_player src/soloplayer.cpp)
target_link_libraries(solo_player ${catkin_LIBRARIES} geo2)
add_dependencies(solo_player kuri_msgs_gencfg kuri_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

