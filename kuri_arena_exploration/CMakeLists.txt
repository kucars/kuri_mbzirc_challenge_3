cmake_minimum_required(VERSION 2.8.3)
project(kuri_arena_exploration)
SET ( CMAKE_CXX_FLAGS "-frounding-math" )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  shape_msgs
  std_msgs
  rviz_visual_tools
  kuri_msgs
  sspp
)

set( ROS_DISTRO $ENV{ROS_DISTRO} )
if( ${ROS_DISTRO} STREQUAL "indigo" )
    find_package(fcl REQUIRED)
elseif(${ROS_DISTRO} STREQUAL "kinetic")
    find_package(PkgConfig REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    pkg_check_modules(LIBFCL REQUIRED fcl)
else()
    message("Unsupported ROS Distro")
endif()

# find_package(Boost REQUIRED COMPONENTS system)
find_package(CGAL QUIET COMPONENTS  )
#find_package(fcl REQUIRED)
find_package(PCL REQUIRED)
find_package(sspp REQUIRED)
find_package(CUDA QUIET)
find_package(component_test QUIET)

if ( NOT component_test_FOUND )
  message(STATUS "exploration waypoints action will not work")
#  return()
endif()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp sspp
)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include(FindPkgConfig)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (component_test_FOUND)
add_executable(exploration_waypoints_action_server src/exploration_waypoints_action_server.cpp src/exploration_waypoints.cpp include/exploration_waypoints.h)
target_link_libraries(exploration_waypoints_action_server SSPathPlanner
    -lgmp -lmpfr
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    CoverageHeuristic
    occlusion_culling_gpu
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES}
    )
endif (component_test_FOUND)