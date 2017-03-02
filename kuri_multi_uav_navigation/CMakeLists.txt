cmake_minimum_required(VERSION 2.8.3)
project(kuri_multi_agent_navigation)
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
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    find_package(PkgConfig REQUIRED)
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

#add_executable(navigator src/navigator.cpp)
#target_link_libraries(navigator SSPathPlanner
#    -lgmp -lmpfr
#    ${CGAL_LIBRARY}
#    ${CGAL_Core_LIBRARY}
#    Qt4::QtGui
#    Qt4::QtCore
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${rviz_visual_tools_LIBRARIES}
#    )

add_library(geo SHARED src/geo.cpp)


add_executable(pathplanner_action_server src/pathplanner_action_server.cpp src/pathgenerator.cpp include/pathgenerator.h)
target_link_libraries(pathplanner_action_server SSPathPlanner
    -lgmp -lmpfr
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES}
    geo
    )

add_executable(navigation_action_server src/navigation_action_server.cpp src/navigator.cpp include/navigator.h)
target_link_libraries(navigation_action_server SSPathPlanner
    -lgmp -lmpfr
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES}
    geo
    )

add_executable(uavs_initiator src/uavs_initiator.cpp)
target_link_libraries(uavs_initiator ${catkin_LIBRARIES})


add_executable(gps_waypoints_local src/gps_waypoints_local.cpp)
target_link_libraries(gps_waypoints_local geo ${catkin_LIBRARIES})

add_executable(gps_waypoints_srv src/gps_waypoints_srv.cpp)
target_link_libraries(gps_waypoints_srv geo ${catkin_LIBRARIES})

#add_executable(path_follower src/pathfollower.cpp)
#target_link_libraries(path_follower SSPathPlanner
#    -lgmp -lmpfr
#    ${CGAL_LIBRARY}
#    ${CGAL_Core_LIBRARY}
#    Qt4::QtGui
#    Qt4::QtCore
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${rviz_visual_tools_LIBRARIES}
#    )


