cmake_minimum_required(VERSION 2.8.3)
project(kuri_aerial_manipulation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    rospy
    sensor_msgs
    image_transport
    cmake_modules
    tf message_filters
    nav_msgs
    kuri_msgs
    message_generation
    actionlib_msgs
)

add_service_files(
  FILES
  Attach.srv
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs
)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(kuri_msgs REQUIRED)

catkin_package(
    CATKIN_DEPENDS
)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(kuri_msgs REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
			    ${OpenCV_INCLUDE_DIRS}
			    ${Eigen_INCLUDE_DIRS}
 			    ${PCL_INCLUDE_DIRS})

add_executable(object_detection src/ActionServerInterface.cpp src/AerialManipulationControl.cpp include/AerialManipulationControl.h)
target_link_libraries(object_detection ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(object_detection ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp kuri_msgs_generate_messages_cpp)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
#  roslaunch_add_file_check(launch/task.launch)
endif()

