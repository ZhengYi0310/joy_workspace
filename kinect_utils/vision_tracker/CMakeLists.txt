cmake_minimum_required(VERSION 2.8.3)
project(vision_tracker)

find_package(catkin REQUIRED)


#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(OpenCV REQUIRED)

add_service_files(
   FILES
   WriteToDisk.srv
 )

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_runtime
  DEPENDS opencv
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})


add_executable(color_tracker src/color_tracker_node.cpp src/Object.cpp)
target_link_libraries(color_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#install(TARGETS color_tracker DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})




