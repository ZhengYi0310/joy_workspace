cmake_minimum_required(VERSION 2.8.3)
project(markov_segm_backup)

find_package(catkin REQUIRED)



#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(Armadillo REQUIRED)

add_service_files(
   FILES
   Run.srv
   Init.srv
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
    
  DEPENDS opencv boost armadillo
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${ARMADILLO_LIBRARIES}
${Boost_SYSTEM_LIBRARIES} ${Boost_FILESYSTEM_LIBRARIES})

link_directories(${catkin_LIBRARY_DIRS})


add_executable(mrf_segm_backup src/cost.cpp src/simulated_annealing.cpp src/config.cpp src/markov_segm_node.cpp src/convert.cpp)
target_link_libraries(mrf_segm_backup ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBRARIES} ${ARMADILLO_LIBRARIES}
${Boost_SYSTEM_LIBRARIES} ${Boost_FILESYSTEM_LIBRARIES})





