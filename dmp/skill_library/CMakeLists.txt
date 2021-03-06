cmake_minimum_required(VERSION 2.4.6)
project(skill_library)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  task_msgs
  std_msgs
  dynamic_movement_primitive
  dynamic_movement_primitive_utilities
  robot_info
  message_generation
  cmake_modules
  genmsg
)

find_package(Boost REQUIRED COMPONENTS thread signals system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(${Boost_DEFINITIONS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

#rosbuild_genmsg()
#rosbuild_gensrv()

add_message_files(
   FILES
   Affordance.msg   
 )

add_service_files(
   FILES
   addAffordance.srv
   getAffordance.srv
 )

generate_messages(DEPENDENCIES task_msgs dynamic_movement_primitive std_msgs)

catkin_package(INCLUDE_DIRS include 
               LIBRARIES skill_library_client
               CATKIN_DEPENDS roscpp rosbag task_msgs std_msgs dynamic_movement_primitive dynamic_movement_primitive_utilities robot_info message_runtime cmake_modules 
               DEPENDS Eigen Eigen3 Boost)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#common commands for building c++ executables and libraries

add_library(skill_library_client
  src/skill_library_client.cpp
)
add_dependencies(skill_library_client skill_library_generate_message_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

add_executable(skill_library
  src/skill_library_node.cpp
  src/skill_library.cpp
  src/dmp_library_client.cpp
)
target_link_libraries(skill_library ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(skill_library skill_library_generate_message_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})  


#target_link_libraries(example ${PROJECT_NAME})
