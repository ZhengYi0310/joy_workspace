cmake_minimum_required(VERSION 2.8.3)
project(ops_wbc_robot)

include(CheckCXXCompilerFlag)
message(STATUS "Use compiler ${CMAKE_CXX_COMPILER}.")
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0x)
if(COMPILER_SUPPORTS_CXX11)
#  message(WARNING "Use compile flag -std=c++11")
  set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
elseif(COMPILER_SUPPORTS_CXX0X)
#  message(WARNING "Use compile flag -std=c++0x")
  set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
#set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED
  COMPONENTS
    ops_wbc_digital_filter
    ops_wbc_utils
    cmake_modules
    roscpp
    tf2
    tf2_ros
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ops_wbc_robot
  CATKIN_DEPENDS
    ops_wbc_digital_filter
    ops_wbc_utils
    cmake_modules
    roscpp
    tf2
    tf2_ros
  DEPENDS yaml-cpp
)

add_library(ops_wbc_robot
  src/robot/transformation.cpp
  src/robot/manipulator.cpp
  src/robot/robot.cpp
  src/robot/tf_publisher.cpp
  src/robot/parser.cpp
  src/robot/math.cpp	
)
target_link_libraries(ops_wbc_robot
  ${catkin_LIBRARIES}
  yaml-cpp
  ${EIGEN3_LIBRARIES}
)

add_executable(parser_test test/test.cpp)
add_dependencies(parser_test ops_wbc_robot_gencpp)
target_link_libraries(parser_test ops_wbc_robot)

add_executable(parser_test2 test/test2.cpp)
add_dependencies(parser_test2 ops_wbc_robot_gencpp)
target_link_libraries(parser_test2 ops_wbc_robot)

add_executable(parser_test3 test/test3.cpp)
add_dependencies(parser_test3 ops_wbc_robot_gencpp)
target_link_libraries(parser_test3 ops_wbc_robot)

add_executable(parser_test4 test/test4.cpp)
add_dependencies(parser_test4 ops_wbc_robot_gencpp)
target_link_libraries(parser_test4 ops_wbc_robot)

