cmake_minimum_required(VERSION 2.8.3)
project(ops_wbc_robot_controller)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0x)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    ops_wbc_digital_filter
    ops_wbc_robot
    ops_wbc_utils
    cmake_modules
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

generate_dynamic_reconfigure_options(cfg/Param.cfg)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ops_wbc_robot_controller
  CATKIN_DEPENDS
    ops_wbc_digital_filter
    ops_wbc_robot
    ops_wbc_utils
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
)

add_library(ops_wbc_robot_controller
  src/param.cpp
  src/effective_mass_matrix3d.cpp
  src/gravity_compensation.cpp
  src/friction_compensation.cpp
  src/damping.cpp
  src/joint_control.cpp
  src/position_control.cpp
  src/orientation_control.cpp
)
add_dependencies(ops_wbc_robot_controller ops_wbc_robot_controller_gencfg)
target_link_libraries(ops_wbc_robot_controller ${catkin_LIBRARIES})
