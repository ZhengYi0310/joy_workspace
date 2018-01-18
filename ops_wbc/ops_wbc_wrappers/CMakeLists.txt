cmake_minimum_required(VERSION 2.8.3)
project(ops_wbc_wrappers)

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
message(STATUS "Compliter FLAG ${CMAKE_CXX_FLAGS}.")

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    image_transport
    message_filters
    roscpp
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ops_wbc_wrappers
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    message_filters
    roscpp
    std_srvs
)

add_library(ops_wbc_wrappers
  src/image_subscriber.cpp
  src/utils.cpp
  src/rect.cpp
  src/hsv_mask.cpp
)
target_link_libraries(ops_wbc_wrappers ${catkin_LIBRARIES})

