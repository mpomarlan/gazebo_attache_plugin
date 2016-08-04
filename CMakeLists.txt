cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
project(gazebo_attache_plugin)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  attache_msgs
  roslib
  roscpp
  gazebo_ros 
  attache_msgs)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(PkgConfig)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

message(STATUS ${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  #/usr/local/include/gazebo-2.2
  #/usr/include/sdformat-1.4
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 
    attache_msgs
    roslib
    roscpp 
    gazebo_ros 
)


add_library(attache SHARED
  src/Attache.cpp)

target_link_libraries(attache
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} ${SDF_LIBRARIES})

add_dependencies(attache ${catkin_EXPORTED_TARGETS})

