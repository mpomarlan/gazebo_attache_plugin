cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

message(STATUS ${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  /usr/local/include/gazebo-2.2
  /usr/include/sdformat-1.4)
  #${GAZEBO_INCLUDE_DIRS})

link_directories(
  /usr/local/lib/x86_64-linux-gnu/gazebo-2.2/plugins/)
  #${GAZEBO_LIBRARY_DIRS})

add_library(attache SHARED
  src/Attache.cpp)

target_link_libraries(attache
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES})