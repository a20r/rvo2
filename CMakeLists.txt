cmake_minimum_required(VERSION 2.8.3)
project(rvo2)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rvo2
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
 DEPENDS system_lib
)

link_directories(/usr/local/lib)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES} RVO)
