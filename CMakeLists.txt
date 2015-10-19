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
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

link_directories(/usr/local/lib)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES} RVO)
add_executable(rvo2_node src/rvo2_node.cpp src/PoseSubscriber.cpp
    src/PrefVelSubscriber.cpp src/VelocitySubscriber.cpp)
target_link_libraries(rvo2_node ${catkin_LIBRARIES} RVO)
