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

# find_package(Boost REQUIRED COMPONENTS system)
# link_directories(${Boost_LIBRARY_DIRS})
# include_directories(${Boost_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rvo2
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

link_directories(/usr/local/lib)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES} RVO)
add_executable(rvo2_node src/rvo2_node.cpp src/PoseSubscriber.cpp
    src/PrefVelSubscriber.cpp)
target_link_libraries(rvo2_node ${catkin_LIBRARIES} RVO)
