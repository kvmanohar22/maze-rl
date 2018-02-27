cmake_minimum_required(VERSION 2.8.3)
project(maze)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "include"
)

set(lib_files src/simulator.cpp)

add_library(maze_LIBS ${lib_files})

add_executable(maze src/main.cpp)
target_link_libraries(maze ${catkin_LIBRARIES} maze_LIBS)
