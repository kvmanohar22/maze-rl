cmake_minimum_required(VERSION 2.8.3)
project(maze)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "include"
)

set(lib_files 
   src/simulator.cpp 
   src/rl.cpp
   include/maze/utils.hpp
   )
add_library(maze_LIBRARIES ${lib_files})
list(APPEND LIBS ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} maze_LIBRARIES)

add_executable(maze src/main.cpp)
target_link_libraries(maze ${LIBS})
