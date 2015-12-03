cmake_minimum_required(VERSION 2.8.3)
project(object_definer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS interactive_markers roscpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(definer
  src/node.cpp
  src/InteractiveObject.cpp
  src/OwlIndividual.cpp)

target_link_libraries(definer
  ${catkin_LIBRARIES})
