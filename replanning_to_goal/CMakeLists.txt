cmake_minimum_required(VERSION 3.0.2)
project(replanning_to_goal)
add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
graph_core
graph_replanning
roscpp
object_loader_msgs
moveit_planning_helper
geometry_msgs
configuration_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES replanning_to_goal
  CATKIN_DEPENDS graph_core graph_replanning roscpp object_loader_msgs moveit_planning_helper geometry_msgs configuration_msgs
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_library(${PROJECT_NAME}
src/replanner_to_goal.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
