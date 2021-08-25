cmake_minimum_required(VERSION 3.0.2)
project(replanners_lib)
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
  LIBRARIES replanners_lib
  CATKIN_DEPENDS graph_core graph_replanning roscpp object_loader_msgs moveit_planning_helper geometry_msgs configuration_msgs
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_library(${PROJECT_NAME}
src/replanners/replanner_base.cpp
src/replanners/replanner_to_goal.cpp
src/replanners/DRRTstar.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_replanner src/test/test_replanner.cpp)
add_dependencies(test_replanner ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_replanner
${PROJECT_NAME}
${catkin_LIBRARIES}
)
