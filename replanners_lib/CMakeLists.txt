cmake_minimum_required(VERSION 3.0.2)
project(replanners_lib)
add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
graph_core
roscpp
object_loader_msgs
moveit_planning_helper
geometry_msgs
subscription_notifier
jsk_rviz_plugins
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS graph_core roscpp object_loader_msgs moveit_planning_helper geometry_msgs subscription_notifier jsk_rviz_plugins
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_library(${PROJECT_NAME}
src/moveit_utils.cpp
src/trajectory.cpp
src/replanners/replanner_base.cpp
src/replanners/MPRRT.cpp
src/replanners/DRRTStar.cpp
src/replanners/DRRT.cpp
src/replanners/anytimeDRRT.cpp
src/replanners/MARS.cpp
src/replanner_managers/replanner_manager_base.cpp
src/replanner_managers/replanner_manager_DRRTStar.cpp
src/replanner_managers/replanner_manager_DRRT.cpp
src/replanner_managers/replanner_manager_MARS.cpp
src/replanner_managers/replanner_manager_anytimeDRRT.cpp
src/replanner_managers/replanner_manager_MPRRT.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(crash_test_replanners src/test/crash_test_replanner.cpp)
add_dependencies(crash_test_replanners ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(crash_test_replanners
${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(crash_test_replanner_managers src/test/crash_test_replanner_manager.cpp)
add_dependencies(crash_test_replanner_managers ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(crash_test_replanner_managers
${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(example_replanner examples/src/example_replanner.cpp)
add_dependencies(example_replanner ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_replanner
${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(example_replanner_manager examples/src/example_replanner_manager.cpp)
add_dependencies(example_replanner_manager ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_replanner_manager
${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(quick_example examples/src/quick_example.cpp)
add_dependencies(quick_example ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(quick_example
${PROJECT_NAME}
${catkin_LIBRARIES}
)
