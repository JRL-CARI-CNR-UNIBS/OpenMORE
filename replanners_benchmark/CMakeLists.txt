cmake_minimum_required(VERSION 3.0.2)
project(replanners_benchmark)
add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  replanners_lib
)
catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS replanners_lib
  DEPENDS
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(replanners_benchmark src/replanners_benchmark.cpp)
add_dependencies(replanners_benchmark ${catkin_EXPORTED_TARGETS})
target_link_libraries(replanners_benchmark
${catkin_LIBRARIES}
)
