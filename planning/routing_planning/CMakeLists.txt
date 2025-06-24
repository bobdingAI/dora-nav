cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(routing_planning)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

find_package(Eigen3 QUIET)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
  ${EIGEN3_LIBRARY_DIRS}
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/operator
  $ENV{HOME}/dora_project/0326_yewai/include
  $ENV{HOME}/dora_project/0326_yewai/planning/routing_planning/include/ 
)

add_executable(routing_planning_node node_routing_core.cpp frenet.cpp pathplanning.cpp)
target_link_libraries(routing_planning_node
  ${EIGEN_LIBRARIES}
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
)




