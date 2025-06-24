cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(road_lane_publisher)
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
link_directories(${EIGEN3_LIBRARY_DIRS})


include_directories(
  ${EIGEN3_INCLUDE_DIR}
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/operator
  $ENV{HOME}/dora_project/0326_yewai/include
  $ENV{HOME}/dora_project/0326_yewai/map/road_line_publisher/include
)

add_executable(road_lane_publisher_node road_lane.cpp frenet.cpp)
target_link_libraries(road_lane_publisher_node
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
  ${EIGEN_LIBRARIES}
  m
  rt
  dl 
  pthread
)




