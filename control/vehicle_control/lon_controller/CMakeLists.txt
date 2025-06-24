cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(lon_controller_node)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")




include_directories(
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/operator
  $ENV{HOME}/dora_project/0326_yewai/control/vehicle_control/lon_controller/include/
  $ENV{HOME}/dora_project/0326_yewai/control/include/
)

add_executable(lon_controller_node interface_lon.cpp lon_controller.cpp)
target_link_libraries(lon_controller_node
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
)




