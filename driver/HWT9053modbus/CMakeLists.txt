cmake_minimum_required(VERSION 2.8.3)

project(imu_driver)

set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


include_directories(
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/operator
  $ENV{HOME}/dora_project/OH_slam_1211/include
  ./include
)
aux_source_directory(${PROJECT_SOURCE_DIR}/src  SRC_LIST)

add_executable(${PROJECT_NAME}_node ${SRC_LIST})



target_link_libraries(${PROJECT_NAME}_node
  $ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
  m
  rt
  dl 
)




