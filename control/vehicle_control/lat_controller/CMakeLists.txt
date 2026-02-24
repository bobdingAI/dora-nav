cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(lat_controller_node)
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

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/dora_config.cmake)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${DORA_INCLUDE_DIR}
  ${DORA_OPERATOR_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${DORA_NAV_ROOT}/control/include
)

add_executable(lat_controller_node interface_lat.cpp pure_pursuit.cpp)
target_link_libraries(lat_controller_node
  ${DORA_LIB_PATH}
  $<$<BOOL:${Python3_FOUND}>:Python3::Python>
  m
  rt
  dl
  pthread
)




