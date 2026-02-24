cmake_minimum_required(VERSION 2.8.3)

project(hdl_localization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/dora_config.cmake)

set(NDT_OMP_LIB_PATH ${CMAKE_SOURCE_DIR}/3rdparty/hdl_ndt_omp/build)
link_directories(${NDT_OMP_LIB_PATH})

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${DORA_OPERATOR_DIR}
  ./include
  ./3rdparty/hdl_ndt_omp/include
)

add_executable(${PROJECT_NAME} src/dora_hdl_node.cpp src/pose_estimator.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${DORA_LIB_PATH}
  $<$<BOOL:${Python3_FOUND}>:Python3::Python>
  ndt_omp
  rt
  pthread
  dl
  m
)
