cmake_minimum_required(VERSION 3.16...3.27)

project(lidar LANGUAGES CXX)
set(CMAKE_WARN_DEVELOPER OFF)


# set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)


include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/dora_config.cmake)

# Rerun SDK - try system install, fallback to local build
find_package(rerun_sdk QUIET)
if(NOT rerun_sdk_FOUND)
    set(RERUN_SDK_BUILD_DIR "/tmp/rerun_cpp_sdk/rerun_cpp_sdk/build")
    set(RERUN_CPP_DIR "/tmp/rerun_cpp_sdk/rerun_cpp_sdk")
    if(EXISTS "${RERUN_SDK_BUILD_DIR}/librerun_sdk.a")
        message(STATUS "Found rerun_sdk at ${RERUN_SDK_BUILD_DIR}")
        add_library(rerun_sdk STATIC IMPORTED)
        set_target_properties(rerun_sdk PROPERTIES
            IMPORTED_LOCATION "${RERUN_SDK_BUILD_DIR}/librerun_sdk.a"
            INTERFACE_INCLUDE_DIRECTORIES "${RERUN_CPP_DIR}/src"
        )
        # Arrow libraries built by rerun
        add_library(arrow_lib STATIC IMPORTED)
        set_target_properties(arrow_lib PROPERTIES
            IMPORTED_LOCATION "${RERUN_SDK_BUILD_DIR}/arrow/lib/libarrow.a"
            INTERFACE_INCLUDE_DIRECTORIES "${RERUN_SDK_BUILD_DIR}/arrow/include"
        )
        add_library(arrow_deps STATIC IMPORTED)
        set_target_properties(arrow_deps PROPERTIES
            IMPORTED_LOCATION "${RERUN_SDK_BUILD_DIR}/arrow/lib/libarrow_bundled_dependencies.a"
        )
        # Rerun C library
        add_library(rerun_c STATIC IMPORTED)
        set_target_properties(rerun_c PROPERTIES
            IMPORTED_LOCATION "${RERUN_CPP_DIR}/lib/librerun_c__linux_x64.a"
        )
    else()
        message(FATAL_ERROR "rerun_sdk not found. Build it from https://github.com/rerun-io/rerun/releases")
    endif()
endif()

find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${DORA_OPERATOR_DIR}
  ${DORA_NAV_ROOT}/include
)

add_executable(to_rerun src/points_to_rerun.cpp)
target_link_options(to_rerun PRIVATE "-Wl,--allow-multiple-definition")

if(TARGET arrow_lib)
    target_link_libraries(to_rerun PRIVATE
      ${PCL_LIBRARIES}
      ${DORA_LIB_PATH}
      $<$<BOOL:${Python3_FOUND}>:Python3::Python>
      rerun_sdk
      rerun_c
      arrow_lib
      arrow_deps
      m
      rt
      dl
      pthread
    )
else()
    target_link_libraries(to_rerun PRIVATE
      ${PCL_LIBRARIES}
      ${DORA_LIB_PATH}
      $<$<BOOL:${Python3_FOUND}>:Python3::Python>
      rerun_sdk
      m
      rt
      dl
      pthread
    )
endif()
