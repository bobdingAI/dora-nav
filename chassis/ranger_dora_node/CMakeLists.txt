cmake_minimum_required(VERSION 3.10)
project(ranger_dora_node)

# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
 
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set C++ standard
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find required packages
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ugv_sdk/include
    $ENV{HOME}/dora/apis/c/node/ #dora的头文件路径 node_api.h
    $ENV{HOME}/dora/apis/c/operator/
    ./include
    /usr/local/include
)

# Link directories
# link_directories(
#     /usr/local/lib
#     ${CMAKE_CURRENT_SOURCE_DIR}/ugv_sdk/build/lib
# )

# Source files
set(SOURCES
    src/ranger_base_node.cpp
    src/ranger_messenger.cpp
)

# Create executable
add_executable(ranger_miniv3_node ${SOURCES})

# Link libraries
target_link_libraries(ranger_miniv3_node
    Threads::Threads
    #ugv_sdk
    yaml-cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ugv_sdk/build/lib/libugv_sdk.a
    $ENV{HOME}/dora/target/release/libdora_node_api_c.a
    m
    rt
    dl
    pthread
)

# Install target
#install(TARGETS ranger_miniv3_node
#    RUNTIME DESTINATION bin
#)
