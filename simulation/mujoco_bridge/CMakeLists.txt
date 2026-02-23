cmake_minimum_required(VERSION 3.10)
project(mujoco_sim_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find Mujoco
find_package(mujoco REQUIRED)

# Find OpenMP for parallel raycasting
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# DORA paths - adjust based on your installation
set(DORA_INCLUDE_DIR $ENV{HOME}/dora/apis/c/node)
set(DORA_OPERATOR_DIR $ENV{HOME}/dora/apis/c/operator)

# Alternative DORA paths (fallback)
if(NOT EXISTS ${DORA_INCLUDE_DIR})
    set(DORA_INCLUDE_DIR $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/node)
    set(DORA_OPERATOR_DIR $ENV{HOME}/dora_project/dddddd/dora-0.3.8/apis/c/operator)
endif()

# DORA library - adjust architecture as needed
set(DORA_LIB_PATH $ENV{HOME}/dora/target/release/libdora_node_api_c.a)
if(NOT EXISTS ${DORA_LIB_PATH})
    # Try aarch64 path (for NVIDIA Orin)
    set(DORA_LIB_PATH $ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a)
endif()
if(NOT EXISTS ${DORA_LIB_PATH})
    # Try x86_64 path
    set(DORA_LIB_PATH $ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/x86_64-unknown-linux-gnu/release/libdora_node_api_c.a)
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${DORA_INCLUDE_DIR}
    ${DORA_OPERATOR_DIR}
)

# Main simulation bridge node
add_executable(mujoco_sim_bridge
    src/main.cpp
    src/mujoco_sim_bridge.cpp
    src/lidar_simulator.cpp
    src/imu_simulator.cpp
)

target_link_libraries(mujoco_sim_bridge
    mujoco::mujoco
    ${DORA_LIB_PATH}
    rt
    pthread
)

# Optional: Standalone LiDAR-only simulator
add_executable(mujoco_lidar_sim
    src/lidar_main.cpp
    src/lidar_simulator.cpp
)

target_link_libraries(mujoco_lidar_sim
    mujoco::mujoco
    ${DORA_LIB_PATH}
    rt
    pthread
)

message(STATUS "DORA include: ${DORA_INCLUDE_DIR}")
message(STATUS "DORA library: ${DORA_LIB_PATH}")
