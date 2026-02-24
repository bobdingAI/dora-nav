cmake_minimum_required(VERSION 3.10)
project(mujoco_sim_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find Mujoco from pip package or system install
find_package(mujoco QUIET)
if(NOT mujoco_FOUND)
    # Locate mujoco from pip install
    execute_process(
        COMMAND python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
        OUTPUT_VARIABLE MUJOCO_PIP_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE MUJOCO_PIP_RESULT
    )
    if(MUJOCO_PIP_RESULT EQUAL 0 AND EXISTS "${MUJOCO_PIP_DIR}/include/mujoco/mujoco.h")
        message(STATUS "Found Mujoco via pip: ${MUJOCO_PIP_DIR}")
        set(MUJOCO_INCLUDE_DIR "${MUJOCO_PIP_DIR}/include")
        file(GLOB MUJOCO_LIB "${MUJOCO_PIP_DIR}/libmujoco.so*")
        if(NOT MUJOCO_LIB)
            file(GLOB MUJOCO_LIB "${MUJOCO_PIP_DIR}/libmujoco.dylib*")
        endif()
        add_library(mujoco::mujoco SHARED IMPORTED)
        list(GET MUJOCO_LIB 0 MUJOCO_LIB_PATH)
        set_target_properties(mujoco::mujoco PROPERTIES
            IMPORTED_LOCATION "${MUJOCO_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
        )
    else()
        message(FATAL_ERROR "Mujoco not found. Install via: pip install mujoco")
    endif()
endif()

# Find OpenMP for parallel raycasting
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# DORA paths - search multiple locations
set(DORA_INCLUDE_DIR "")
set(DORA_OPERATOR_DIR "")
set(DORA_LIB_PATH "")

# Search order for DORA headers
foreach(_dora_base
    "$ENV{HOME}/Public/dora"
    "$ENV{HOME}/dora"
    "$ENV{HOME}/dora_project/dddddd/dora-0.3.8"
)
    if(EXISTS "${_dora_base}/apis/c/node/node_api.h" AND NOT DORA_INCLUDE_DIR)
        set(DORA_INCLUDE_DIR "${_dora_base}/apis/c/node")
        set(DORA_OPERATOR_DIR "${_dora_base}/apis/c/operator")
    endif()
endforeach()

# Search order for DORA library
foreach(_dora_lib
    "$ENV{HOME}/Public/dora/target/release/libdora_node_api_c.a"
    "$ENV{HOME}/Public/dora/target/debug/libdora_node_api_c.a"
    "$ENV{HOME}/dora/target/release/libdora_node_api_c.a"
    "$ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a"
    "$ENV{HOME}/dora_project/dddddd/dora-0.3.8/target/x86_64-unknown-linux-gnu/release/libdora_node_api_c.a"
)
    if(EXISTS "${_dora_lib}" AND NOT DORA_LIB_PATH)
        set(DORA_LIB_PATH "${_dora_lib}")
    endif()
endforeach()

if(NOT DORA_INCLUDE_DIR)
    message(FATAL_ERROR "DORA C API headers not found")
endif()
if(NOT DORA_LIB_PATH)
    message(FATAL_ERROR "DORA C API library not found")
endif()

# Find Python (needed by DORA's pyo3 dependency)
find_package(Python3 COMPONENTS Development)
if(NOT Python3_FOUND)
    # Fallback to python3-config
    execute_process(
        COMMAND python3-config --ldflags --embed
        OUTPUT_VARIABLE PYTHON_LDFLAGS
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE PYTHON_CONFIG_RESULT
    )
    if(NOT PYTHON_CONFIG_RESULT EQUAL 0)
        execute_process(
            COMMAND python3-config --ldflags
            OUTPUT_VARIABLE PYTHON_LDFLAGS
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    endif()
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../control/include
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

if(Python3_FOUND)
    target_link_libraries(mujoco_sim_bridge
        mujoco::mujoco
        ${DORA_LIB_PATH}
        Python3::Python
        rt
        pthread
        dl
        m
    )
else()
    target_link_libraries(mujoco_sim_bridge
        mujoco::mujoco
        ${DORA_LIB_PATH}
        rt
        pthread
        dl
        m
    )
    target_link_options(mujoco_sim_bridge PRIVATE ${PYTHON_LDFLAGS})
endif()

# Optional: Standalone LiDAR-only simulator
add_executable(mujoco_lidar_sim
    src/lidar_main.cpp
    src/lidar_simulator.cpp
)

if(Python3_FOUND)
    target_link_libraries(mujoco_lidar_sim
        mujoco::mujoco
        ${DORA_LIB_PATH}
        Python3::Python
        rt
        pthread
        dl
        m
    )
else()
    target_link_libraries(mujoco_lidar_sim
        mujoco::mujoco
        ${DORA_LIB_PATH}
        rt
        pthread
        dl
        m
    )
    target_link_options(mujoco_lidar_sim PRIVATE ${PYTHON_LDFLAGS})
endif()

message(STATUS "DORA include: ${DORA_INCLUDE_DIR}")
message(STATUS "DORA library: ${DORA_LIB_PATH}")
