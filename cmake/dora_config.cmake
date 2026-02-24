# Shared DORA configuration for all dora-nav nodes
# Include this from CMakeLists.txt: include(${CMAKE_SOURCE_DIR}/../../cmake/dora_config.cmake)

# Find DORA headers
set(DORA_INCLUDE_DIR "")
set(DORA_OPERATOR_DIR "")
set(DORA_LIB_PATH "")

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

# Python (needed by DORA's pyo3)
find_package(Python3 COMPONENTS Development QUIET)

# Project-level includes
get_filename_component(DORA_NAV_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

message(STATUS "DORA include: ${DORA_INCLUDE_DIR}")
message(STATUS "DORA library: ${DORA_LIB_PATH}")
message(STATUS "DORA-NAV root: ${DORA_NAV_ROOT}")
