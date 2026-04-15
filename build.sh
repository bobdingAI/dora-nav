#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# build.sh — Unified build script for dora-nav
#
# Usage:
#   ./build.sh              Build everything (C++ nodes + Python deps)
#   ./build.sh cpp          Build only C++ nodes
#   ./build.sh python       Install only Python dependencies
#   ./build.sh clean        Remove all build/ directories
#   ./build.sh <node_dir>   Build a single C++ node (e.g. ./build.sh planning/routing_planning)
# ============================================================

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

# C++ node directories (each must contain a CMakeLists.txt)
CPP_NODES=(
    "driver/HWT9053modbus"
    "driver/rslidar_driver"
    "driver/geo_pos_conv"
    "driver/gnss_poser"
    "driver/imu/imu_wit"
    "driver/imu_corrector"
    "driver/livox/livox_dora_driver2"
    "localization/dora-hdl_localization"
    "map/pub_road"
    "map/road_line_publisher"
    "mapping/ndt_mapping"
    "peception/euclidean_cluster"
    "peception/obj_track"
    "peception/patchwork-plusplus-ros-master"
    "planning/routing_planning"
    "planning/mission_planning/task_exc"
    "planning/mission_planning/task_pub"
    "control/vehicle_control/lat_controller"
    "control/vehicle_control/lon_controller"
    "control/vehicle_control/vehicle_chassis_n3"
    "rerun"
    "vehicle/adora_chassis_control_dora_node"
    "simulation/mujoco_bridge"
)

# ---------- helpers ------------------------------------------

log()  { printf "\033[1;34m[build]\033[0m %s\n" "$*"; }
ok()   { printf "\033[1;32m[build]\033[0m %s\n" "$*"; }
fail() { printf "\033[1;31m[build]\033[0m %s\n" "$*" >&2; }

build_cpp_node() {
    local node_dir="$1"
    local full_path="${ROOT_DIR}/${node_dir}"

    if [[ ! -f "${full_path}/CMakeLists.txt" ]]; then
        fail "SKIP ${node_dir} — no CMakeLists.txt"
        return 1
    fi

    log "Building ${node_dir} ..."
    mkdir -p "${full_path}/build"
    cmake -S "${full_path}" -B "${full_path}/build" -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    cmake --build "${full_path}/build" -j "$(nproc 2>/dev/null || sysctl -n hw.ncpu)" 2>&1 | tail -5
    ok "Done: ${node_dir}"
}

build_ndt_omp() {
    local ndt_dir="${ROOT_DIR}/localization/dora-hdl_localization/3rdparty/hdl_ndt_omp"
    if [[ -d "${ndt_dir}" ]]; then
        log "Building NDT-OMP dependency ..."
        mkdir -p "${ndt_dir}/build"
        cmake -S "${ndt_dir}" -B "${ndt_dir}/build" 2>&1 | tail -3
        cmake --build "${ndt_dir}/build" -j "$(nproc 2>/dev/null || sysctl -n hw.ncpu)" 2>&1 | tail -5
        ok "Done: NDT-OMP"
    fi
}

install_python_deps() {
    log "Installing Python dependencies ..."
    pip install -r "${ROOT_DIR}/python/requirements.txt"
    ok "Python dependencies installed"
}

clean_all() {
    log "Cleaning all build directories ..."
    find "${ROOT_DIR}" -type d -name build -exec rm -rf {} + 2>/dev/null || true
    ok "Clean complete"
}

# ---------- main ---------------------------------------------

MODE="${1:-all}"

case "${MODE}" in
    all)
        install_python_deps
        build_ndt_omp
        for node in "${CPP_NODES[@]}"; do
            build_cpp_node "${node}" || true
        done
        echo ""
        ok "Full build complete."
        ;;
    cpp)
        build_ndt_omp
        for node in "${CPP_NODES[@]}"; do
            build_cpp_node "${node}" || true
        done
        echo ""
        ok "C++ build complete."
        ;;
    python)
        install_python_deps
        ;;
    clean)
        clean_all
        ;;
    *)
        # Treat argument as a single node path
        if [[ -d "${ROOT_DIR}/${MODE}" ]]; then
            build_cpp_node "${MODE}"
        else
            fail "Unknown target: ${MODE}"
            echo "Usage: $0 [all|cpp|python|clean|<node_dir>]"
            exit 1
        fi
        ;;
esac
