#include "mujoco_sim_bridge.hpp"
#include <cstdlib>
#include <cstring>
#include <iostream>

extern "C" {
#include "node_api.h"
}

static dora_sim::SimConfig loadConfigFromEnv() {
    dora_sim::SimConfig config;

    const char* model_path = std::getenv("MUJOCO_MODEL_PATH");
    if (model_path) {
        config.model_path = model_path;
    } else {
        config.model_path = "./models/robot.xml";
    }

    const char* world_path = std::getenv("MUJOCO_WORLD_PATH");
    if (world_path) {
        config.world_path = world_path;
    }

    const char* timestep = std::getenv("SIM_TIMESTEP");
    if (timestep) {
        config.sim_timestep = std::atof(timestep);
    }

    const char* rtf = std::getenv("REAL_TIME_FACTOR");
    if (rtf) {
        config.real_time_factor = std::atof(rtf);
    }

    const char* lidar_rate = std::getenv("POINTCLOUD_RATE");
    if (lidar_rate) {
        config.lidar_output_rate_hz = std::atoi(lidar_rate);
    }

    const char* imu_rate = std::getenv("IMU_RATE");
    if (imu_rate) {
        config.imu_output_rate_hz = std::atoi(imu_rate);
    }

    const char* enable_viz = std::getenv("ENABLE_VISUALIZATION");
    if (enable_viz && std::strcmp(enable_viz, "1") == 0) {
        config.enable_visualization = true;
    }

    return config;
}

int main() {
    std::cout << "[MujocoSimBridge] Initializing..." << std::endl;

    // Initialize DORA node
    auto dora_context = init_dora_context_from_env();
    if (dora_context == nullptr) {
        std::cerr << "[MujocoSimBridge] Failed to initialize DORA node" << std::endl;
        return 1;
    }

    // Load configuration from environment
    auto config = loadConfigFromEnv();
    std::cout << "[MujocoSimBridge] Model path: " << config.model_path << std::endl;
    std::cout << "[MujocoSimBridge] Timestep: " << config.sim_timestep << "s" << std::endl;
    std::cout << "[MujocoSimBridge] Pointcloud rate: " << config.lidar_output_rate_hz << " Hz" << std::endl;
    std::cout << "[MujocoSimBridge] IMU rate: " << config.imu_output_rate_hz << " Hz" << std::endl;

    // Create and initialize bridge
    dora_sim::MujocoSimBridge bridge;
    if (!bridge.initialize(config)) {
        std::cerr << "[MujocoSimBridge] Failed to initialize simulation" << std::endl;
        return 1;
    }

    std::cout << "[MujocoSimBridge] Starting simulation loop..." << std::endl;

    // Run the main loop
    bridge.run(dora_context);

    std::cout << "[MujocoSimBridge] Shutdown complete" << std::endl;
    return 0;
}
