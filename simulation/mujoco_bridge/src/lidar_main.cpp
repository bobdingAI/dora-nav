// Standalone LiDAR-only simulator for hybrid simulation mode
// Use this when you want simulated LiDAR but real IMU

#include "lidar_simulator.hpp"
#include <mujoco/mujoco.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <atomic>

extern "C" {
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

static std::atomic<bool> running{true};

int main() {
    std::cout << "[MujocoLidarSim] Initializing standalone LiDAR simulator..." << std::endl;

    // Initialize DORA node
    auto dora_context = init_dora_node();
    if (dora_context == nullptr) {
        std::cerr << "[MujocoLidarSim] Failed to initialize DORA node" << std::endl;
        return 1;
    }

    // Load Mujoco model
    const char* model_path = std::getenv("MUJOCO_MODEL_PATH");
    if (!model_path) {
        model_path = "./models/robot.xml";
    }

    char error[1000] = "";
    mjModel* model = mj_loadXML(model_path, nullptr, error, 1000);
    if (!model) {
        std::cerr << "[MujocoLidarSim] Failed to load model: " << error << std::endl;
        return 1;
    }

    mjData* data = mj_makeData(model);
    if (!data) {
        std::cerr << "[MujocoLidarSim] Failed to create simulation data" << std::endl;
        mj_deleteModel(model);
        return 1;
    }

    // Find LiDAR body
    int lidar_body_id = 1;
    for (int i = 1; i < model->nbody; i++) {
        const char* name = mj_id2name(model, mjOBJ_BODY, i);
        if (name && (strstr(name, "lidar") || strstr(name, "laser"))) {
            lidar_body_id = i;
            break;
        }
    }

    // Configure LiDAR
    dora_sim::LidarConfig config;

    const char* lidar_rays = std::getenv("LIDAR_RAYS");
    if (lidar_rays) config.horizontal_rays = std::atoi(lidar_rays);

    const char* lidar_range = std::getenv("LIDAR_RANGE");
    if (lidar_range) config.max_range = std::atof(lidar_range);

    dora_sim::LidarSimulator lidar_sim(config);

    std::cout << "[MujocoLidarSim] Starting with " << config.horizontal_rays
              << " rays, " << config.max_range << "m range" << std::endl;

    // Main loop
    while (running) {
        void* event = dora_next_event(dora_context);
        if (event == nullptr) break;

        enum DoraEventType event_type = read_dora_event_type(event);

        if (event_type == DoraEventType_Input) {
            char* input_id;
            size_t input_id_len;
            read_dora_input_id(event, &input_id, &input_id_len);

            if (strncmp(input_id, "tick", 4) == 0) {
                // Advance simulation (optional - for dynamic scenes)
                mj_step(model, data);

                // Generate and send pointcloud
                auto pointcloud = lidar_sim.generatePointCloud(model, data, lidar_body_id);

                std::string output_id = "pointcloud";
                dora_send_output(
                    dora_context,
                    output_id.c_str(),
                    output_id.length(),
                    reinterpret_cast<char*>(pointcloud.data()),
                    pointcloud.size()
                );
            }
        } else if (event_type == DoraEventType_Stop) {
            running = false;
        }

        free_dora_event(event);
    }

    // Cleanup
    mj_deleteData(data);
    mj_deleteModel(model);

    std::cout << "[MujocoLidarSim] Shutdown complete" << std::endl;
    return 0;
}
