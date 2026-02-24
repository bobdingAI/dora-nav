#ifndef MUJOCO_SIM_BRIDGE_HPP
#define MUJOCO_SIM_BRIDGE_HPP

#include <mujoco/mujoco.h>
#include <string>
#include <memory>
#include <atomic>

#include "lidar_simulator.hpp"
#include "imu_simulator.hpp"

namespace dora_sim {

struct SimConfig {
    std::string model_path;
    std::string world_path;
    double sim_timestep = 0.001;        // 1ms physics step
    double real_time_factor = 1.0;      // 1.0 = real-time
    int lidar_output_rate_hz = 10;      // 10Hz pointcloud
    int imu_output_rate_hz = 20;        // 20Hz IMU
    bool enable_visualization = false;
};

struct GroundTruthPose {
    float x;
    float y;
    float theta;  // radians
};

class MujocoSimBridge {
public:
    MujocoSimBridge();
    ~MujocoSimBridge();

    bool initialize(const SimConfig& config);
    void run(void* dora_context);
    void stop();

    GroundTruthPose getGroundTruthPose() const;

private:
    void stepSimulation(int num_steps);
    void processControlInput(const char* data, size_t len);
    void applyVehicleControl();
    bool shouldSendPointcloud(double sim_time) const;
    bool shouldSendIMU(double sim_time) const;

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    std::unique_ptr<LidarSimulator> lidar_sim_;
    std::unique_ptr<IMUSimulator> imu_sim_;

    SimConfig config_;
    double sim_time_ = 0.0;
    double last_pointcloud_time_ = 0.0;
    double last_imu_time_ = 0.0;

    int robot_body_id_ = -1;
    int lidar_body_id_ = -1;

    std::atomic<bool> running_{false};

    // Vehicle control state (from SteeringCmd / TrqBreCmd)
    float steering_angle_ = 0.0f;   // degrees
    float torque_pct_ = 0.0f;       // torque percentage
    float brake_value_ = 0.0f;
    uint8_t trq_enable_ = 0;
    uint8_t bre_enable_ = 0;
};

}  // namespace dora_sim

#endif  // MUJOCO_SIM_BRIDGE_HPP
