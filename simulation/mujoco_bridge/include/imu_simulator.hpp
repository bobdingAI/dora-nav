#ifndef IMU_SIMULATOR_HPP
#define IMU_SIMULATOR_HPP

#include <mujoco/mujoco.h>
#include <random>
#include <cstdint>

namespace dora_sim {

struct IMUConfig {
    float accel_noise_stddev = 0.01f;   // m/s^2
    float gyro_noise_stddev = 0.001f;   // rad/s
    float accel_bias = 0.0f;            // m/s^2
    float gyro_bias = 0.0f;             // rad/s
    bool add_noise = true;
    bool add_gravity = true;            // Add gravity to accelerometer
    float gravity = 9.81f;              // m/s^2
};

// IMU message format matching HWT9053 driver output
// Must match include/imu_msg.h
struct Vector3 {
    float x;
    float y;
    float z;
};

struct IMUMessage {
    double stamp;                       // Unix timestamp (seconds)
    Vector3 linear_acceleration;        // m/s^2
    Vector3 angular_velocity;           // rad/s
};

class IMUSimulator {
public:
    IMUSimulator(const IMUConfig& config = IMUConfig());
    ~IMUSimulator() = default;

    void setConfig(const IMUConfig& config);

    // Generate IMU data from Mujoco state
    // robot_body_id: The body ID of the robot base where IMU is mounted
    IMUMessage generateIMU(
        const mjModel* model,
        const mjData* data,
        int robot_body_id
    );

    // Serialize to binary format for DORA
    std::vector<uint8_t> serialize(const IMUMessage& imu);

private:
    void transformToBodyFrame(
        const mjtNum* world_vec,
        const mjtNum* body_mat,
        Vector3& body_vec
    );

    IMUConfig config_;

    std::mt19937 rng_;
    std::normal_distribution<float> accel_noise_;
    std::normal_distribution<float> gyro_noise_;
};

}  // namespace dora_sim

#endif  // IMU_SIMULATOR_HPP
