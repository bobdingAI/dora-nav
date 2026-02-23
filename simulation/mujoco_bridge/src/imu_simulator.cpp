#include "imu_simulator.hpp"
#include <cstring>
#include <cmath>
#include <sys/time.h>

namespace dora_sim {

IMUSimulator::IMUSimulator(const IMUConfig& config)
    : config_(config)
    , rng_(std::random_device{}())
    , accel_noise_(0.0f, config.accel_noise_stddev)
    , gyro_noise_(0.0f, config.gyro_noise_stddev)
{
}

void IMUSimulator::setConfig(const IMUConfig& config) {
    config_ = config;
    accel_noise_ = std::normal_distribution<float>(0.0f, config.accel_noise_stddev);
    gyro_noise_ = std::normal_distribution<float>(0.0f, config.gyro_noise_stddev);
}

IMUMessage IMUSimulator::generateIMU(
    const mjModel* model,
    const mjData* data,
    int robot_body_id
) {
    IMUMessage imu;

    // Get timestamp
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    imu.stamp = static_cast<double>(tv.tv_sec) + static_cast<double>(tv.tv_usec) * 1e-6;

    // Get body rotation matrix for frame transformation
    const mjtNum* body_mat = data->xmat + robot_body_id * 9;

    // Get linear acceleration from Mujoco
    // cacc contains [linear_acc(3), angular_acc(3)] for each body
    const mjtNum* body_cacc = data->cacc + robot_body_id * 6;

    // World frame acceleration
    mjtNum world_accel[3] = {body_cacc[0], body_cacc[1], body_cacc[2]};

    // Add gravity in world frame (Z-up convention)
    if (config_.add_gravity) {
        world_accel[2] += config_.gravity;
    }

    // Transform acceleration to body frame
    // We need the inverse (transpose) of the rotation matrix
    mjtNum body_mat_T[9];
    mju_transpose(body_mat_T, body_mat, 3, 3);

    mjtNum body_accel[3];
    mju_mulMatVec3(body_accel, body_mat_T, world_accel);

    imu.linear_acceleration.x = static_cast<float>(body_accel[0]);
    imu.linear_acceleration.y = static_cast<float>(body_accel[1]);
    imu.linear_acceleration.z = static_cast<float>(body_accel[2]);

    // Get angular velocity from Mujoco
    // cvel contains [linear_vel(3), angular_vel(3)] for each body
    const mjtNum* body_cvel = data->cvel + robot_body_id * 6;

    // World frame angular velocity
    mjtNum world_angvel[3] = {body_cvel[3], body_cvel[4], body_cvel[5]};

    // Transform to body frame
    mjtNum body_angvel[3];
    mju_mulMatVec3(body_angvel, body_mat_T, world_angvel);

    imu.angular_velocity.x = static_cast<float>(body_angvel[0]);
    imu.angular_velocity.y = static_cast<float>(body_angvel[1]);
    imu.angular_velocity.z = static_cast<float>(body_angvel[2]);

    // Add noise if enabled
    if (config_.add_noise) {
        imu.linear_acceleration.x += accel_noise_(rng_) + config_.accel_bias;
        imu.linear_acceleration.y += accel_noise_(rng_) + config_.accel_bias;
        imu.linear_acceleration.z += accel_noise_(rng_) + config_.accel_bias;

        imu.angular_velocity.x += gyro_noise_(rng_) + config_.gyro_bias;
        imu.angular_velocity.y += gyro_noise_(rng_) + config_.gyro_bias;
        imu.angular_velocity.z += gyro_noise_(rng_) + config_.gyro_bias;
    }

    return imu;
}

std::vector<uint8_t> IMUSimulator::serialize(const IMUMessage& imu) {
    // IMU message format matches include/imu_msg.h
    // Total size: 8 (stamp) + 12 (linear_acc) + 12 (angular_vel) = 32 bytes

    std::vector<uint8_t> buffer(sizeof(IMUMessage));
    std::memcpy(buffer.data(), &imu, sizeof(IMUMessage));
    return buffer;
}

void IMUSimulator::transformToBodyFrame(
    const mjtNum* world_vec,
    const mjtNum* body_mat,
    Vector3& body_vec
) {
    // Rotate world vector to body frame using transpose of rotation matrix
    mjtNum body_mat_T[9];
    mju_transpose(body_mat_T, body_mat, 3, 3);

    mjtNum result[3];
    mju_mulMatVec3(result, body_mat_T, world_vec);

    body_vec.x = static_cast<float>(result[0]);
    body_vec.y = static_cast<float>(result[1]);
    body_vec.z = static_cast<float>(result[2]);
}

}  // namespace dora_sim
