#ifndef LIDAR_SIMULATOR_HPP
#define LIDAR_SIMULATOR_HPP

#include <mujoco/mujoco.h>
#include <vector>
#include <cstdint>
#include <random>

namespace dora_sim {

struct LidarConfig {
    int horizontal_rays = 1080;         // Number of rays per scan
    int vertical_beams = 16;            // Number of vertical beams (e.g., RSHELIOS-16)
    float horizontal_fov = 360.0f;      // degrees
    float vertical_fov = 30.0f;         // degrees (-15 to +15)
    float min_range = 0.5f;             // meters
    float max_range = 100.0f;           // meters
    float noise_stddev = 0.01f;         // meters
    bool add_noise = true;
};

// Point format matching rslidar driver output
struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

class LidarSimulator {
public:
    LidarSimulator(const LidarConfig& config = LidarConfig());
    ~LidarSimulator() = default;

    void setConfig(const LidarConfig& config);

    // Generate point cloud from Mujoco state
    // Returns binary data in DORA pointcloud format:
    // [seq(4) + padding(4) + timestamp(8)] + [points * 16 bytes]
    std::vector<uint8_t> generatePointCloud(
        const mjModel* model,
        const mjData* data,
        int lidar_body_id
    );

    // Get number of points in last scan
    size_t getLastPointCount() const { return last_point_count_; }

private:
    void computeRayDirection(
        float h_angle,
        float v_angle,
        const mjtNum* lidar_mat,
        mjtNum* ray_dir
    );

    float computeIntensity(float distance, int geom_id);

    std::vector<uint8_t> serializePointCloud(const std::vector<PointXYZI>& points);

    LidarConfig config_;
    std::vector<float> vertical_angles_;
    uint32_t seq_counter_ = 0;
    size_t last_point_count_ = 0;

    std::mt19937 rng_;
    std::normal_distribution<float> noise_dist_;
};

}  // namespace dora_sim

#endif  // LIDAR_SIMULATOR_HPP
