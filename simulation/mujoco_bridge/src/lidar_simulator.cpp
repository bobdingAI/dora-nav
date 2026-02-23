#include "lidar_simulator.hpp"
#include <cmath>
#include <cstring>
#include <sys/time.h>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace dora_sim {

LidarSimulator::LidarSimulator(const LidarConfig& config)
    : config_(config)
    , rng_(std::random_device{}())
    , noise_dist_(0.0f, config.noise_stddev)
{
    // Pre-compute vertical beam angles for multi-beam LiDAR
    // Example: RSHELIOS-16 has 16 beams from -15 to +15 degrees
    vertical_angles_.resize(config_.vertical_beams);
    float v_step = config_.vertical_fov / (config_.vertical_beams - 1);
    float v_start = -config_.vertical_fov / 2.0f;

    for (int i = 0; i < config_.vertical_beams; i++) {
        vertical_angles_[i] = (v_start + i * v_step) * M_PI / 180.0f;  // Convert to radians
    }
}

void LidarSimulator::setConfig(const LidarConfig& config) {
    config_ = config;
    noise_dist_ = std::normal_distribution<float>(0.0f, config.noise_stddev);

    // Recompute vertical angles
    vertical_angles_.resize(config_.vertical_beams);
    float v_step = config_.vertical_fov / (config_.vertical_beams - 1);
    float v_start = -config_.vertical_fov / 2.0f;

    for (int i = 0; i < config_.vertical_beams; i++) {
        vertical_angles_[i] = (v_start + i * v_step) * M_PI / 180.0f;
    }
}

std::vector<uint8_t> LidarSimulator::generatePointCloud(
    const mjModel* model,
    const mjData* data,
    int lidar_body_id
) {
    std::vector<PointXYZI> points;
    points.reserve(config_.horizontal_rays * config_.vertical_beams);

    // Get LiDAR position and orientation
    const mjtNum* lidar_pos = data->xpos + lidar_body_id * 3;
    const mjtNum* lidar_mat = data->xmat + lidar_body_id * 9;

    float h_step = (config_.horizontal_fov * M_PI / 180.0f) / config_.horizontal_rays;

    // Parallel raycasting for performance
    #ifdef _OPENMP
    #pragma omp parallel
    {
        std::vector<PointXYZI> local_points;
        local_points.reserve(config_.horizontal_rays * config_.vertical_beams / omp_get_num_threads());

        #pragma omp for nowait
        for (int h = 0; h < config_.horizontal_rays; h++) {
            float h_angle = h * h_step;

            for (int v = 0; v < config_.vertical_beams; v++) {
                float v_angle = vertical_angles_[v];

                // Compute ray direction in LiDAR frame
                mjtNum ray_local[3];
                ray_local[0] = std::cos(v_angle) * std::cos(h_angle);
                ray_local[1] = std::cos(v_angle) * std::sin(h_angle);
                ray_local[2] = std::sin(v_angle);

                // Transform to world frame
                mjtNum ray_world[3];
                mju_mulMatVec3(ray_world, lidar_mat, ray_local);

                // Perform raycast
                int geomid[1];
                mjtNum normal[3];
                mjtNum dist = mj_ray(
                    model, data,
                    lidar_pos, ray_world,
                    nullptr,        // Check all geom groups
                    1,              // Include static geoms
                    lidar_body_id,  // Exclude LiDAR body
                    geomid,
                    normal
                );

                // Check if hit within valid range
                if (dist > 0 && dist >= config_.min_range && dist <= config_.max_range) {
                    // Add noise if enabled
                    if (config_.add_noise) {
                        std::normal_distribution<float> noise(0.0f, config_.noise_stddev);
                        dist += noise(rng_);
                    }

                    // Compute point in LiDAR frame
                    PointXYZI p;
                    p.x = static_cast<float>(dist * ray_local[0]);
                    p.y = static_cast<float>(dist * ray_local[1]);
                    p.z = static_cast<float>(dist * ray_local[2]);
                    p.intensity = computeIntensity(static_cast<float>(dist), geomid[0]);

                    local_points.push_back(p);
                }
            }
        }

        #pragma omp critical
        points.insert(points.end(), local_points.begin(), local_points.end());
    }
    #else
    // Single-threaded fallback
    for (int h = 0; h < config_.horizontal_rays; h++) {
        float h_angle = h * h_step;

        for (int v = 0; v < config_.vertical_beams; v++) {
            float v_angle = vertical_angles_[v];

            // Compute ray direction in LiDAR frame
            mjtNum ray_local[3];
            ray_local[0] = std::cos(v_angle) * std::cos(h_angle);
            ray_local[1] = std::cos(v_angle) * std::sin(h_angle);
            ray_local[2] = std::sin(v_angle);

            // Transform to world frame
            mjtNum ray_world[3];
            mju_mulMatVec3(ray_world, lidar_mat, ray_local);

            // Perform raycast
            int geomid[1];
            mjtNum normal[3];
            mjtNum dist = mj_ray(
                model, data,
                lidar_pos, ray_world,
                nullptr,
                1,
                lidar_body_id,
                geomid,
                normal
            );

            if (dist > 0 && dist >= config_.min_range && dist <= config_.max_range) {
                if (config_.add_noise) {
                    dist += noise_dist_(rng_);
                }

                PointXYZI p;
                p.x = static_cast<float>(dist * ray_local[0]);
                p.y = static_cast<float>(dist * ray_local[1]);
                p.z = static_cast<float>(dist * ray_local[2]);
                p.intensity = computeIntensity(static_cast<float>(dist), geomid[0]);

                points.push_back(p);
            }
        }
    }
    #endif

    last_point_count_ = points.size();
    return serializePointCloud(points);
}

void LidarSimulator::computeRayDirection(
    float h_angle,
    float v_angle,
    const mjtNum* lidar_mat,
    mjtNum* ray_dir
) {
    // Ray in LiDAR local frame
    mjtNum local_dir[3];
    local_dir[0] = std::cos(v_angle) * std::cos(h_angle);
    local_dir[1] = std::cos(v_angle) * std::sin(h_angle);
    local_dir[2] = std::sin(v_angle);

    // Transform to world frame using rotation matrix
    mju_mulMatVec3(ray_dir, lidar_mat, local_dir);
}

float LidarSimulator::computeIntensity(float distance, int geom_id) {
    // Simple intensity model based on distance
    // Real LiDARs have more complex models based on surface reflectivity
    float base_intensity = 100.0f;
    float attenuation = 1.0f / (1.0f + 0.01f * distance * distance);
    return base_intensity * attenuation;
}

std::vector<uint8_t> LidarSimulator::serializePointCloud(const std::vector<PointXYZI>& points) {
    // Format: [seq(4) + padding(4) + timestamp(8)] + [points * 16 bytes]
    // Matches rslidar_driver output format

    size_t header_size = 16;
    size_t point_size = sizeof(PointXYZI);  // 16 bytes
    size_t total_size = header_size + points.size() * point_size;

    std::vector<uint8_t> buffer(total_size);

    // Header: sequence number
    uint32_t* seq_ptr = reinterpret_cast<uint32_t*>(buffer.data());
    *seq_ptr = seq_counter_++;

    // Header: padding (4 bytes)
    uint32_t* padding_ptr = reinterpret_cast<uint32_t*>(buffer.data() + 4);
    *padding_ptr = 0;

    // Header: timestamp (microseconds since epoch)
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    uint64_t timestamp_us = static_cast<uint64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;
    uint64_t* timestamp_ptr = reinterpret_cast<uint64_t*>(buffer.data() + 8);
    *timestamp_ptr = timestamp_us;

    // Points data
    if (!points.empty()) {
        std::memcpy(buffer.data() + header_size, points.data(), points.size() * point_size);
    }

    return buffer;
}

}  // namespace dora_sim
