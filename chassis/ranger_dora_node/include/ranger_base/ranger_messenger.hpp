/**
 * @file ranger_messenger.hpp
 * @date 2026-02-04
 * @brief DORA messenger for Ranger robot
 *
 * @copyright Copyright (c) 2021 AgileX Robotics
 * @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
 */

#ifndef RANGER_MESSENGER_HPP
#define RANGER_MESSENGER_HPP

// Standard includes
#include <string>
#include <memory>
#include <cmath>
#include <mutex>
#include <chrono>

// ugv_sdk includes
#include "ugv_sdk/details/robot_base/ranger_base.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

// JSON library
#include <nlohmann/json.hpp>

// Local includes
#include "ranger_base/ranger_params.hpp"
#include "ranger_base/ranger_msgs.h"

namespace westonrobot
{

  class RangerDoraMessenger
  {
  public:
    struct RobotParams
    {
      float track;
      float wheelbase;
      float max_linear_speed;
      float max_angular_speed;
      float max_speed_cmd;
      float max_steer_angle_central;
      float max_steer_angle_parallel;
      float max_round_angle;
      float min_turn_radius;
      float max_steer_angle_ackermann;
    };

    enum class RangerSubType
    {
      kRanger = 0,
      kRangerMiniV1,
      kRangerMiniV2,
      kRangerMiniV3
    };

  public:
    explicit RangerDoraMessenger(void *dora_context);
    ~RangerDoraMessenger();

    void Run();

  private:
    void LoadParameters();
    void SetupRobot();
    void PublishStateToROS();
    void TwistCmdCallback(const ranger_msgs::Twist &msg);
    float CalculateSteeringAngle(const ranger_msgs::Twist &msg, float &radius);
    void UpdateOdometry(float linear, float angular, float angle, float dt);

    float ConvertInnerAngleToCentral(float angle);
    float ConvertCentralAngleToInner(float angle);

    // Message conversion functions
    ranger_msgs::Odometry CreateOdometryMsg(float linear_x, float linear_y, float angular_z);
    ranger_msgs::SystemState CreateSystemStateMsg(const RangerCoreState &state);
    ranger_msgs::MotionState CreateMotionStateMsg(const RangerCoreState &state);
    ranger_msgs::ActuatorStateArray CreateActuatorStateMsg(const RangerActuatorState &actuator_state);
    ranger_msgs::BatteryState CreateBatteryStateMsg(const RangerCommonSensorState &sensor_state);

    // JSON conversion functions
    nlohmann::json ToJson(const ranger_msgs::Odometry &msg);
    nlohmann::json ToJson(const ranger_msgs::SystemState &msg);
    nlohmann::json ToJson(const ranger_msgs::MotionState &msg);
    nlohmann::json ToJson(const ranger_msgs::ActuatorStateArray &msg);
    nlohmann::json ToJson(const ranger_msgs::BatteryState &msg);

    ranger_msgs::Twist FromJson(const char *data, size_t data_len);

    // DORA communication
    int SendOutput(const std::string &output_id, const nlohmann::json &j_data);

    // Helper functions
    void GetCurrentTime(uint32_t &sec, uint32_t &nanosec);
    ranger_msgs::Quaternion CreateQuaternionFromYaw(float yaw);

  private:
    void *dora_context_;
    std::shared_ptr<RangerRobot> robot_;
    RangerSubType robot_type_;
    RobotParams robot_params_;
    std::mutex robot_mutex_;

    // Constants
    const float steer_angle_tolerance_ = 0.005; // ~+-0.287 degrees

    // Parameters
    std::string config_file_path_; // Path to YAML config file
    std::string robot_model_;
    std::string port_name_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_name_;
    int update_rate_;
    bool publish_odom_tf_;

    uint8_t motion_mode_ = 0;

    // Odometry variables
    std::chrono::steady_clock::time_point last_time_;
    std::chrono::steady_clock::time_point current_time_;
    float position_x_ = 0.0;
    float position_y_ = 0.0;
    float theta_ = 0.0;
    uint32_t odom_seq_ = 0;
  };

} // namespace westonrobot

#endif // RANGER_MESSENGER_HPP
