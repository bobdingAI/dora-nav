/**
 * @file ranger_messenger.cpp
 * @date 2026-02-04
 * @brief DORA messenger implementation for Ranger robot
 *
 * @copyright Copyright (c) 2021 AgileX Robotics
 * @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
 */

#include "ranger_base/ranger_messenger.hpp"

#include "ranger_base/kinematics_model.hpp"
// DORA includes
extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include <iostream>
#include <sys/time.h>
#include <cstring>
#include <limits>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace nlohmann;

namespace westonrobot
{

  ///////////////////////////////////////////////////////////////////////////////////
  RangerDoraMessenger::RangerDoraMessenger(void *dora_context)
      : dora_context_(dora_context)
  {

    LoadParameters();

    // Create robot instance based on type
    if (robot_type_ == RangerSubType::kRangerMiniV1)
    {
      robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
    }
    else if (robot_type_ == RangerSubType::kRangerMiniV2)
    {
      robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
    }
    else if (robot_type_ == RangerSubType::kRangerMiniV3)
    {
      robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
    }
    else
    {
      robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
    }

    // Connect to CAN bus
    if (port_name_.find("can") != std::string::npos)
    {
      if (!robot_->Connect(port_name_))
      {
        cerr << "Failed to connect to the CAN port: " << port_name_ << endl;
        throw std::runtime_error("Failed to connect to CAN port");
      }
      robot_->EnableCommandedMode();
      cout << "Connected to CAN port: " << port_name_ << endl;
      cout << "Commanded mode enabled" << endl;
    }
    else
    {
      cerr << "Invalid port name: " << port_name_ << endl;
      throw std::runtime_error("Invalid port name");
    }
  }

  RangerDoraMessenger::~RangerDoraMessenger()
  {
    if (robot_)
    {
      // Stop the robot
      robot_->SetMotionCommand(0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void RangerDoraMessenger::Run()
  {
    last_time_ = std::chrono::steady_clock::now();

    while (true)
    {
      void *event = dora_next_event(dora_context_);

      if (event == NULL)
      {
        cerr << "[ranger_dora_node] ERROR: unexpected end of event" << endl;
        return;
      }

      enum DoraEventType ty = read_dora_event_type(event);

      if (ty == DoraEventType_Input)
      {
        char *id;
        size_t id_len;
        read_dora_input_id(event, &id, &id_len);

        if (strncmp(id, "CmdVelTwist", 11) == 0)
        {
          // Process velocity command
          char *data;
          size_t data_len;
          read_dora_input_data(event, &data, &data_len);

          auto twist = FromJson(data, data_len);
          TwistCmdCallback(twist);
        }
      }
      else if (ty == DoraEventType_Stop)
      {
        cout << "[ranger_dora_node] received stop event" << endl;
        free_dora_event(event);
        break;
      }

      // Periodic state update
      // 放在这个会不会太快了
      PublishStateToROS();

      free_dora_event(event);
    }
  }

  void RangerDoraMessenger::LoadParameters()
  {
    // Get config file path from environment variable or use default
    config_file_path_ = std::getenv("RANGER_CONFIG_FILE")
                            ? std::getenv("RANGER_CONFIG_FILE")
                            : "config/ranger_config.yaml";

    cout << "Loading parameters from: " << config_file_path_ << endl;

    try
    {
      // Load YAML file
      YAML::Node config = YAML::LoadFile(config_file_path_);

      // Load parameters from YAML
      port_name_ = config["can_port"].as<std::string>();
      robot_model_ = config["robot_model"].as<std::string>();
      odom_frame_ = config["odom_frame"].as<std::string>();
      base_frame_ = config["base_frame"].as<std::string>();
      update_rate_ = config["update_rate"].as<int>();
      odom_topic_name_ = config["odom_topic_name"].as<std::string>();
      publish_odom_tf_ = config["publish_odom_tf"].as<bool>();

      cout << "Successfully loaded the following parameters:" << endl;
      cout << "  port_name: " << port_name_ << endl;
      cout << "  robot_model: " << robot_model_ << endl;
      cout << "  odom_frame: " << odom_frame_ << endl;
      cout << "  base_frame: " << base_frame_ << endl;
      cout << "  update_rate: " << update_rate_ << endl;
      cout << "  odom_topic_name: " << odom_topic_name_ << endl;
    }
    catch (const YAML::Exception &e)
    {
      cerr << "Error loading YAML config file: " << e.what() << endl;
      cerr << "Using default parameters..." << endl;

      // Use default values if YAML loading fails
      port_name_ = "can0";
      robot_model_ = "ranger_mini_v3";
      odom_frame_ = "odom";
      base_frame_ = "base_link";
      update_rate_ = 100;
      odom_topic_name_ = "odom";
      publish_odom_tf_ = false;
    }

    // Load robot parameters based on robot model
    if (robot_model_ == "ranger_mini_v1")
    {
      robot_type_ = RangerSubType::kRangerMiniV1;

      robot_params_.track = RangerMiniV1Params::track;
      robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
      robot_params_.max_steer_angle_central = RangerMiniV1Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel = RangerMiniV1Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann = RangerMiniV1Params::max_steer_angle_ackermann;
    }
    else if (robot_model_ == "ranger_mini_v2")
    {
      robot_type_ = RangerSubType::kRangerMiniV2;

      robot_params_.track = RangerMiniV2Params::track;
      robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
      robot_params_.max_steer_angle_central = RangerMiniV2Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel = RangerMiniV2Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann = RangerMiniV2Params::max_steer_angle_ackermann;
    }
    else if (robot_model_ == "ranger_mini_v3")
    {
      robot_type_ = RangerSubType::kRangerMiniV3;

      robot_params_.track = RangerMiniV3Params::track;
      robot_params_.wheelbase = RangerMiniV3Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV3Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV3Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV3Params::max_speed_cmd;
      robot_params_.max_steer_angle_central = RangerMiniV3Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel = RangerMiniV3Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV3Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV3Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann = RangerMiniV3Params::max_steer_angle_ackermann;
    }
    else
    {
      robot_type_ = RangerSubType::kRanger;
      robot_params_.track = RangerParams::track;
      robot_params_.wheelbase = RangerParams::wheelbase;
      robot_params_.max_linear_speed = RangerParams::max_linear_speed;
      robot_params_.max_angular_speed = RangerParams::max_angular_speed;
      robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
      robot_params_.max_steer_angle_central = RangerParams::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel = RangerParams::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerParams::max_round_angle;
      robot_params_.min_turn_radius = RangerParams::min_turn_radius;
      robot_params_.max_steer_angle_ackermann = RangerParams::max_steer_angle_ackermann;
    }
  }

  void RangerDoraMessenger::PublishStateToROS()
  {
    current_time_ = std::chrono::steady_clock::now();
    static bool init_run = true;
    if (init_run)
    {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
    std::lock_guard<std::mutex> lock(robot_mutex_);

    auto state = robot_->GetRobotState();
    auto actuator_state = robot_->GetActuatorState();
    auto sensor_state = robot_->GetCommonSensorState();

    // Update odometry
    {
      float dt = std::chrono::duration<float>(current_time_ - last_time_).count();
      UpdateOdometry(state.motion_state.linear_velocity,
                     state.motion_state.angular_velocity,
                     state.motion_state.steering_angle, dt);
      last_time_ = current_time_;
    }

    // Publish system state
    {
      auto system_msg = CreateSystemStateMsg(state);
      SendOutput("SystemState", ToJson(system_msg));
    }

    // Publish motion state
    {
      motion_mode_ = state.motion_mode_state.motion_mode;
      auto motion_msg = CreateMotionStateMsg(state);
      SendOutput("MotionState", ToJson(motion_msg));
    }

    // Publish actuator state
    {
      auto actuator_msg = CreateActuatorStateMsg(actuator_state);
      SendOutput("ActuatorState", ToJson(actuator_msg));
    }

    // Publish odometry
    {
      float linear_x = state.motion_state.linear_velocity;
      float linear_y = 0.0;
      float angular_z = state.motion_state.angular_velocity;

      // Adjust velocities based on motion mode
      if (motion_mode_ == ranger_msgs::MotionState::MOTION_MODE_PARALLEL)
      {
        float phi = state.motion_state.steering_angle;
        linear_x = state.motion_state.linear_velocity * std::cos(phi);
        linear_y = state.motion_state.linear_velocity * std::sin(phi);
        angular_z = 0.0;
      }
      else if (motion_mode_ == ranger_msgs::MotionState::MOTION_MODE_SIDE_SLIP)
      {
        linear_x = 0.0;
        linear_y = state.motion_state.linear_velocity;
        angular_z = 0.0;
      }

      auto odom_msg = CreateOdometryMsg(linear_x, linear_y, angular_z);
      SendOutput("Odometry", ToJson(odom_msg));
    }

    // Publish battery state
    {
      auto battery_msg = CreateBatteryStateMsg(sensor_state);
      SendOutput("BatteryState", ToJson(battery_msg));
    }
  }

  void RangerDoraMessenger::UpdateOdometry(float linear, float angular,
                                           float angle, float dt)
  {
    // Update odometry calculations
    if (motion_mode_ == ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN)
    {
      // float delta_theta = angular * dt;
      // float delta_x = linear * std::cos(theta_ + delta_theta / 2.0) * dt;
      // float delta_y = linear * std::sin(theta_ + delta_theta / 2.0) * dt;

      // position_x_ += delta_x;
      // position_y_ += delta_y;
      // theta_ += delta_theta;
      // 原版代码
      DualAckermanModel::state_type x = {(double)position_x_, (double)position_y_, (double)theta_};
      DualAckermanModel::control_type u;
      u.v = (double)linear;
      u.phi = ConvertInnerAngleToCentral((double)angle);

      boost::numeric::odeint::integrate_const(
          boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
          DualAckermanModel((double)robot_params_.wheelbase, u), x, 0.0, (double)dt, (dt / 10.0));
      // std::cout<<" steer: "<<angle<<" central: "<<u.phi<<std::endl;
      position_x_ = x[0];
      position_y_ = x[1];
      theta_ = x[2];
    }
    else if (motion_mode_ == ranger_msgs::MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == ranger_msgs::MotionState::MOTION_MODE_SIDE_SLIP)
    {
      ParallelModel::state_type x = {(double)position_x_, (double)position_y_, (double)theta_};
      ParallelModel::control_type u;
      u.v = (double)linear;
      if (motion_mode_ == ranger_msgs::MotionState::MOTION_MODE_SIDE_SLIP)
      {
        u.phi = M_PI / 2.0;
      }
      else
      {
        u.phi = (double)angle;
      }
      boost::numeric::odeint::integrate_const(
          boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
          ParallelModel(u), x, 0.0, (double)dt, ((double)dt / 10.0));

      position_x_ = x[0];
      position_y_ = x[1];
      theta_ = x[2];
    }
    else if (motion_mode_ == ranger_msgs::MotionState::MOTION_MODE_SPINNING)
    {
      SpinningModel::state_type x = {(double)position_x_, (double)position_y_, (double)theta_};
      SpinningModel::control_type u;
      u.w = (double)angular;

      boost::numeric::odeint::integrate_const(
          boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
          SpinningModel(u), x, 0.0, (double)dt, ((double)dt / 10.0));

      position_x_ = x[0];
      position_y_ = x[1];
      theta_ = x[2];
    }

    // Normalize theta to [-pi, pi]
    while (theta_ > M_PI)
      theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI)
      theta_ += 2.0 * M_PI;
  }

  void RangerDoraMessenger::TwistCmdCallback(const ranger_msgs::Twist &msg)
  {
    std::lock_guard<std::mutex> lock(robot_mutex_);

    float steer_cmd;
    float radius;
    // analyze Twist msg and switch motion_mode
    // check for parking mode, only applicable to RangerMiniV2
    // if (parking_mode_ && robot_type_ == RangerSubType::kRangerMiniV2)
    // {
    //   return;
    // }
    // else if (msg.linear.y != 0)
    if (msg.linear.y != 0)
    {
      if (msg.linear.x == 0.0 && robot_type_ == RangerSubType::kRangerMiniV1)
      {
        motion_mode_ = ranger_msgs::MotionState::MOTION_MODE_SIDE_SLIP;
        robot_->SetMotionMode(ranger_msgs::MotionState::MOTION_MODE_SIDE_SLIP);
      }
      else
      {
        motion_mode_ = ranger_msgs::MotionState::MOTION_MODE_PARALLEL;
        robot_->SetMotionMode(ranger_msgs::MotionState::MOTION_MODE_PARALLEL);
      }
    }
    else
    {
      steer_cmd = CalculateSteeringAngle(msg, radius);
      // Use minimum turn radius to switch between dual ackerman and spinning mode
      if (radius < robot_params_.min_turn_radius)
      {
        // cout << " spinning mode radius: " << radius << "   x: " << msg.linear.x << "   z: " << msg.angular.z << std::endl;
        motion_mode_ = ranger_msgs::MotionState::MOTION_MODE_SPINNING;
        robot_->SetMotionMode(ranger_msgs::MotionState::MOTION_MODE_SPINNING);
      }
      else
      {
        // cout << " dual ackerman mode radius: " << radius << "   x: " << msg.linear.x << "   z: " << msg.angular.z << std::endl;
        motion_mode_ = ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN;
        robot_->SetMotionMode(ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN);
      }
    }

    // Send motion command to robot
    switch (motion_mode_)
    {
    case ranger_msgs::MotionState::MOTION_MODE_DUAL_ACKERMAN:
    {
      if (steer_cmd > robot_params_.max_steer_angle_ackermann)
      {
        steer_cmd = robot_params_.max_steer_angle_ackermann;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_ackermann)
      {
        steer_cmd = -robot_params_.max_steer_angle_ackermann;
      }
      robot_->SetMotionCommand(msg.linear.x, steer_cmd);
      break;
    }
    case ranger_msgs::MotionState::MOTION_MODE_PARALLEL:
    {
      steer_cmd = atan(msg.linear.y / msg.linear.x);

      static double last_nonzero_x = 1.0;

      if (msg.linear.x != 0.0)
      {
        last_nonzero_x = msg.linear.x;
      }

      if (std::signbit(msg.linear.x))
      {
        steer_cmd = -steer_cmd;
      }

      if (steer_cmd > robot_params_.max_steer_angle_parallel)
      {
        steer_cmd = robot_params_.max_steer_angle_parallel;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_parallel)
      {
        steer_cmd = -robot_params_.max_steer_angle_parallel;
      }
      double vel = 1.0;

      if (msg.linear.x == 0.0 && msg.linear.y != 0.0)
      {
        // std::cout << "MOTION_MODE_SIDE_SLIP" << std::endl;

        if (std::signbit(last_nonzero_x))
        {
          steer_cmd = -std::abs(steer_cmd);
        }
        else
        {
          steer_cmd = std::abs(steer_cmd);
        }
        vel = msg.linear.y >= 0 ? 1.0 : -1.0;
      }
      else
      {
        vel = msg.linear.x >= 0 ? 1.0 : -1.0;
      }
      robot_->SetMotionCommand(vel * sqrt(msg.linear.x * msg.linear.x +
                                          msg.linear.y * msg.linear.y),
                               steer_cmd);
      break;
    }
    case ranger_msgs::MotionState::MOTION_MODE_SPINNING:
    {
      double a_v = msg.angular.z;
      if (a_v > robot_params_.max_angular_speed)
      {
        a_v = robot_params_.max_angular_speed;
      }
      if (a_v < -robot_params_.max_angular_speed)
      {
        a_v = -robot_params_.max_angular_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, a_v);
      break;
    }
    case ranger_msgs::MotionState::MOTION_MODE_SIDE_SLIP:
    {
      double l_v = msg.linear.y;
      if (l_v > robot_params_.max_linear_speed)
      {
        l_v = robot_params_.max_linear_speed;
      }
      if (l_v < -robot_params_.max_linear_speed)
      {
        l_v = -robot_params_.max_linear_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, l_v);
      break;
    }
    }
  }

  float RangerDoraMessenger::CalculateSteeringAngle(const ranger_msgs::Twist &msg,
                                                    float &radius)
  {
    double linear = std::abs(msg.linear.x);
    double angular = std::abs(msg.angular.z);

    if (angular < 1e-6)
    {
      radius = std::numeric_limits<double>::infinity();
      return 0.0;
    }
    // Circular motion
    radius = linear / angular;
    int k = (msg.angular.z * msg.linear.x) >= 0 ? 1 : -1;

    double l, w, phi_i, x;
    l = robot_params_.wheelbase;
    w = robot_params_.track;
    x = sqrt(radius * radius + (l / 2) * (l / 2));
    // phi_i = atan((l / 2) / (x - w / 2));
    phi_i = atan((l / 2) / radius);

    const double max_phi_rad = 40.0 * M_PI / 180.0;
    phi_i = std::min(phi_i, max_phi_rad);

    return k * phi_i;
  }

  float RangerDoraMessenger::ConvertInnerAngleToCentral(float angle)
  {
    double phi = 0;
    double phi_i = std::abs(angle);

    phi = std::atan(robot_params_.wheelbase * std::sin(phi_i) /
                    (robot_params_.wheelbase * std::cos(phi_i) +
                     robot_params_.track * std::sin(phi_i)));

    phi *= angle >= 0 ? 1.0 : -1.0;
    return phi;
  }

  float RangerDoraMessenger::ConvertCentralAngleToInner(float angle)
  {
    double phi = std::abs(angle);
    double phi_i = 0;

    phi_i = std::atan(robot_params_.wheelbase * std::sin(phi) /
                      (robot_params_.wheelbase * std::cos(phi) -
                       robot_params_.track * std::sin(phi)));
    phi_i *= angle >= 0 ? 1.0 : -1.0;
    return phi_i;
  }

  // Message creation functions
  ranger_msgs::Odometry RangerDoraMessenger::CreateOdometryMsg(
      float linear_x, float linear_y, float angular_z)
  {
    ranger_msgs::Odometry odom_msg;

    uint32_t sec, nanosec;
    GetCurrentTime(sec, nanosec);

    odom_msg.header.seq = odom_seq_++;
    odom_msg.header.stamp.sec = sec;
    odom_msg.header.stamp.nanosec = nanosec;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = CreateQuaternionFromYaw(theta_);

    odom_msg.twist.twist.linear.x = linear_x;
    odom_msg.twist.twist.linear.y = linear_y;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_z;

    return odom_msg;
  }

  ranger_msgs::SystemState RangerDoraMessenger::CreateSystemStateMsg(
      const RangerCoreState &state)
  {
    ranger_msgs::SystemState system_msg;

    uint32_t sec, nanosec;
    GetCurrentTime(sec, nanosec);

    system_msg.header.seq = 0;
    system_msg.header.stamp.sec = sec;
    system_msg.header.stamp.nanosec = nanosec;

    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;
    system_msg.motion_mode = state.motion_mode_state.motion_mode;

    return system_msg;
  }

  ranger_msgs::MotionState RangerDoraMessenger::CreateMotionStateMsg(
      const RangerCoreState &state)
  {
    ranger_msgs::MotionState motion_msg;

    uint32_t sec, nanosec;
    GetCurrentTime(sec, nanosec);

    motion_msg.header.seq = 0;
    motion_msg.header.stamp.sec = sec;
    motion_msg.header.stamp.nanosec = nanosec;
    motion_msg.motion_mode = state.motion_mode_state.motion_mode;

    return motion_msg;
  }

  ranger_msgs::ActuatorStateArray RangerDoraMessenger::CreateActuatorStateMsg(
      const RangerActuatorState &actuator_state)
  {
    ranger_msgs::ActuatorStateArray actuator_msg;

    uint32_t sec, nanosec;
    GetCurrentTime(sec, nanosec);

    actuator_msg.header.seq = 0;
    actuator_msg.header.stamp.sec = sec;
    actuator_msg.header.stamp.nanosec = nanosec;

    for (int i = 0; i < 8; i++)
    {
      ranger_msgs::ActuatorState actuator;
      actuator.id = i;

      actuator.driver.driver_voltage = actuator_state.actuator_ls_state->driver_voltage;
      actuator.driver.driver_temperature = actuator_state.actuator_ls_state->driver_temp;
      actuator.driver.motor_temperature = actuator_state.actuator_ls_state->motor_temp;
      actuator.driver.driver_state = actuator_state.actuator_ls_state->driver_state;

      actuator.motor.current = actuator_state.actuator_hs_state->current;
      actuator.motor.pulse_count = actuator_state.actuator_hs_state->pulse_count;
      actuator.motor.rpm = actuator_state.actuator_hs_state->rpm;
      actuator.motor.motor_angles = actuator_state.motor_angles.angle_5;
      actuator.motor.motor_speeds = actuator_state.motor_speeds.speed_1;

      actuator_msg.states.push_back(actuator);
    }

    return actuator_msg;
  }

  ranger_msgs::BatteryState RangerDoraMessenger::CreateBatteryStateMsg(
      const RangerCommonSensorState &sensor_state)
  {
    ranger_msgs::BatteryState batt_msg;

    uint32_t sec, nanosec;
    GetCurrentTime(sec, nanosec);

    batt_msg.header.seq = 0;
    batt_msg.header.stamp.sec = sec;
    batt_msg.header.stamp.nanosec = nanosec;

    batt_msg.voltage = sensor_state.bms_basic_state.voltage;
    batt_msg.temperature = sensor_state.bms_basic_state.temperature;
    batt_msg.current = sensor_state.bms_basic_state.current;
    batt_msg.percentage = sensor_state.bms_basic_state.battery_soc;
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status = ranger_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health = ranger_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology = ranger_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = true;

    return batt_msg;
  }

  // JSON conversion functions
  json RangerDoraMessenger::ToJson(const ranger_msgs::Odometry &msg)
  {
    json j;
    j["header"]["frame_id"] = msg.header.frame_id;
    j["header"]["seq"] = msg.header.seq;
    j["header"]["stamp"]["sec"] = msg.header.stamp.sec;
    j["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;

    j["child_frame_id"] = msg.child_frame_id;

    j["pose"]["pose"]["position"]["x"] = msg.pose.pose.position.x;
    j["pose"]["pose"]["position"]["y"] = msg.pose.pose.position.y;
    j["pose"]["pose"]["position"]["z"] = msg.pose.pose.position.z;

    j["pose"]["pose"]["orientation"]["x"] = msg.pose.pose.orientation.x;
    j["pose"]["pose"]["orientation"]["y"] = msg.pose.pose.orientation.y;
    j["pose"]["pose"]["orientation"]["z"] = msg.pose.pose.orientation.z;
    j["pose"]["pose"]["orientation"]["w"] = msg.pose.pose.orientation.w;

    j["twist"]["twist"]["linear"]["x"] = msg.twist.twist.linear.x;
    j["twist"]["twist"]["linear"]["y"] = msg.twist.twist.linear.y;
    j["twist"]["twist"]["linear"]["z"] = msg.twist.twist.linear.z;

    j["twist"]["twist"]["angular"]["x"] = msg.twist.twist.angular.x;
    j["twist"]["twist"]["angular"]["y"] = msg.twist.twist.angular.y;
    j["twist"]["twist"]["angular"]["z"] = msg.twist.twist.angular.z;

    return j;
  }

  json RangerDoraMessenger::ToJson(const ranger_msgs::SystemState &msg)
  {
    json j;
    j["header"]["stamp"]["sec"] = msg.header.stamp.sec;
    j["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;

    j["vehicle_state"] = msg.vehicle_state;
    j["control_mode"] = msg.control_mode;
    j["error_code"] = msg.error_code;
    j["battery_voltage"] = msg.battery_voltage;
    j["motion_mode"] = msg.motion_mode;

    return j;
  }

  json RangerDoraMessenger::ToJson(const ranger_msgs::MotionState &msg)
  {
    json j;
    j["header"]["stamp"]["sec"] = msg.header.stamp.sec;
    j["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
    j["motion_mode"] = msg.motion_mode;

    return j;
  }

  json RangerDoraMessenger::ToJson(const ranger_msgs::ActuatorStateArray &msg)
  {
    json j;
    j["header"]["stamp"]["sec"] = msg.header.stamp.sec;
    j["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;

    json states = json::array();
    for (const auto &actuator : msg.states)
    {
      json act;
      act["id"] = actuator.id;

      act["driver"]["driver_voltage"] = actuator.driver.driver_voltage;
      act["driver"]["driver_temperature"] = actuator.driver.driver_temperature;
      act["driver"]["motor_temperature"] = actuator.driver.motor_temperature;
      act["driver"]["driver_state"] = actuator.driver.driver_state;

      act["motor"]["current"] = actuator.motor.current;
      act["motor"]["pulse_count"] = actuator.motor.pulse_count;
      act["motor"]["rpm"] = actuator.motor.rpm;
      act["motor"]["motor_angle"] = actuator.motor.motor_angles;
      act["motor"]["motor_speed"] = actuator.motor.motor_speeds;

      states.push_back(act);
    }

    j["states"] = states;
    return j;
  }

  json RangerDoraMessenger::ToJson(const ranger_msgs::BatteryState &msg)
  {
    json j;
    j["header"]["stamp"]["sec"] = msg.header.stamp.sec;
    j["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;

    j["voltage"] = msg.voltage;
    j["temperature"] = msg.temperature;
    j["current"] = msg.current;
    j["percentage"] = msg.percentage;

    return j;
  }

  ranger_msgs::Twist RangerDoraMessenger::FromJson(const char *data, size_t data_len)
  {
    ranger_msgs::Twist twist;

    try
    {
      json j = json::parse(string(data, data_len));

      twist.linear.x = j["linear"]["x"].get<float>();
      twist.linear.y = j["linear"]["y"].get<float>();
      twist.linear.z = j["linear"]["z"].get<float>();

      twist.angular.x = j["angular"]["x"].get<float>();
      twist.angular.y = j["angular"]["y"].get<float>();
      twist.angular.z = j["angular"]["z"].get<float>();

      cout << "Received cmd_vel: linear_x=" << twist.linear.x
           << ", linear_y=" << twist.linear.y
           << ", angular_z=" << twist.angular.z << endl;
    }
    catch (const exception &e)
    {
      cerr << "Error parsing cmd_vel JSON: " << e.what() << endl;
      twist.linear.x = 0.0;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 0.0;
    }

    return twist;
  }

  // DORA communication
  int RangerDoraMessenger::SendOutput(const std::string &output_id, const json &j_data)
  {
    string json_string = j_data.dump();
    int result = dora_send_output(dora_context_, (char *)output_id.c_str(), output_id.length(),
                                  (char *)json_string.c_str(), json_string.length());
    if (result != 0)
    {
      cerr << "Failed to send output: " << output_id << endl;
    }
    return result;
  }

  // Helper functions
  void RangerDoraMessenger::GetCurrentTime(uint32_t &sec, uint32_t &nanosec)
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    sec = tv.tv_sec;
    nanosec = tv.tv_usec * 1000;
  }

  ranger_msgs::Quaternion RangerDoraMessenger::CreateQuaternionFromYaw(float yaw)
  {
    ranger_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
  }

} // namespace westonrobot
