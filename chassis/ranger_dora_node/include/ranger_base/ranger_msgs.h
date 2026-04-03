/**
 * @file ranger_msgs.h
 * @date 2026-02-04
 * @brief Message type definitions for Ranger DORA node
 *
 * This file defines C++ structures that correspond to ROS2 message types
 * used in the ranger_ros2 package.
 */

#ifndef RANGER_MSGS_H
#define RANGER_MSGS_H

#include <cstdint>
#include <vector>
#include <string>

namespace ranger_msgs
{

    /**
     * @brief Header structure (equivalent to std_msgs/Header)
     */
    struct Header
    {
        uint32_t seq;
        struct
        {
            uint32_t sec;
            uint32_t nanosec;
        } stamp;
        std::string frame_id;
    };

    /**
     * @brief Vector3 structure (equivalent to geometry_msgs/Vector3)
     */
    struct Vector3
    {
        float x;
        float y;
        float z;
    };

    /**
     * @brief Quaternion structure (equivalent to geometry_msgs/Quaternion)
     */
    struct Quaternion
    {
        float x;
        float y;
        float z;
        float w;
    };

    /**
     * @brief Twist structure (equivalent to geometry_msgs/Twist)
     */
    struct Twist
    {
        Vector3 linear;
        Vector3 angular;
    };

    /**
     * @brief Point structure (equivalent to geometry_msgs/Point)
     */
    struct Point
    {
        float x;
        float y;
        float z;
    };

    /**
     * @brief Pose structure (equivalent to geometry_msgs/Pose)
     */
    struct Pose
    {
        Point position;
        Quaternion orientation;
    };

    /**
     * @brief PoseWithCovariance structure
     */
    struct PoseWithCovariance
    {
        Pose pose;
        float covariance[36]; // 6x6 covariance matrix
    };

    /**
     * @brief TwistWithCovariance structure
     */
    struct TwistWithCovariance
    {
        Twist twist;
        float covariance[36]; // 6x6 covariance matrix
    };

    /**
     * @brief Odometry structure (equivalent to nav_msgs/Odometry)
     */
    struct Odometry
    {
        Header header;
        std::string child_frame_id;
        PoseWithCovariance pose;
        TwistWithCovariance twist;
    };

    /**
     * @brief SystemState structure (equivalent to ranger_msgs/SystemState)
     */
    struct SystemState
    {
        Header header;
        uint8_t vehicle_state;
        uint8_t control_mode;
        uint16_t error_code;
        float battery_voltage;
        uint8_t motion_mode;
    };

    /**
     * @brief MotionState structure (equivalent to ranger_msgs/MotionState)
     */
    struct MotionState
    {
        Header header;
        uint8_t motion_mode;

        // Motion mode constants
        static constexpr uint8_t MOTION_MODE_DUAL_ACKERMAN = 0;
        static constexpr uint8_t MOTION_MODE_PARALLEL = 1;
        static constexpr uint8_t MOTION_MODE_SPINNING = 2;
        static constexpr uint8_t MOTION_MODE_SIDE_SLIP = 3;
    };

    /**
     * @brief DriverState structure (equivalent to ranger_msgs/DriverState)
     */
    struct DriverState
    {
        float driver_voltage;
        int8_t driver_temperature;
        int8_t motor_temperature;
        uint8_t driver_state;
    };

    /**
     * @brief MotorState structure (equivalent to ranger_msgs/MotorState)
     */
    struct MotorState
    {
        float current;
        int32_t pulse_count;
        float rpm;
        float motor_angles;
        float motor_speeds;
    };

    /**
     * @brief ActuatorState structure (equivalent to ranger_msgs/ActuatorState)
     */
    struct ActuatorState
    {
        uint8_t id;
        DriverState driver;
        MotorState motor;
    };

    /**
     * @brief ActuatorStateArray structure (equivalent to ranger_msgs/ActuatorStateArray)
     */
    struct ActuatorStateArray
    {
        Header header;
        std::vector<ActuatorState> states;
    };

    /**
     * @brief BatteryState structure (equivalent to sensor_msgs/BatteryState)
     */
    struct BatteryState
    {
        Header header;
        float voltage;                   // Voltage in Volts
        float temperature;               // Temperature in Celsius
        float current;                   // Current in Amperes (negative for discharge)
        float charge;                    // Current charge in Ah
        float capacity;                  // Capacity in Ah
        float design_capacity;           // Design capacity in Ah
        float percentage;                // Charge percentage (0-100)
        uint8_t power_supply_status;     // Power supply status
        uint8_t power_supply_health;     // Power supply health
        uint8_t power_supply_technology; // Power supply technology
        bool present;                    // Battery present

        // Power supply status constants
        static constexpr uint8_t POWER_SUPPLY_STATUS_UNKNOWN = 0;
        static constexpr uint8_t POWER_SUPPLY_STATUS_CHARGING = 1;
        static constexpr uint8_t POWER_SUPPLY_STATUS_DISCHARGING = 2;
        static constexpr uint8_t POWER_SUPPLY_STATUS_NOT_CHARGING = 3;
        static constexpr uint8_t POWER_SUPPLY_STATUS_FULL = 4;

        // Power supply health constants
        static constexpr uint8_t POWER_SUPPLY_HEALTH_UNKNOWN = 0;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_GOOD = 1;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERHEAT = 2;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_DEAD = 3;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_COLD = 6;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7;
        static constexpr uint8_t POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8;

        // Power supply technology constants
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NIMH = 1;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LION = 2;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIPO = 3;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIFE = 4;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NICD = 5;
        static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIMN = 6;
    };

} // namespace ranger_msgs

#endif // RANGER_MSGS_H
