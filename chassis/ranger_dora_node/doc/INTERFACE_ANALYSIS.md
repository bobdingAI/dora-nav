# Ranger节点接口分析文档

## 1. ROS2节点接口分析

### 1.1 输入接口

| 接口名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cmd_vel` | `geometry_msgs::msg::Twist` | 速度命令，包含线速度(x,y)和角速度(z) |

**Twist消息结构**：
```cpp
geometry_msgs::msg::Twist {
    Vector3 linear;   // x: 前后, y: 左右, z: 上下
    Vector3 angular;  // x: roll, y: pitch, z: yaw
}
```

### 1.2 输出接口

| 接口名称 | 消息类型 | 频率 | 说明 |
|---------|---------|------|------|
| `/system_state` | `ranger_msgs::msg::SystemState` | 50Hz | 系统状态 |
| `/motion_state` | `ranger_msgs::msg::MotionState` | 50Hz | 运动模式 |
| `/actuator_state` | `ranger_msgs::msg::ActuatorStateArray` | 50Hz | 执行器状态 |
| `/odom` | `nav_msgs::msg::Odometry` | 50Hz | 里程计 |
| `/battery_state` | `sensor_msgs::msg::BatteryState` | 50Hz | 电池状态 |

**SystemState消息内容**：
- `vehicle_state`: 车辆状态
- `control_mode`: 控制模式
- `error_code`: 错误代码
- `battery_voltage`: 电池电压
- `motion_mode`: 运动模式

**ActuatorStateArray消息内容**：
- 8个执行器的状态数组
- 每个执行器包含：
  - `driver_state`: 驱动器状态（电压、温度、状态码）
  - `motor_state`: 电机状态（电流、脉冲计数、转速、角度、速度）

**Odometry消息内容**：
- `pose`: 位置和姿态（x, y, z, 四元数）
- `twist`: 线速度和角速度

**BatteryState消息内容**：
- `voltage`: 电压
- `current`: 电流
- `temperature`: 温度
- `percentage`: 电量百分比

### 1.3 参数

| 参数名 | 类型 | 默认值 | 说明 |
|-------|------|--------|------|
| `port_name` | string | "can0" | CAN接口名称 |
| `robot_model` | string | "ranger" | 机器人型号 |
| `odom_frame` | string | "odom" | 里程计坐标系 |
| `base_frame` | string | "base_link" | 基座坐标系 |
| `update_rate` | int | 50 | 更新频率(Hz) |
| `odom_topic_name` | string | "odom" | 里程计话题名 |
| `publish_odom_tf` | bool | false | 是否发布TF变换 |

## 2. ugv_sdk调用方式

### 2.1 初始化流程

```cpp
// 1. 创建机器人实例
robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);

// 2. 连接CAN总线
robot_->Connect(port_name_);

// 3. 启用命令模式
robot_->EnableCommandedMode();
```

### 2.2 状态读取

```cpp
// 获取机器人核心状态
auto state = robot_->GetRobotState();
// 返回: RangerCoreState
//   - system_state: 系统状态
//   - motion_state: 运动状态（速度、转向角）
//   - motion_mode_state: 运动模式

// 获取执行器状态
auto actuator_state = robot_->GetActuatorState();
// 返回: RangerActuatorState
//   - motor_angles: 电机角度
//   - motor_speeds: 电机速度
//   - actuator_ls_state: 低速状态（驱动器信息）
//   - actuator_hs_state: 高速状态（电机信息）

// 获取传感器状态
auto sensor_state = robot_->GetCommonSensorState();
// 返回: CommonSensorState
//   - bms_basic_state: BMS电池状态
```

### 2.3 命令发送

```cpp
// 设置运动模式
robot_->SetMotionMode(mode);
// mode可选值:
//   - MOTION_MODE_DUAL_ACKERMAN (0): 双阿克曼
//   - MOTION_MODE_PARALLEL (1): 平行模式
//   - MOTION_MODE_SPINNING (2): 旋转模式
//   - MOTION_MODE_SIDE_SLIP (3): 侧滑模式

// 发送运动命令（阿克曼/平行模式）
robot_->SetMotionCommand(linear_velocity, steering_angle);

// 发送运动命令（旋转模式）
robot_->SetMotionCommand(0.0, 0.0, angular_velocity);
```

### 2.4 运动模式自动切换逻辑

```cpp
if (linear_y != 0) {
    // 有侧向速度 -> 平行模式
    mode = MOTION_MODE_PARALLEL;
} else {
    // 计算转弯半径
    radius = |linear_x / angular_z|;

    if (radius < min_turn_radius) {
        // 小半径转弯 -> 旋转模式
        mode = MOTION_MODE_SPINNING;
    } else {
        // 大半径转弯 -> 阿克曼模式
        mode = MOTION_MODE_DUAL_ACKERMAN;
    }
}
```

## 3. DORA节点接口设计

### 3.1 输入接口

| 接口名称 | 数据格式 | 说明 |
|---------|---------|------|
| `CmdVelTwist` | JSON | 速度命令 |
| `tick` | Timer | 定时触发器（100Hz） |

**CmdVelTwist JSON格式**：
```json
{
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.2}
}
```

### 3.2 输出接口

| 接口名称 | 数据格式 | 频率 | 说明 |
|---------|---------|------|------|
| `Odometry` | JSON | 100Hz | 里程计数据 |
| `SystemState` | JSON | 100Hz | 系统状态 |
| `MotionState` | JSON | 100Hz | 运动模式 |
| `ActuatorState` | JSON | 100Hz | 执行器状态 |
| `BatteryState` | JSON | 100Hz | 电池状态 |

**Odometry JSON格式**：
```json
{
    "header": {
        "frame_id": "odom",
        "seq": 123,
        "stamp": {"sec": 1234567890, "nanosec": 123456789}
    },
    "child_frame_id": "base_link",
    "pose": {
        "pose": {
            "position": {"x": 1.0, "y": 2.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.1, "w": 0.995}
        }
    },
    "twist": {
        "twist": {
            "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.2}
        }
    }
}
```

### 3.3 环境变量配置

| 变量名 | 类型 | 默认值 | 说明 |
|-------|------|--------|------|
| `CAN_PORT` | string | "can0" | CAN接口名称 |
| `UPDATE_RATE` | int | 100 | 更新频率(Hz) |

## 4. 关键差异对比

| 特性 | ROS2节点 | DORA节点 |
|------|---------|---------|
| **通信框架** | ROS2 DDS | DORA |
| **数据格式** | ROS2消息（二进制） | JSON（文本） |
| **配置方式** | launch文件 + 参数 | dataflow.yml + 环境变量 |
| **更新频率** | 50 Hz | 100 Hz |
| **TF发布** | 可选 | 不支持 |
| **依赖** | ROS2生态系统 | DORA runtime |
| **里程计计算** | Boost ODE积分 | 简化欧拉积分 |
| **线程模型** | ROS2 executor | DORA事件循环 |

## 5. 实现细节

### 5.1 坐标系转换

**阿克曼转向角度转换**：
```cpp
// 内轮角 -> 中心角
double ConvertInnerAngleToCentral(double angle) {
    double r = wheelbase / tan(angle) + track / 2.0;
    return atan(wheelbase / r);
}

// 中心角 -> 内轮角
double ConvertCentralAngleToInner(double angle) {
    double r = wheelbase / tan(angle);
    return atan(wheelbase / (r - track / 2.0));
}
```

### 5.2 里程计更新

**ROS2版本**（使用Boost ODE）：
```cpp
boost::numeric::odeint::integrate_const(
    boost::numeric::odeint::runge_kutta4<state_type>(),
    DualAckermanModel(wheelbase, u), x, 0.0, dt, (dt / 10.0)
);
```

**DORA版本**（简化欧拉积分）：
```cpp
double delta_theta = angular_z * dt;
double delta_x = linear_x * cos(theta + delta_theta / 2.0) * dt;
double delta_y = linear_x * sin(theta + delta_theta / 2.0) * dt;
position_x += delta_x;
position_y += delta_y;
theta += delta_theta;
```

### 5.3 线程安全

DORA节点使用mutex保护robot_对象：
```cpp
std::lock_guard<std::mutex> lock(robot_mutex_);
auto state = robot_->GetRobotState();
```

## 6. 性能考虑

### 6.1 更新频率

- ROS2: 50 Hz（20ms周期）
- DORA: 100 Hz（10ms周期）

更高的频率提供更好的实时性，但增加CPU负载。

### 6.2 数据序列化

- ROS2: 使用CDR二进制序列化，效率高
- DORA: 使用JSON文本序列化，可读性好但效率较低

如需更高性能，可考虑使用Apache Arrow格式。

### 6.3 通信延迟

- ROS2 DDS: 通常1-5ms
- DORA: 通常<1ms（共享内存）

## 7. 扩展建议

### 7.1 添加诊断功能

```cpp
// 监控CAN通信状态
// 检测超时和错误
// 发布诊断信息
```

### 7.2 参数动态调整

```cpp
// 支持运行时调整速度限制
// 支持运行时切换运动模式
```

### 7.3 安全保护

```cpp
// 速度限制检查
// 急停功能
// 看门狗定时器
```

### 7.4 性能优化

```cpp
// 使用Apache Arrow替代JSON
// 减少内存拷贝
// 优化里程计计算
```

## 8. 测试建议

1. **单元测试**：测试各个函数的正确性
2. **集成测试**：测试与ugv_sdk的交互
3. **性能测试**：测试100Hz更新是否稳定
4. **压力测试**：长时间运行测试
5. **边界测试**：测试极限速度和转向角

## 9. 文档参考

- ugv_sdk文档：查看API详细说明
- DORA文档：了解dataflow配置
- Ranger用户手册：了解机器人参数和限制
