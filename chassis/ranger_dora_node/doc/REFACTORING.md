# 代码重构说明

## 重构目标

按照用户要求，将DORA节点代码重构为与ROS2版本相同的目录结构和函数命名规范。

## 新的目录结构

```
ranger_dora_node/
├── include/
│   └── ranger_base/
│       ├── ranger_msgs.h           # 消息类型定义（结构体）
│       ├── ranger_params.hpp       # 机器人参数定义
│       └── ranger_messenger.hpp    # Messenger类头文件
├── src/
│   ├── ranger_base_node.cpp        # 主程序入口
│   ├── ranger_messenger.cpp        # Messenger类实现
│   └── ranger_dora_node.cc.backup  # 旧实现（备份）
├── CMakeLists.txt                   # 编译配置
├── ranger_miniv3_dataflow.yml       # DORA配置
└── ...
```

## 主要变更

### 1. 消息类型定义 (ranger_msgs.h)

创建了完整的消息结构体定义，对应ROS2消息类型：

- `Header` - 消息头
- `Vector3` - 三维向量
- `Quaternion` - 四元数
- `Twist` - 速度命令
- `Pose` - 位姿
- `Odometry` - 里程计
- `SystemState` - 系统状态
- `MotionState` - 运动模式
- `ActuatorState` / `ActuatorStateArray` - 执行器状态
- `BatteryState` - 电池状态

### 2. 类结构 (RangerDoraMessenger)

采用与ROS2版本相同的类结构：

```cpp
class RangerDoraMessenger {
 public:
  RangerDoraMessenger(void* dora_context);
  void Run();

 private:
  void LoadParameters();
  void SetupRobot();
  void PublishStateToROS();
  void TwistCmdCallback(const ranger_msgs::Twist& msg);
  void UpdateOdometry(double linear, double angular, double angle, double dt);
  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);
  // ...
};
```

### 3. 函数命名

保持与ROS2版本完全一致的函数命名：

| ROS2版本 | DORA版本 | 说明 |
|---------|---------|------|
| `LoadParameters()` | `LoadParameters()` | 加载参数 |
| `SetupSubscription()` | `SetupRobot()` | 设置机器人（DORA无订阅） |
| `PublishStateToROS()` | `PublishStateToROS()` | 发布状态 |
| `TwistCmdCallback()` | `TwistCmdCallback()` | 速度命令回调 |
| `UpdateOdometry()` | `UpdateOdometry()` | 更新里程计 |
| `ConvertInnerAngleToCentral()` | `ConvertInnerAngleToCentral()` | 角度转换 |
| `ConvertCentralAngleToInner()` | `ConvertCentralAngleToInner()` | 角度转换 |

### 4. 参数加载

使用环境变量替代ROS2参数：

```cpp
port_name_ = std::getenv("CAN_PORT") ? std::getenv("CAN_PORT") : "can0";
robot_model_ = std::getenv("ROBOT_MODEL") ? std::getenv("ROBOT_MODEL") : "ranger_mini_v3";
update_rate_ = std::getenv("UPDATE_RATE") ? std::stoi(std::getenv("UPDATE_RATE")) : 100;
```

### 5. 消息转换流程

```
ugv_sdk数据 -> ranger_msgs结构体 -> JSON -> DORA输出
DORA输入 -> JSON -> ranger_msgs结构体 -> ugv_sdk命令
```

## 与ROS2版本的对比

### 相同点

1. ✅ 目录结构相同
2. ✅ 类名和函数名相同
3. ✅ 消息类型定义对应
4. ✅ 运动控制逻辑相同
5. ✅ 里程计计算相同
6. ✅ 参数结构相同

### 差异点

| 特性 | ROS2版本 | DORA版本 |
|------|---------|---------|
| 通信框架 | ROS2 DDS | DORA |
| 参数加载 | ROS2参数服务器 | 环境变量 |
| 消息格式 | ROS2消息（二进制） | JSON（文本） |
| TF发布 | 支持 | 不支持 |
| 事件循环 | rclcpp::spin | dora_next_event |

## 编译和运行

### 编译

```bash
cd ranger_dora_node
./build.sh
```

### 运行

```bash
# 设置环境变量（可选）
export CAN_PORT=can0
export ROBOT_MODEL=ranger_mini_v3
export UPDATE_RATE=100

# 启动DORA
dora up
dora start ranger_miniv3_dataflow.yml
```

## 配置说明

### 环境变量

在`ranger_miniv3_dataflow.yml`中配置：

```yaml
envs:
  CAN_PORT: can0
  ROBOT_MODEL: ranger_mini_v3
  UPDATE_RATE: 100
  ODOM_FRAME: odom
  BASE_FRAME: base_link
```

### 支持的机器人型号

- `ranger` - 标准Ranger
- `ranger_mini_v1` - Ranger Mini V1
- `ranger_mini_v2` - Ranger Mini V2
- `ranger_mini_v3` - Ranger Mini V3（默认）

## 测试建议

1. **编译测试**：确保代码编译通过
2. **连接测试**：测试CAN总线连接
3. **命令测试**：发送速度命令，观察机器人响应
4. **状态测试**：检查所有输出数据是否正确
5. **模式切换测试**：测试4种运动模式的自动切换

## 已知问题

1. **执行器状态**：当前对8个执行器使用相同的数据，需要根据实际情况索引不同执行器
2. **里程计精度**：使用简化的欧拉积分，精度略低于ROS2版本的Boost ODE积分

## 后续改进

1. 添加Boost ODE积分支持，提高里程计精度
2. 完善执行器状态的索引逻辑
3. 添加更多的错误处理和诊断功能
4. 支持参数动态调整

## 文件对照表

| ROS2文件 | DORA文件 | 说明 |
|---------|---------|------|
| ranger_base/include/ranger_base/ranger_messenger.hpp | include/ranger_base/ranger_messenger.hpp | Messenger头文件 |
| ranger_base/include/ranger_base/ranger_params.hpp | include/ranger_base/ranger_params.hpp | 参数定义 |
| ranger_base/src/ranger_messenger.cpp | src/ranger_messenger.cpp | Messenger实现 |
| ranger_base/src/ranger_base_node.cpp | src/ranger_base_node.cpp | 主程序 |
| ranger_msgs/msg/*.msg | include/ranger_base/ranger_msgs.h | 消息定义 |
| ranger_base/launch/*.launch.py | ranger_miniv3_dataflow.yml | 启动配置 |

## 总结

重构后的代码完全遵循ROS2版本的结构和命名规范，使得：

1. 代码更易于理解和维护
2. 与ROS2版本的对比更清晰
3. 消息类型定义完整，避免类型缺失问题
4. 便于后续功能扩展和优化
