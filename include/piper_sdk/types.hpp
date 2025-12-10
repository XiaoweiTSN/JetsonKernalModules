/**
 * @file types.hpp
 * @brief Core type definitions for Piper SDK
 *        Piper SDK 核心类型定义
 * 
 * @details This file contains all fundamental data structures used throughout the SDK,
 *          including motion control types, feedback structures, and configuration parameters.
 *          本文件包含SDK中使用的所有基础数据结构，包括运动控制类型、反馈结构和配置参数。
 * 
 * @author Wesley Cui 崔笑唯
 * @copyright Copyright (c) 2025 TNCA
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace piper_sdk {

/**
 * @enum MotionMode
 * @brief Motion control mode enumeration
 *        运动控制模式枚举
 */
enum class MotionMode {
  kUnknown,  ///< Unknown mode | 未知模式
  kJoint,    ///< Joint space motion | 关节空间运动
  kLinear,   ///< Linear Cartesian motion | 笛卡尔空间直线运动
  kCircular, ///< Circular trajectory motion | 圆弧轨迹运动
  kPoint,    ///< Point-to-point motion | 点到点运动
  kMit       ///< MIT motor control mode | MIT电机控制模式
};

/**
 * @struct CartesianPose
 * @brief Cartesian space pose representation
 *        笛卡尔空间位姿表示
 * 
 * @details Represents 6-DOF pose with position in micrometers (0.001 mm)
 *          and orientation in millidegrees (0.001 deg).
 *          表示六自由度位姿，位置单位为微米（0.001 mm），
 *          姿态单位为毫度（0.001 deg）。
 */
struct CartesianPose {
  std::array<int32_t, 3> position_um{};      ///< Position [x, y, z] in 0.001 mm | 位置 [x, y, z]，单位 0.001 mm
  std::array<int32_t, 3> orientation_mdeg{}; ///< Orientation [roll, pitch, yaw] in 0.001 deg | 姿态 [roll, pitch, yaw]，单位 0.001 deg
};

/**
 * @struct LinkPose
 * @brief Individual link pose in floating-point representation
 *        单个连杆位姿的浮点表示
 * 
 * @details Used for forward kinematics output, matching Python SDK GetFK() format.
 *          用于正运动学输出，与 Python SDK 的 GetFK() 格式匹配。
 */
struct LinkPose {
  std::array<double, 3> position_mm{};    ///< Position [x, y, z] in mm | 位置 [x, y, z]，单位 mm
  std::array<double, 3> orientation_deg{}; ///< Orientation [roll, pitch, yaw] in degrees | 姿态 [roll, pitch, yaw]，单位度
};

/**
 * @enum FKMode
 * @brief Forward kinematics calculation mode
 *        正运动学计算模式
 */
enum class FKMode { 
  kFeedback, ///< Calculate FK from feedback joint positions | 从反馈关节位置计算正运动学
  kControl   ///< Calculate FK from control command positions | 从控制命令位置计算正运动学
};

/**
 * @struct JointState
 * @brief Joint space state information
 *        关节空间状态信息
 * 
 * @note Joint velocity is NOT included here to avoid unit confusion.
 *       Use ArmFeedback.end_velocity_accel_feedback for end-effector velocity data.
 *       此处不包含关节速度以避免单位混淆。
 *       使用 ArmFeedback.end_velocity_accel_feedback 获取末端执行器速度数据。
 */
struct JointState {
  std::vector<int32_t> position_mdeg;  ///< Joint positions in 0.001 deg | 关节位置，单位 0.001 deg
  std::vector<int32_t> effort_mNm;     ///< Joint torques in 0.001 N·m | 关节力矩，单位 0.001 N·m
};

/**
 * @struct LowSpeedDriverStatus
 * @brief Low-speed driver status information
 *        低速驱动器状态信息
 * 
 * @details Contains electrical parameters and status flags for motor drivers.
 *          包含电机驱动器的电气参数和状态标志。
 */
struct LowSpeedDriverStatus {
  uint16_t bus_voltage_decivolts{0};  ///< Bus voltage in 0.1 V | 母线电压，单位 0.1 V
  int16_t foc_temperature_c{0};       ///< FOC controller temperature in °C | FOC控制器温度，单位 °C
  int8_t motor_temperature_c{0};      ///< Motor temperature in °C | 电机温度，单位 °C
  uint8_t status_code{0};             ///< Status code byte | 状态码字节
  uint16_t bus_current_ma{0};         ///< Bus current in mA | 母线电流，单位 mA

  /**
   * @struct FocFlags
   * @brief FOC controller status flags
   *        FOC控制器状态标志
   */
  struct FocFlags {
    bool voltage_too_low{false};        ///< Bus voltage too low | 母线电压过低
    bool motor_overheating{false};      ///< Motor overheating | 电机过热
    bool driver_overcurrent{false};     ///< Driver overcurrent protection | 驱动器过流保护
    bool driver_overheating{false};     ///< Driver overheating | 驱动器过热
    bool collision_status{false};       ///< Collision detected | 检测到碰撞
    bool driver_error_status{false};    ///< Driver error occurred | 驱动器错误
    bool driver_enable_status{false};   ///< Driver enabled | 驱动器已使能
    bool stall_status{false};           ///< Motor stall detected | 检测到电机堵转
  } flags;
};

/**
 * @struct ArmFeedback
 * @brief Comprehensive arm feedback state
 *        机械臂综合反馈状态
 * 
 * @details Contains all feedback data from the robot arm including joint states,
 *          end-effector pose, motor states, and driver status information.
 *          包含来自机械臂的所有反馈数据，包括关节状态、末端执行器位姿、
 *          电机状态和驱动器状态信息。
 */
struct ArmFeedback {
  std::chrono::steady_clock::time_point timestamp{std::chrono::steady_clock::now()}; ///< Feedback timestamp | 反馈时间戳
  JointState joints;                                    ///< Joint state data | 关节状态数据
  std::optional<CartesianPose> end_effector;           ///< End-effector pose | 末端执行器位姿
  
  /**
   * @brief End-effector velocity/acceleration feedback from CAN IDs 0x481-0x486 (per joint)
   *        末端执行器速度/加速度反馈，来自 CAN ID 0x481-0x486（每个关节）
   * @details [0]: linear velocity (0.001 m/s) | 线速度（0.001 m/s）
   *          [1]: angular velocity (0.001 rad/s) | 角速度（0.001 rad/s）
   *          [2]: linear acceleration (0.001 m/s²) | 线加速度（0.001 m/s²）
   *          [3]: angular acceleration (0.001 rad/s²) | 角加速度（0.001 rad/s²）
   */
  std::vector<std::array<int32_t, 4>> end_velocity_accel_feedback;
  
  std::vector<int16_t> motor_speed_mrad_per_s;  ///< Motor speeds in 0.001 rad/s (NOT rpm!) | 电机速度，单位 0.001 rad/s（不是 rpm！）
  std::vector<int16_t> motor_current_ma;        ///< Motor currents in mA | 电机电流，单位 mA
  std::vector<int32_t> motor_position_rad;      ///< Motor positions in rad | 电机位置，单位 rad
  std::vector<int32_t> motor_effort_mNm;        ///< Motor torques in 0.001 N·m (calculated from current) | 电机力矩，单位 0.001 N·m（从电流计算）
  std::vector<LowSpeedDriverStatus> low_speed_driver_info; ///< Driver status per joint | 每个关节的驱动器状态
  std::optional<std::array<int32_t, 2>> gripper_raw;  ///< Gripper state [position in 0.001 mm, effort in 0.001 N·m] | 夹爪状态 [位置(0.001 mm), 力矩(0.001 N·m)]
  std::optional<std::array<uint8_t, 6>> collision_levels; ///< Collision protection levels per joint | 每个关节的碰撞保护等级
  
  uint8_t control_mode{0};     ///< Current control mode | 当前控制模式
  uint8_t arm_status{0};       ///< Arm status byte | 机械臂状态字节
  uint8_t mode_feedback{0};    ///< Mode feedback byte | 模式反馈字节
  uint8_t teach_status{0};     ///< Teaching mode status | 示教模式状态
  uint8_t motion_status{0};    ///< Motion execution status | 运动执行状态
  uint8_t trajectory_num{0};   ///< Current trajectory number | 当前轨迹编号
  int16_t error_code{0};       ///< Error code (0 = no error) | 错误代码（0 = 无错误）
  bool enabled{false};         ///< Arm enabled state | 机械臂使能状态
  MotionMode mode{MotionMode::kUnknown}; ///< Active motion mode | 活动运动模式
};

/**
 * @struct InstructionResponse
 * @brief Response to command instructions
 *        指令响应
 * 
 * @details Represents the controller's response to configuration or query commands.
 *          表示控制器对配置或查询命令的响应。
 */
struct InstructionResponse {
  uint8_t code{0};                ///< Response code | 响应代码
  std::array<uint8_t, 7> payload{}; ///< Response payload data | 响应载荷数据
};

/**
 * @struct TimedJointState
 * @brief Joint state with timestamp and frequency
 *        带时间戳和频率的关节状态
 */
struct TimedJointState {
  JointState state;                                      ///< Joint state data | 关节状态数据
  std::chrono::steady_clock::time_point timestamp{};    ///< Update timestamp | 更新时间戳
  double hz{0.0};                                       ///< Update frequency in Hz | 更新频率，单位 Hz
};

/**
 * @struct TimedPoseState
 * @brief Cartesian pose with timestamp and frequency
 *        带时间戳和频率的笛卡尔位姿
 */
struct TimedPoseState {
  CartesianPose pose{};                                 ///< Cartesian pose | 笛卡尔位姿
  std::chrono::steady_clock::time_point timestamp{};   ///< Update timestamp | 更新时间戳
  double hz{0.0};                                      ///< Update frequency in Hz | 更新频率，单位 Hz
};

/**
 * @struct TimedGripperState
 * @brief Gripper state with timestamp and frequency
 *        带时间戳和频率的夹爪状态
 */
struct TimedGripperState {
  std::array<int32_t, 2> state{0, 0};  ///< [position in 0.001 mm, effort in 0.001 N·m] | [位置(0.001 mm), 力矩(0.001 N·m)]
  std::chrono::steady_clock::time_point timestamp{}; ///< Update timestamp | 更新时间戳
  double hz{0.0};                                    ///< Update frequency in Hz | 更新频率，单位 Hz
};

/**
 * @struct TimedMotorState
 * @brief Motor state with timestamp and frequency
 *        带时间戳和频率的电机状态
 */
struct TimedMotorState {
  std::vector<int16_t> motor_speed_mrad_per_s;  ///< Motor speeds in 0.001 rad/s | 电机速度，单位 0.001 rad/s
  std::vector<int16_t> motor_current_ma;        ///< Motor currents in mA | 电机电流，单位 mA
  std::vector<int32_t> motor_position_rad;      ///< Motor positions in rad | 电机位置，单位 rad
  std::vector<int32_t> motor_effort_mNm;        ///< Motor torques in 0.001 N·m | 电机力矩，单位 0.001 N·m
  std::chrono::steady_clock::time_point timestamp{}; ///< Update timestamp | 更新时间戳
  double hz{0.0};                                    ///< Update frequency in Hz | 更新频率，单位 Hz
};

/**
 * @struct TimedDriverState
 * @brief Driver state with timestamp and frequency
 *        带时间戳和频率的驱动器状态
 */
struct TimedDriverState {
  std::vector<LowSpeedDriverStatus> drivers;         ///< Driver status per joint | 每个关节的驱动器状态
  std::chrono::steady_clock::time_point timestamp{}; ///< Update timestamp | 更新时间戳
  double hz{0.0};                                    ///< Update frequency in Hz | 更新频率，单位 Hz
};

/**
 * @struct JointCtrlState
 * @brief Joint control command state
 *        关节控制命令状态
 */
struct JointCtrlState {
  std::array<int32_t, 6> joints_mdeg{}; ///< Joint target positions in 0.001 deg | 关节目标位置，单位 0.001 deg
};

/**
 * @struct GripperCtrlState
 * @brief Gripper control command state
 *        夹爪控制命令状态
 */
struct GripperCtrlState {
  int32_t gripper_angle_um{0};    ///< Gripper target position in 0.001 mm | 夹爪目标位置，单位 0.001 mm
  int16_t gripper_effort_mNm{0};  ///< Gripper target effort in 0.001 N·m | 夹爪目标力矩，单位 0.001 N·m
  uint8_t status_code{0};         ///< Status code | 状态码
  uint8_t set_zero_flag{0};       ///< Zero position flag | 零位标志
};

/**
 * @struct MotionControlParameters
 * @brief Motion control configuration parameters (internal cache)
 *        运动控制配置参数（内部缓存）
 * 
 * @details These parameters control motion behavior such as speed, acceleration,
 *          and trajectory execution mode. Used internally for caching current state.
 *          Note: External API uses individual parameters instead of this struct.
 *          这些参数控制运动行为，如速度、加速度和轨迹执行模式。
 *          内部用于缓存当前状态。
 *          注意：外部API使用独立参数而非此结构体。
 */
struct MotionControlParameters {
  uint8_t ctrl_mode{0x01};       ///< Control mode selector | 控制模式选择器
  uint8_t move_mode{0x01};       ///< Movement mode selector | 运动模式选择器
  uint8_t speed_percent{50};     ///< Speed percentage (0-100%) | 速度百分比（0-100%）
  uint8_t mit_mode{0x00};        ///< MIT mode enable flag | MIT模式使能标志
  uint8_t residence_time{0x00};  ///< Waypoint residence time | 路径点停留时间
  uint8_t installation_pos{0x00}; ///< Installation position configuration | 安装位置配置
};

/**
 * @struct Limits
 * @brief Velocity and acceleration limits
 *        速度和加速度限制
 */
struct Limits {
  int32_t velocity{0};      ///< Velocity limit | 速度限制
  int32_t acceleration{0};  ///< Acceleration limit | 加速度限制
};

/**
 * @struct MotionControl1Cache
 * @brief Cached motion control command 1 parameters (CAN ID 0x150)
 *        缓存的运动控制命令1参数（CAN ID 0x150）
 */
struct MotionControl1Cache {
  uint8_t emergency_stop{0};   ///< Emergency stop flag | 急停标志
  uint8_t track_ctrl{0};       ///< Trajectory control flag | 轨迹控制标志
  uint8_t teach_ctrl{0};       ///< Teaching control flag | 示教控制标志
  uint8_t trajectory_index{0}; ///< Trajectory index | 轨迹索引
  uint16_t name_index{0};      ///< Name index | 名称索引
  uint16_t crc16{0};           ///< CRC16 checksum | CRC16校验和
};

/**
 * @struct MotionCommand
 * @brief Unified motion command structure
 *        统一运动命令结构
 * 
 * @details Can represent either joint-space or Cartesian-space motion commands.
 *          可以表示关节空间或笛卡尔空间运动命令。
 */
struct MotionCommand {
  std::variant<std::vector<int32_t>, CartesianPose> target; ///< Target position (joint or Cartesian) | 目标位置（关节或笛卡尔）
  Limits limits;                                            ///< Motion limits | 运动限制
  MotionMode mode{MotionMode::kJoint};                     ///< Motion mode | 运动模式
};

/**
 * @struct TimedJointCtrl
 * @brief Joint control command with timestamp and frequency
 *        带时间戳和频率的关节控制命令
 */
struct TimedJointCtrl {
  JointCtrlState ctrl{};                                 ///< Joint control state | 关节控制状态
  std::chrono::steady_clock::time_point timestamp{};    ///< Command timestamp | 命令时间戳
  double hz{0.0};                                       ///< Command frequency in Hz | 命令频率，单位 Hz
};

/**
 * @struct TimedGripperCtrl
 * @brief Gripper control command with timestamp and frequency
 *        带时间戳和频率的夹爪控制命令
 */
struct TimedGripperCtrl {
  GripperCtrlState ctrl{};                              ///< Gripper control state | 夹爪控制状态
  std::chrono::steady_clock::time_point timestamp{};   ///< Command timestamp | 命令时间戳
  double hz{0.0};                                      ///< Command frequency in Hz | 命令频率，单位 Hz
};

/**
 * @struct TimedModeCtrl
 * @brief Motion control parameters with timestamp and frequency
 *        带时间戳和频率的运动控制参数
 */
struct TimedModeCtrl {
  MotionControlParameters params{};                     ///< Motion control parameters | 运动控制参数
  std::chrono::steady_clock::time_point timestamp{};   ///< Parameter update timestamp | 参数更新时间戳
  double hz{0.0};                                      ///< Update frequency in Hz | 更新频率，单位 Hz
};

/**
 * @struct MotorLimits
 * @brief Motor angle and speed limits
 *        电机角度和速度限制
 * 
 * @note Angle limits use 0.1 degree units, NOT 0.001 degree!
 *       角度限制使用 0.1 度单位，不是 0.001 度！
 */
struct MotorLimits {
  uint8_t motor_index{0};             ///< Motor index (1-6) | 电机索引（1-6）
  int16_t max_angle_limit{0};         ///< Maximum angle in 0.1 deg | 最大角度，单位 0.1 deg
  int16_t min_angle_limit{0};         ///< Minimum angle in 0.1 deg | 最小角度，单位 0.1 deg
  uint16_t max_joint_speed{0};        ///< Maximum speed in 0.001 rad/s | 最大速度，单位 0.001 rad/s
  std::optional<uint16_t> max_joint_accel; ///< Maximum acceleration in 0.01 rad/s² | 最大加速度，单位 0.01 rad/s²
};

/**
 * @struct AllMotorLimits
 * @brief Aggregated motor limits for all 6 joints
 *        所有6个关节的聚合电机限制
 * 
 * @details Matches Python SDK behavior for collecting limits from all motors.
 *          匹配 Python SDK 从所有电机收集限制的行为。
 */
struct AllMotorLimits {
  std::array<std::optional<MotorLimits>, 6> motors{}; ///< Limits per motor | 每个电机的限制
  std::chrono::steady_clock::time_point timestamp{};  ///< Collection timestamp | 收集时间戳
  
  /**
   * @brief Check if all motor limits have been received
   *        检查是否已接收所有电机限制
   * @return true if all 6 motors have limit data | 如果所有6个电机都有限制数据则返回 true
   */
  bool all_received() const {
    return std::all_of(motors.begin(), motors.end(), 
                      [](const auto& m) { return m.has_value(); });
  }
  
  /**
   * @brief Clear all motor limits
   *        清除所有电机限制
   */
  void clear() {
    for (auto& m : motors) {
      m.reset();
    }
  }
};

/**
 * @struct EndVelocityLimits
 * @brief End-effector velocity and acceleration limits
 *        末端执行器速度和加速度限制
 */
struct EndVelocityLimits {
  uint16_t max_linear_velocity{0};     ///< Max linear velocity in 0.001 m/s | 最大线速度，单位 0.001 m/s
  uint16_t max_angular_velocity{0};    ///< Max angular velocity in 0.001 rad/s | 最大角速度，单位 0.001 rad/s
  uint16_t max_linear_acceleration{0}; ///< Max linear acceleration in 0.001 m/s² | 最大线加速度，单位 0.001 m/s²
  uint16_t max_angular_acceleration{0}; ///< Max angular acceleration in 0.001 rad/s² | 最大角加速度，单位 0.001 rad/s²
};

/**
 * @struct GripperCommand
 * @brief Gripper control command
 *        夹爪控制命令
 */
struct GripperCommand {
  int32_t position_um{0};       ///< Target position in 0.001 mm | 目标位置，单位 0.001 mm
  int32_t effort_mNm{0};        ///< Target effort in 0.001 N·m | 目标力矩，单位 0.001 N·m
  uint8_t gripper_code{0x01};   ///< Gripper control code | 夹爪控制码
  uint8_t set_zero{0x00};       ///< Set zero position flag | 设置零位标志
};

/**
 * @struct MitJointCommand
 * @brief MIT motor control command (raw format)
 *        MIT电机控制命令（原始格式）
 * 
 * @details Low-level MIT Cheetah motor control protocol command structure.
 *          底层 MIT Cheetah 电机控制协议命令结构。
 */
struct MitJointCommand {
  uint16_t pos_ref{0};      ///< Raw position reference | 原始位置参考
  uint16_t vel_ref{0};      ///< 12-bit velocity reference | 12位速度参考
  uint16_t kp{10};          ///< Proportional gain (12-bit) | 比例增益（12位）
  uint16_t kd{1};           ///< Derivative gain (12-bit) | 微分增益（12位）
  uint8_t torque_ref{0};    ///< 8-bit torque reference | 8位力矩参考
  uint8_t crc{0};           ///< 4-bit CRC checksum | 4位CRC校验和
};

/**
 * @struct CollisionProtectionConfig
 * @brief Collision protection configuration per joint
 *        每个关节的碰撞保护配置
 */
struct CollisionProtectionConfig {
  std::array<uint8_t, 6> joint_levels{};  ///< Protection levels 0-8 per joint (0 = disabled) | 每个关节的保护等级 0-8（0 = 禁用）
};

/**
 * @struct SoftwareLimitsConfig
 * @brief SDK software limits configuration
 *        SDK软件限制配置
 * 
 * @details Software limits enforced by the SDK before sending commands to hardware.
 *          SDK在向硬件发送命令前强制执行的软件限制。
 */
struct SoftwareLimitsConfig {
  std::vector<int32_t> joint_min_mdeg;     ///< Joint minimum angles in 0.001 deg | 关节最小角度，单位 0.001 deg
  std::vector<int32_t> joint_max_mdeg;     ///< Joint maximum angles in 0.001 deg | 关节最大角度，单位 0.001 deg
  std::optional<int32_t> gripper_min_um;   ///< Gripper minimum position in 0.001 mm | 夹爪最小位置，单位 0.001 mm
  std::optional<int32_t> gripper_max_um;   ///< Gripper maximum position in 0.001 mm | 夹爪最大位置，单位 0.001 mm
  bool enable_joint_limits{false};         ///< Enable joint limit enforcement | 启用关节限制强制执行
  bool enable_gripper_limits{false};       ///< Enable gripper limit enforcement | 启用夹爪限制强制执行
};

/**
 * @struct GripperTeachingPendantParam
 * @brief Gripper teaching pendant parameters
 *        夹爪示教器参数
 */
struct GripperTeachingPendantParam {
  std::array<uint8_t, 8> raw{}; ///< Raw parameter bytes | 原始参数字节
};

/**
 * @struct ConnectStatus
 * @brief CAN bus connection status
 *        CAN总线连接状态
 */
struct ConnectStatus {
  bool connected{false};  ///< CAN transport connected | CAN传输已连接
  bool receiving{false};  ///< Receiving thread active | 接收线程活动
  bool stale{true};       ///< Feedback data is stale | 反馈数据过期
  std::chrono::steady_clock::time_point last_feedback_time{}; ///< Last feedback timestamp | 最后反馈时间戳
};

}  // namespace piper_sdk
