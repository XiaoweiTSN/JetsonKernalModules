/**
 * @file piper_interface.hpp
 * @brief Main interface for Piper robot arm control
 *        Piper 机械臂控制主接口
 * 
 * @details Comprehensive C++ SDK for controlling Piper robot arms via CAN bus.
 *          Provides high-level control commands, state feedback, and configuration.
 *          通过 CAN 总线控制 Piper 机械臂的综合 C++ SDK。
 *          提供高级控制命令、状态反馈和配置。
 * 
 * @author Wesley Cui 崔笑唯
 * @copyright Copyright (c) 2025 TNCA
 * @version 0.1.0
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

#include <array>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "piper_sdk/can_transport.hpp"
#include "piper_sdk/types.hpp"
#include "piper_sdk/logger.hpp"
#include "piper_sdk/fps_counter.hpp"
#include "piper_sdk/kinematics.hpp"

namespace piper_sdk {

/**
 * @struct PiperConfig
 * @brief Configuration parameters for PiperInterface
 *        PiperInterface 的配置参数
 * 
 * @details Contains all initialization settings for the robot arm interface.
 *          包含机械臂接口的所有初始化设置。
 */
struct PiperConfig {
  size_t joint_count{6};                          ///< Number of joints (default: 6) | 关节数量（默认：6）
  Limits default_limits{100, 0};                  ///< Default velocity/acceleration limits | 默认速度/加速度限制
  MotionControlParameters motion_params{};        ///< Motion control parameters | 运动控制参数
  bool dh_is_offset{true};                        ///< Use modified DH parameters | 使用修改的 DH 参数
  SoftwareLimitsConfig software_limits{};         ///< Software joint/gripper limits | 软件关节/夹爪限制
  bool enable_fk_calculation{true};               ///< Enable automatic FK calculation | 启用自动正运动学计算
  LogLevel logger_level{LogLevel::kInfo};         ///< Logging level | 日志级别
  bool log_to_file{false};                        ///< Enable file logging | 启用文件日志记录
  std::string log_file_path;                      ///< Log file path | 日志文件路径
  std::string can_name;                           ///< CAN interface name | CAN 接口名称
};

/**
 * @class PiperInterface
 * @brief Main interface for Piper robot arm control
 *        Piper 机械臂控制主接口
 * 
 * @details Thread-safe interface for controlling Piper robot arms.
 *          Supports multiple control modes: joint space, Cartesian space, MIT motor control.
 *          Provides state feedback, forward kinematics, and hardware configuration.
 *          用于控制 Piper 机械臂的线程安全接口。
 *          支持多种控制模式：关节空间、笛卡尔空间、MIT 电机控制。
 *          提供状态反馈、正运动学和硬件配置。
 */
class PiperInterface {
 public:
  /**
   * @brief Construct Piper interface with CAN transport and configuration
   *        使用 CAN 传输和配置构造 Piper 接口
   * 
   * @param transport Shared pointer to CAN transport implementation | CAN 传输实现的共享指针
   * @param config Configuration parameters | 配置参数
   */
  explicit PiperInterface(std::shared_ptr<CanTransport> transport, PiperConfig config = {});
  
  /**
   * @brief Destructor - stops background threads and closes connections
   *        析构函数 - 停止后台线程并关闭连接
   */
  ~PiperInterface();

  // Non-copyable | 不可复制
  PiperInterface(const PiperInterface&) = delete;
  PiperInterface& operator=(const PiperInterface&) = delete;
  
  // Movable | 可移动
  PiperInterface(PiperInterface&&) = default;
  PiperInterface& operator=(PiperInterface&&) = default;

  //============================================================================
  // Basic Control Commands | 基本控制命令
  //============================================================================

  /**
   * @brief 使能机械臂电源和伺服控制
   * 
   * @details 发送使能指令以打开机械臂电源和伺服控制。
   *          可以单独使能某个关节，或使能所有关节。
   *          
   *          CAN ID：
   *            0x471（电机使能/失能配置指令）
   *          
   *          数据布局：
   *            Byte0 = motor_num（1-7，7 表示全部电机）
   *            Byte1 = 0x02（使能命令）
   * 
   * @param motor_num 电机编号
   *                  - 1-6：单独使能特定关节
   *                  - 7：使能所有关节（默认值）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Enable arm power and servo control
   * 
   * @details Sends an enable command to power on and activate servo control for the robot arm.
   *          Can enable a specific joint individually, or enable all joints at once.
   *          
   *          CAN ID:
   *            0x471 (ARM_MOTOR_ENABLE_DISABLE_CONFIG)
   *          
   *          Frame layout:
   *            Byte0 = motor_num (1-7, 7 = all motors)
   *            Byte1 = 0x02 (enable command)
   * 
   * @param motor_num Motor number
   *                  - 1-6: Enable specific joint
   *                  - 7: Enable all joints (default)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult enable_arm(uint8_t motor_num = 7);
  
  /**
   * @brief 禁用机械臂电源和伺服控制
   * 
   * @details 发送禁用指令以关闭机械臂电源和伺服控制。
   *          可以单独禁用某个关节，或禁用所有关节。
   *          
   *          CAN ID：
   *            0x471（电机使能/失能配置指令）
   *          
   *          数据布局：
   *            Byte0 = motor_num（1-7，7 表示全部电机）
   *            Byte1 = 0x01（失能命令）
   * 
   * @param motor_num 电机编号
   *                  - 1-6：单独禁用特定关节
   *                  - 7：禁用所有关节（默认值）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Disable arm power and servo control
   * 
   * @details Sends a disable command to power off and deactivate servo control for the robot arm.
   *          Can disable a specific joint individually, or disable all joints at once.
   *          
   *          CAN ID:
   *            0x471 (ARM_MOTOR_ENABLE_DISABLE_CONFIG)
   *          
   *          Frame layout:
   *            Byte0 = motor_num (1-7, 7 = all motors)
   *            Byte1 = 0x01 (disable command)
   * 
   * @param motor_num Motor number
   *                  - 1-6: Disable specific joint
   *                  - 7: Disable all joints (default)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult disable_arm(uint8_t motor_num = 7);
  
  /**
   * @brief 触发或释放急停
   * 
   * @details 机械臂紧急停止以及恢复指令。
   *          
   *          CAN ID：
   *            0x150
   * 
   * @param emergency_code 急停控制码 uint8
   *                       - 0x00：无效（默认值）
   *                       - 0x01：触发急停
   *                       - 0x02：释放急停/恢复
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Trigger or release emergency stop
   * 
   * @details Emergency stop and recovery command for the robotic arm.
   *          
   *          CAN ID:
   *            0x150
   * 
   * @param emergency_code Emergency control code uint8
   *                       - 0x00: Invalid (default)
   *                       - 0x01: Trigger emergency stop
   *                       - 0x02: Release emergency stop / Resume
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult emergency_stop(uint8_t emergency_code = 0x00);
  
  /**
   * @brief 重置机械臂，清除错误并断电
   * 
   * @details 机械臂会立刻失电落下，清除所有错误和内部标志位。
   *          
   *          CAN ID：
   *            0x150
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Reset the arm, clearing errors and dropping power
   * 
   * @details The robot will immediately lose power and fall down, clearing all errors and internal flags.
   *          
   *          CAN ID:
   *            0x150
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult reset_arm();

  //============================================================================
  // Motion Control Commands | 运动控制命令
  //============================================================================

  /**
   * @brief 发送运动控制命令1（机械臂快速急停 | 轨迹指令 | 拖动示教指令）
   * 
   * @details 机械臂运动控制指令1，多功能命令。
   *          包括：快速急停控制、轨迹控制、拖动示教控制。
   *          
   *          CAN ID：
   *            0x150
   * 
   * @param emergency_stop 快速急停 uint8
   *                       - 0x00：无效
   *                       - 0x01：快速急停
   *                       - 0x02：恢复
   * @param track_ctrl 轨迹控制 uint8
   *                   - 0x00：无效
   *                   - 0x01：暂停当前规划
   *                   - 0x02：继续当前轨迹
   *                   - 0x03：清除当前轨迹
   *                   - 0x04：清除所有轨迹
   *                   - 0x05：获取当前规划轨迹
   *                   - 0x06：终止执行
   *                   - 0x07：轨迹传输
   *                   - 0x08：轨迹传输结束
   * @param teach_ctrl 拖动示教控制 uint8
   *                   - 0x00：无效
   *                   - 0x01：开始示教
   *                   - 0x02：停止示教
   *                   - 0x03：执行示教轨迹
   *                   - 0x04：暂停执行
   *                   - 0x05：继续执行
   *                   - 0x06：终止执行
   *                   - 0x07：运动到轨迹起点
   * @param trajectory_index 轨迹索引 uint8（0-255）
   *                         主控收到后会应答 0x476 byte0=0x50; byte2=N
   * @param name_index 名称索引 uint16（用于轨迹包）
   * @param crc16 CRC16校验和 uint16
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Send motion control command 1 (ARM_STOP_CTRL | ARM_TRACK_CTRL | ARM_GRAG_TEACH_CTRL)
   * 
   * @details Robotic arm motion control command 1, multi-function command.
   *          Includes: emergency stop control, trajectory control, drag teaching control.
   *          
   *          CAN ID:
   *            0x150
   * 
   * @param emergency_stop Emergency stop uint8
   *                       - 0x00: Invalid
   *                       - 0x01: Emergency stop
   *                       - 0x02: Resume
   * @param track_ctrl Trajectory control uint8
   *                   - 0x00: Invalid
   *                   - 0x01: Pause current plan
   *                   - 0x02: Continue current trajectory
   *                   - 0x03: Clear current trajectory
   *                   - 0x04: Clear all trajectories
   *                   - 0x05: Get current planned trajectory
   *                   - 0x06: Terminate execution
   *                   - 0x07: Trajectory transfer
   *                   - 0x08: Trajectory transfer end
   * @param teach_ctrl Drag teaching control uint8
   *                   - 0x00: Invalid
   *                   - 0x01: Start teaching
   *                   - 0x02: Stop teaching
   *                   - 0x03: Execute taught trajectory
   *                   - 0x04: Pause execution
   *                   - 0x05: Continue execution
   *                   - 0x06: Terminate execution
   *                   - 0x07: Move to trajectory start
   * @param trajectory_index Trajectory index uint8 (0-255)
   *                         Controller responds with 0x476 byte0=0x50; byte2=N
   * @param name_index Name index uint16 (for trajectory packet)
   * @param crc16 CRC16 checksum uint16
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult motion_control_1(uint8_t emergency_stop = 0x00, uint8_t track_ctrl = 0x00, 
                                 uint8_t teach_ctrl = 0x00, uint8_t trajectory_index = 0x00, 
                                 uint16_t name_index = 0x0000, uint16_t crc16 = 0x0000);
  
  /**
   * @brief 发送运动控制命令2（机械臂模式控制 | 运动模式控制 | 速度百分比控制）
   * 
   * @details 机械臂运动控制指令2，多功能命令，并更新存储的参数。
   *          包括：控制模式设置、运动模式设置、速度百分比设置、MIT模式设置等。
   *          
   *          相关函数：
   *          - move_joint(): 关节空间运动（需先设置 move_mode = 0x01）
   *          - move_cartesian(): 笛卡尔空间运动（需先设置 move_mode = 0x02）
   *          - move_circular(): 圆弧运动（需先设置 move_mode = 0x03）
   *          - control_joint_mit(): MIT模式控制（需先设置 is_mit_mode = 0xAD）
   *          
   *          CAN ID：
   *            0x151
   * 
   * @param ctrl_mode 控制模式 uint8
   *                  - 0x00：待机模式
   *                  - 0x01：CAN指令控制模式
   *                  - 0x03：以太网控制模式
   *                  - 0x04：WiFi控制模式
   *                  - 0x07：离线轨迹模式
   * @param move_mode MOVE模式 uint8
   *                  - 0x00：MOVE P（点到点）
   *                  - 0x01：MOVE J（关节空间）
   *                  - 0x02：MOVE L（直线）
   *                  - 0x03：MOVE C（圆弧）
   *                  - 0x04：MOVE M（基于V1.5-2版本后）
   *                  - 0x05：MOVE CPV（基于V1.8-1版本后）
   * @param move_spd_rate_ctrl 运动速度百分比 uint8（范围0~100）
   * @param is_mit_mode MIT模式 uint8
   *                    - 0x00：位置速度模式
   *                    - 0xAD：MIT模式
   *                    - 0xFF：无效
   * @param residence_time 离线轨迹点停留时间 uint8（0~254单位秒，255表示轨迹终止）
   * @param installation_pos 安装位置 uint8（注意接线朝后，基于V1.5-2版本后）
   *                         - 0x00：无效值
   *                         - 0x01：水平正装
   *                         - 0x02：侧装左
   *                         - 0x03：侧装右
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Send motion control command 2 (ARM_MODE_CTRL | ARM_MOVE_MODE_CTRL | ARM_MOVE_SPD_RATE_CTRL)
   * 
   * @details Robotic arm motion control command 2, multi-function command, and updates stored parameters.
   *          Includes: control mode setting, motion mode setting, speed percentage setting, MIT mode setting, etc.
   *          
   *          Related functions:
   *          - move_joint(): Joint space motion (requires move_mode = 0x01)
   *          - move_cartesian(): Cartesian space motion (requires move_mode = 0x02)
   *          - move_circular(): Circular motion (requires move_mode = 0x03)
   *          - control_joint_mit(): MIT mode control (requires is_mit_mode = 0xAD)
   *          
   *          CAN ID:
   *            0x151
   * 
   * @param ctrl_mode Control mode uint8
   *                  - 0x00: Standby mode
   *                  - 0x01: CAN command control mode
   *                  - 0x03: Ethernet control mode
   *                  - 0x04: WiFi control mode
   *                  - 0x07: Offline trajectory mode
   * @param move_mode MOVE mode uint8
   *                  - 0x00: MOVE P (Point-to-point)
   *                  - 0x01: MOVE J (Joint space)
   *                  - 0x02: MOVE L (Linear)
   *                  - 0x03: MOVE C (Circular)
   *                  - 0x04: MOVE M (Based on version V1.5-2 and later)
   *                  - 0x05: MOVE CPV (Based on version V1.8-1 and later)
   * @param move_spd_rate_ctrl Movement speed percentage uint8 (range 0~100)
   * @param is_mit_mode MIT mode uint8
   *                    - 0x00: Position-velocity mode
   *                    - 0xAD: MIT mode
   *                    - 0xFF: Invalid
   * @param residence_time Offline trajectory point residence time uint8 (0~254 unit seconds, 255 means trajectory termination)
   * @param installation_pos Installation position uint8 (Pay attention to rear-facing wiring, based on version V1.5-2 and later)
   *                         - 0x00: Invalid value
   *                         - 0x01: Horizontal upright
   *                         - 0x02: Side mount left
   *                         - 0x03: Side mount right
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult motion_control_2(uint8_t ctrl_mode = 0x01, uint8_t move_mode = 0x01,
                                  uint8_t move_spd_rate_ctrl = 50, uint8_t is_mit_mode = 0x00,
                                  uint8_t residence_time = 0, uint8_t installation_pos = 0x00);

  /**
   * @brief 在关节空间中移动机械臂
   * 
   * @details 机械臂关节控制，发送前需要切换机械臂模式为关节控制模式。
   *          关节位置以 0.001 度单位指定（与 Python SDK 匹配）。
   *          
   *          CAN ID：
   *            0x155, 0x156, 0x157
   *          
   *          关节限制参考：
   *          | 关节名   | 限制范围（弧度）         | 限制范围（角度）        |
   *          |----------|------------------------|----------------------|
   *          | joint1   | [-2.6179, 2.6179]      | [-150.0, 150.0]      |
   *          | joint2   | [0, 3.14]              | [0, 180.0]           |
   *          | joint3   | [-2.967, 0]            | [-170, 0]            |
   *          | joint4   | [-1.745, 1.745]        | [-100.0, 100.0]      |
   *          | joint5   | [-1.22, 1.22]          | [-70.0, 70.0]        |
   *          | joint6   | [-2.09439, 2.09439]    | [-120.0, 120.0]      |
   * 
   * @param positions_mdeg 目标关节位置，单位 0.001 度（6个关节的角度值）
   * @param limits 可选的速度/加速度限制
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Move arm in joint space
   * 
   * @details Joint control for the robotic arm. Before sending, switch the robotic arm mode to joint control mode.
   *          Joint positions are specified in 0.001 degree units (matching Python SDK).
   *          
   *          CAN ID:
   *            0x155, 0x156, 0x157
   *          
   *          Joint limits reference:
   *          | joint_name | limit(rad)             | limit(angle)         |
   *          |------------|------------------------|----------------------|
   *          | joint1     | [-2.6179, 2.6179]      | [-150.0, 150.0]      |
   *          | joint2     | [0, 3.14]              | [0, 180.0]           |
   *          | joint3     | [-2.967, 0]            | [-170, 0]            |
   *          | joint4     | [-1.745, 1.745]        | [-100.0, 100.0]      |
   *          | joint5     | [-1.22, 1.22]          | [-70.0, 70.0]        |
   *          | joint6     | [-2.09439, 2.09439]    | [-120.0, 120.0]      |
   * 
   * @param positions_mdeg Target joint positions in 0.001 deg (angle values for 6 joints)
   * @param limits Optional velocity/acceleration limits
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult move_joint(const std::vector<int32_t>& positions_mdeg,
                           std::optional<Limits> limits = std::nullopt);
  
  /**
   * @brief 在笛卡尔空间中移动机械臂
   * 
   * @details 机械臂末端数值发送，发送前需要切换机械臂模式为末端控制模式。
   *          末端表示为欧拉角。
   *          位置单位 0.001 mm，姿态单位 0.001 度（与 Python SDK 匹配）。
   *          
   *          CAN ID：
   *            0x152, 0x153, 0x154
   * 
   * @param pose 目标笛卡尔位姿
   *             - X：X坐标，单位 0.001 mm
   *             - Y：Y坐标，单位 0.001 mm
   *             - Z：Z坐标，单位 0.001 mm
   *             - RX：绕X轴旋转角度，单位 0.001 度
   *             - RY：绕Y轴旋转角度，单位 0.001 度
   *             - RZ：绕Z轴旋转角度，单位 0.001 度
   * @param limits 可选的速度/加速度限制
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Move arm in Cartesian space
   * 
   * @details End-effector control for the robotic arm. Before sending, switch the robotic arm mode to end-effector control mode.
   *          The end-effector is expressed as Euler angles.
   *          Position in 0.001 mm, orientation in 0.001 deg (matching Python SDK).
   *          
   *          CAN ID:
   *            0x152, 0x153, 0x154
   * 
   * @param pose Target Cartesian pose
   *             - X: X-axis coordinate, in 0.001 mm
   *             - Y: Y-axis coordinate, in 0.001 mm
   *             - Z: Z-axis coordinate, in 0.001 mm
   *             - RX: Rotation about X-axis, in 0.001 degrees
   *             - RY: Rotation about Y-axis, in 0.001 degrees
   *             - RZ: Rotation about Z-axis, in 0.001 degrees
   * @param limits Optional velocity/acceleration limits
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult move_cartesian(const CartesianPose& pose,
                               std::optional<Limits> limits = std::nullopt);
  
  /**
   * @brief 命令圆弧轨迹点更新
   * 
   * @details MoveC模式坐标点更新指令，发送前需要切换机械臂模式为MoveC控制模式。
   *          用于圆弧/圆周运动规划。
   *          
   *          CAN ID：
   *            0x158
   *          
   *          使用方法：
   *          1. 首先使用 move_cartesian 确定起点，然后调用 move_circular(0x01)
   *          2. 然后使用 move_cartesian 确定中点，然后调用 move_circular(0x02)
   *          3. 最后使用 move_cartesian 确定终点，然后调用 move_circular(0x03)
   * 
   *          示例代码:
   *          ```cpp
   *          // 1. 设置圆弧运动模式
   *          interface.motion_control_2(0x01, 0x03);  // CAN模式，MOVE C
   *          
   *          // 2. 定义起点
   *          CartesianPose start{100000, 200000, 300000, 0, 0, 0};
   *          interface.move_cartesian(start);
   *          interface.move_circular(0x01);  // 标记为起点
   *          
   *          // 3. 定义中点
   *          CartesianPose mid{150000, 250000, 300000, 0, 0, 0};
   *          interface.move_cartesian(mid);
   *          interface.move_circular(0x02);  // 标记为中点
   *          
   *          // 4. 定义终点
   *          CartesianPose end{200000, 200000, 300000, 0, 0, 0};
   *          interface.move_cartesian(end);
   *          interface.move_circular(0x03);  // 标记为终点并执行
   *          ```
   * 
   * @param instruction_num 指令点序号 uint8
   *                        - 0x00：无效
   *                        - 0x01：起点
   *                        - 0x02：中点
   *                        - 0x03：终点
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Command a circular trajectory point update
   * 
   * @details MoveC mode coordinate point update command. Before sending, switch the robotic arm mode to MoveC control mode.
   *          Used for circular/arc motion planning.
   *          
   *          CAN ID:
   *            0x158
   *          
   *          Usage:
   *          1. First use move_cartesian to determine the start point, then call move_circular(0x01)
   *          2. Then use move_cartesian to determine the midpoint, then call move_circular(0x02)
   *          3. Finally use move_cartesian to determine the endpoint, then call move_circular(0x03)
   * 
   *          Example code:
   *          ```cpp
   *          // 1. Set circular motion mode
   *          interface.motion_control_2(0x01, 0x03);  // CAN mode, MOVE C
   *          
   *          // 2. Define start point
   *          CartesianPose start{100000, 200000, 300000, 0, 0, 0};
   *          interface.move_cartesian(start);
   *          interface.move_circular(0x01);  // Mark as start point
   *          
   *          // 3. Define mid point
   *          CartesianPose mid{150000, 250000, 300000, 0, 0, 0};
   *          interface.move_cartesian(mid);
   *          interface.move_circular(0x02);  // Mark as mid point
   *          
   *          // 4. Define end point
   *          CartesianPose end{200000, 200000, 300000, 0, 0, 0};
   *          interface.move_cartesian(end);
   *          interface.move_circular(0x03);  // Mark as end point and execute
   *          ```
   * 
   * @param instruction_num Instruction point sequence number uint8
   *                        - 0x00: Invalid
   *                        - 0x01: Start point
   *                        - 0x02: Midpoint
   *                        - 0x03: Endpoint
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult move_circular(uint8_t instruction_num);

  /**
   * @brief 设置运动模式
   * 
   * @details 封装 motion_control_2 的便捷方法，用于切换机械臂运动模式。
   *          
   *          CAN ID：
   *            0x151
   * 
   * @param mode 要激活的运动模式
   *             - kPoint (0x00)：点到点模式 MOVE P
   *             - kJoint (0x01)：关节空间模式 MOVE J
   *             - kLinear (0x02)：直线/笛卡尔模式 MOVE L
   *             - kCircular (0x03)：圆弧模式 MOVE C
   *             - kMit (0x04)：MIT控制模式 MOVE M（mit_mode = 0xAD）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Set motion mode
   * 
   * @details Convenience wrapper for motion_control_2 to switch arm motion mode.
   *          
   *          CAN ID:
   *            0x151
   * 
   * @param mode Motion mode to activate
   *             - kPoint (0x00): Point-to-point mode MOVE P
   *             - kJoint (0x01): Joint space mode MOVE J
   *             - kLinear (0x02): Linear/Cartesian mode MOVE L
   *             - kCircular (0x03): Circular/arc mode MOVE C
   *             - kMit (0x04): MIT control mode MOVE M (mit_mode = 0xAD)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult set_motion_mode(MotionMode mode);

  //============================================================================
  // Gripper and MIT Control | 夹爪和 MIT 控制
  //============================================================================

  /**
   * @brief 控制夹爪
   * 
   * @details 夹爪控制指令。
   *          使用 0.001 mm 和 0.001 N·m 单位（与 Python SDK GripperCtrl 单位和行为匹配）。
   *          
   *          CAN ID：
   *            0x159
   * 
   *          注意事项：
   *          - gripper_effort 范围限制为 0-5000 (0-5 N·m)，超出范围将被截断
   *          - 设置 set_zero = 0xAE 会重置夹爪零点，请谨慎使用
   *          - 在使能夹爪前（gripper_code = 0x01/0x03），确保机械臂已使能
   * 
   * @param command 夹爪控制命令结构体
   *                - gripper_angle：夹爪目标开口角度，单位 0.001 mm
   *                - gripper_effort：夹爪力矩，单位 0.001 N·m
   *                - gripper_code：夹爪控制码 uint8
   *                  * 0x00：失能夹爪驱动
   *                  * 0x01：使能夹爪驱动
   *                  * 0x02：失能并清除错误
   *                  * 0x03：使能并清除错误
   *                - set_zero：设置零位标志 uint8
   *                  * 0x00：无效
   *                  * 0xAE：设置当前位置为零位并清除错误
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Control gripper
   * 
   * @details Gripper control command.
   *          Uses 0.001 mm and 0.001 N·m units (matches Python SDK GripperCtrl units and behavior).
   *          
   *          CAN ID:
   *            0x159
   * 
   *          Important notes:
   *          - gripper_effort is limited to 0-5000 (0-5 N·m), values outside range will be clamped
   *          - Setting set_zero = 0xAE will reset gripper zero position, use with caution
   *          - Before enabling gripper (gripper_code = 0x01/0x03), ensure arm is already enabled
   * 
   * @param command Gripper control command structure
   *                - gripper_angle: Gripper target opening angle, in 0.001 mm
   *                - gripper_effort: Gripper torque, in 0.001 N·m
   *                - gripper_code: Gripper control code uint8
   *                  * 0x00: Disable drive
   *                  * 0x01: Enable drive
   *                  * 0x02: Disable + clear error
   *                  * 0x03: Enable + clear error
   *                - set_zero: Set zero position flag uint8
   *                  * 0x00: Invalid
   *                  * 0xAE: Set current position as zero and clear errors
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult control_gripper(const GripperCommand& command);
  
  /**
   * @brief 在 MIT 模式下控制关节
   * 
   * @details MIT模式关节控制指令。
   *          将物理单位转换为定点表示并发送 MIT 控制命令。
   *          基于 V1.5-2 版本后。
   *          
   *          CAN ID：
   *            0x15A：关节1 MIT控制
   *            0x15B：关节2 MIT控制
   *            0x15C：关节3 MIT控制
   *            0x15D：关节4 MIT控制
   *            0x15E：关节5 MIT控制
   *            0x15F：关节6 MIT控制
   * 
   * @param motor_num 电机编号（1-6）
   * @param pos_ref 目标位置，单位 rad，范围 [-12.5, 12.5]
   * @param vel_ref 目标速度，单位 rad/s，范围 [-45.0, 45.0]
   * @param kp 位置比例增益，范围 [0.0, 500.0]，参考值 10.0
   * @param kd 速度微分增益，范围 [-5.0, 5.0]，参考值 0.8
   * @param torque_ref 目标力矩，单位 N·m，范围 [-18.0, 18.0]
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Control a joint in MIT mode
   * 
   * @details MIT mode joint control command.
   *          Converts physical units to fixed-point representation and sends MIT control command.
   *          Based on V1.5-2 version and later.
   *          
   *          CAN ID:
   *            0x15A: Joint 1 MIT control
   *            0x15B: Joint 2 MIT control
   *            0x15C: Joint 3 MIT control
   *            0x15D: Joint 4 MIT control
   *            0x15E: Joint 5 MIT control
   *            0x15F: Joint 6 MIT control
   * 
   * @param motor_num Motor number (1-6)
   * @param pos_ref Target position in rad, range [-12.5, 12.5]
   * @param vel_ref Target velocity in rad/s, range [-45.0, 45.0]
   * @param kp Position proportional gain, range [0.0, 500.0], typical: 10.0
   * @param kd Velocity derivative gain, range [-5.0, 5.0], typical: 0.8
   * @param torque_ref Target torque in N·m, range [-18.0, 18.0]
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult control_joint_mit(uint8_t motor_num, double pos_ref, double vel_ref, double kp,
                                  double kd, double torque_ref);
  
  /**
   * @brief MIT 模式原始命令控制（低层接口）
   * 
   * @details 发送带打包参数的 MIT 电机控制命令（原始定点格式）。
   *          打包命令包含定点表示：
   *          - pos_ref：16位位置参考值
   *          - vel_ref：12位速度参考值
   *          - kp：12位比例增益
   *          - kd：12位微分增益
   *          - torque_ref：8位力矩参考值
   *          
   *          CAN ID：
   *            0x15A：关节1 MIT控制
   *            0x15B：关节2 MIT控制
   *            0x15C：关节3 MIT控制
   *            0x15D：关节4 MIT控制
   *            0x15E：关节5 MIT控制
   *            0x15F：关节6 MIT控制
   * 
   * @param motor_num 电机编号（1-6）
   * @param command 打包的定点格式命令
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief MIT mode raw command control (low-level interface)
   * 
   * @details Sends MIT motor control command with packed parameters (raw fixed-point format).
   *          The packed command contains fixed-point representations:
   *          - pos_ref: 16-bit position reference
   *          - vel_ref: 12-bit velocity reference
   *          - kp: 12-bit proportional gain
   *          - kd: 12-bit derivative gain
   *          - torque_ref: 8-bit torque reference
   *          
   *          CAN ID:
   *            0x15A: Joint 1 MIT control
   *            0x15B: Joint 2 MIT control
   *            0x15C: Joint 3 MIT control
   *            0x15D: Joint 4 MIT control
   *            0x15E: Joint 5 MIT control
   *            0x15F: Joint 6 MIT control
   * 
   * @param motor_num Motor number (1-6)
   * @param command Packed fixed-point command
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult control_joint_mit_raw(uint8_t motor_num, const MitJointCommand& command);

  //============================================================================
  // Configuration Commands | 配置命令
  //============================================================================

  /**
   * @brief 配置主从联动
   * 
   * @details 随动主从模式设置指令，配置主从示教模式和 CAN ID 偏移。
   *          
   *          CAN ID：
   *            0x470
   * 
   * @param linkage_config 联动设置指令 uint8
   *                       - 0x00：无效
   *                       - 0xFA：设置为示教输入臂
   *                       - 0xFC：设置为运动输出臂
   * @param feedback_offset 反馈指令偏移值 uint8
   *                        - 0x00：不偏移/恢复默认
   *                        - 0x10：反馈指令基ID由2Ax偏移为2Bx
   *                        - 0x20：反馈指令基ID由2Ax偏移为2Cx
   * @param ctrl_offset 控制指令偏移值 uint8
   *                    - 0x00：不偏移/恢复默认
   *                    - 0x10：控制指令基ID由15x偏移为16x
   *                    - 0x20：控制指令基ID由15x偏移为17x
   * @param linkage_offset 联动模式控制目标地址偏移值 uint8
   *                       - 0x00：不偏移/恢复默认
   *                       - 0x10：控制目标地址基ID由15x偏移为16x
   *                       - 0x20：控制目标地址基ID由15x偏移为17x
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Configure master/slave linkage
   * 
   * @details Master-slave follow mode configuration command, configures master-slave teaching mode and CAN ID offsets.
   *          
   *          CAN ID:
   *            0x470
   * 
   * @param linkage_config Linkage setting command uint8
   *                       - 0x00: Invalid
   *                       - 0xFA: Set as teaching input arm
   *                       - 0xFC: Set as motion output arm
   * @param feedback_offset Feedback command offset value uint8
   *                        - 0x00: No offset / restore default
   *                        - 0x10: Feedback command base ID shifts from 2Ax to 2Bx
   *                        - 0x20: Feedback command base ID shifts from 2Ax to 2Cx
   * @param ctrl_offset Control command offset value uint8
   *                    - 0x00: No offset / restore default
   *                    - 0x10: Control command base ID shifts from 15x to 16x
   *                    - 0x20: Control command base ID shifts from 15x to 17x
   * @param linkage_offset Linkage mode control target address offset value uint8
   *                       - 0x00: No offset / restore default
   *                       - 0x10: Control target address base ID shifts from 15x to 16x
   *                       - 0x20: Control target address base ID shifts from 15x to 17x
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult configure_master_slave(uint8_t linkage_config, uint8_t feedback_offset,
                                       uint8_t ctrl_offset, uint8_t linkage_offset);
  
  /**
   * @brief 配置碰撞保护等级
   * 
   * @details 为每个关节设置碰撞保护灵敏度等级（0-8）。
   *          
   *          CAN ID：
   *            0x47A
   *          
   *          保护等级：
   *          - 0：不检测碰撞
   *          - 1-8：检测等级逐级递增（检测阈值逐级增加）
   *          
   *          反馈通过 CAN ID 0x47B（碰撞防护等级反馈指令）
   * 
   * @param config 碰撞保护配置结构体
   *               包含6个关节的碰撞防护等级（每个关节值范围0-8）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Configure collision protection levels
   * 
   * @details Sets collision protection sensitivity levels (0-8) for each joint.
   *          
   *          CAN ID:
   *            0x47A
   *          
   *          Protection levels:
   *          - 0: No collision detection
   *          - 1-8: Detection levels increase progressively (detection threshold increases progressively)
   *          
   *          Feedback via CAN ID 0x47B (collision protection level feedback command)
   * 
   * @param config Collision protection configuration structure
   *               Contains collision protection levels for 6 joints (each joint value range 0-8)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult configure_collision_protection(const CollisionProtectionConfig& config);
  
  /**
   * @brief 控制板载照明
   * 
   * @details 灯光控制指令。
   *          
   *          CAN ID：
   *            0x121
   * 
   * @param mode 照明模式 uint8
   * @param brightness 亮度级别 uint8（范围0-255）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Control onboard lighting
   * 
   * @details Lighting control command.
   *          
   *          CAN ID:
   *            0x121
   * 
   * @param mode Lighting mode uint8
   * @param brightness Brightness level uint8 (range 0-255)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult control_light(uint8_t mode, uint8_t brightness);
  
  /**
   * @brief 配置负载参数以进行动力学补偿
   * 
   * @details 通过机械臂参数查询与设置指令将负载质量映射到离散设置。
   *          
   *          CAN ID：
   *            0x477
   *          
   *          负载设置：
   *          - 0x00：无负载（≤0 kg）
   *          - 0x01：轻负载（≤0.5 kg）
   *          - 0x02：重负载（>0.5 kg）
   *          
   *          注意：重心参数目前保留供将来使用。
   * 
   * @param mass_kg 负载质量，单位 kg
   * @param center_of_gravity_m 重心坐标，单位米 [x, y, z]（保留参数）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Configure load parameters for dynamics compensation
   * 
   * @details Maps load mass to discrete settings via ARM_PARAM_ENQUIRY_AND_CONFIG command.
   *          
   *          CAN ID:
   *            0x477
   *          
   *          Load settings:
   *          - 0x00: No load (≤0 kg)
   *          - 0x01: Light load (≤0.5 kg)
   *          - 0x02: Heavy load (>0.5 kg)
   *          
   *          Note: center_of_gravity parameter is currently reserved for future use.
   * 
   * @param mass_kg Load mass in kg
   * @param center_of_gravity_m Center of gravity in meters [x, y, z] (reserved parameter)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult set_load_parameters(double mass_kg, const std::array<double, 3>& center_of_gravity_m);
  
  /**
   * @brief 将控制器恢复到默认参数
   * 
   * @details 通过机械臂参数查询与设置指令发送参数恢复命令。
   *          将所有可配置参数重置为出厂默认值。
   *          
   *          CAN ID：
   *            0x477（Byte1=0x02）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Restore controller to default parameters
   * 
   * @details Sends parameter restore command via ARM_PARAM_ENQUIRY_AND_CONFIG command.
   *          Resets all configurable parameters to factory defaults.
   *          
   *          CAN ID:
   *            0x477 (Byte1=0x02)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult restore_default_parameters();
  
  /**
   * @brief 配置关节零位
   * 
   * @details 通过关节设置指令发送关节零位设置命令。
   *          将电机 N 的当前位置设置为零点。
   *          
   *          CAN ID：
   *            0x475
   *          
   *          反馈通过 CAN ID 0x476（设置指令应答反馈）：
   *          - Byte 0：0x75（应答0x475）
   *          - Byte 1：0x01表示成功，0x00表示失败
   * 
   * @param motor_num 电机编号（1-6）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Configure joint zero position
   * 
   * @details Sends joint zero-setting command via ARM_JOINT_CONFIG command.
   *          Sets the current position of motor N as the zero point.
   *          
   *          CAN ID:
   *            0x475
   *          
   *          Response via CAN ID 0x476 (set instruction response feedback):
   *          - Byte 0: 0x75 (response to 0x475)
   *          - Byte 1: 0x01 if successful, 0x00 if failed
   * 
   * @param motor_num Motor number (1-6)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult set_joint_zero(uint8_t motor_num);
  
  /**
   * @brief 配置夹爪零位并清除错误
   * 
   * @details 通过夹爪控制指令发送夹爪零位设置命令（set_zero = 0xAE）。
   *          将当前夹爪位置设置为零点并清除任何错误。
   *          
   *          CAN ID：
   *            0x159
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Configure gripper zero position and clear errors
   * 
   * @details Sends gripper zero-setting command (set_zero = 0xAE) via ARM_GRIPPER_CTRL command.
   *          Sets the current gripper position as zero and clears any errors.
   *          
   *          CAN ID:
   *            0x159
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult set_gripper_zero();

  //============================================================================
  // Query Commands | 查询命令
  //============================================================================

  /**
   * @brief 请求固件版本信息
   * 
   * @details 通过固件读取指令发送固件版本查询。
   *          响应为固件版本字符串（通常为 8 字节，以 "S-V" 开头）。
   *          使用 get_cached_firmware_version() 检索缓存的版本字符串。
   *          
   *          CAN ID：
   *            0x4AF
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Request firmware version information
   * 
   * @details Sends firmware version query via ARM_FIRMWARE_READ command.
   *          Response is firmware version string (typically 8 bytes starting with "S-V").
   *          Use get_cached_firmware_version() to retrieve the cached version string.
   *          
   *          CAN ID:
   *            0x4AF
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult request_firmware_version();
  
  /**
   * @brief 请求电机速度限制信息
   * 
   * @details 查询电机角度/最大速度限制。
   *          
   *          CAN ID：
   *            0x472（Byte1=0x01）
   *          
   *          反馈通过 CAN ID 0x473（反馈当前电机角度限制/最大速度）
   * 
   * @param motor_num 电机编号（0x00表示查询所有电机）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Request motor speed limit information
   * 
   * @details Queries motor angle/max speed limit.
   *          
   *          CAN ID:
   *            0x472 (Byte1=0x01)
   *          
   *          Response via CAN ID 0x473 (feedback current motor angle limit/max speed)
   * 
   * @param motor_num Motor number (0x00 for querying all motors)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult search_motor_max_speed(uint8_t motor_num = 0x00);
  
  /**
   * @brief 请求电机加速度限制
   * 
   * @details 查询电机最大加速度限制。
   *          
   *          CAN ID：
   *            0x472（Byte1=0x02）
   *          
   *          反馈通过 CAN ID 0x47C（反馈当前电机最大加速度限制）
   * 
   * @param motor_num 电机编号（0x00表示查询所有电机）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Request motor acceleration limits
   * 
   * @details Queries motor max acceleration limit.
   *          
   *          CAN ID:
   *            0x472 (Byte1=0x02)
   *          
   *          Response via CAN ID 0x47C (feedback current motor max acceleration limit)
   * 
   * @param motor_num Motor number (0x00 for querying all motors)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult search_motor_max_acceleration(uint8_t motor_num = 0x00);
  
  /**
   * @brief 查询所有关节的角度/速度限制
   * 
   * @details 按顺序查询每个关节的最大速度限制。
   *          为电机 1-6 发送查询电机角度/最大速度限制指令。
   *          使用 get_all_motor_limits() 检索聚合结果。
   *          
   *          CAN ID：
   *            0x472（Byte1=0x01）
   *          
   *          反馈通过 CAN ID 0x473（反馈当前电机最大角度限制、最小角度限制、最大关节速度）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Query all joints for angle/speed limits
   * 
   * @details Queries each joint sequentially for maximum speed limits.
   *          Sends query motor angle/max speed limit command for motors 1-6.
   *          Use get_all_motor_limits() to retrieve aggregated results.
   *          
   *          CAN ID:
   *            0x472 (Byte1=0x01)
   *          
   *          Response via CAN ID 0x473 (feedback current motor max angle limit, min angle limit, max joint speed)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult search_all_motor_max_angle_speed();
  
  /**
   * @brief 查询所有关节的加速度限制
   * 
   * @details 按顺序查询每个关节的最大加速度限制。
   *          为电机 1-6 发送查询电机最大加速度限制指令。
   *          使用 get_all_motor_acceleration_limits() 检索聚合结果。
   *          
   *          CAN ID：
   *            0x472（Byte1=0x02）
   *          
   *          反馈通过 CAN ID 0x47C（反馈当前电机最大加速度限制）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Query all joints for acceleration limits
   * 
   * @details Queries each joint sequentially for maximum acceleration limits.
   *          Sends query motor max acceleration limit command for motors 1-6.
   *          Use get_all_motor_acceleration_limits() to retrieve aggregated results.
   *          
   *          CAN ID:
   *            0x472 (Byte1=0x02)
   *          
   *          Response via CAN ID 0x47C (feedback current motor max acceleration limit)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult search_all_motor_max_acc_limit();
  
  /**
   * @brief 请求末端执行器速度/加速度限制
   * 
   * @details 通过机械臂参数查询与设置指令发送参数查询（param_enquiry = 0x01）。
   *          使用 get_cached_end_velocity_limits() 检索缓存的限制。
   *          
   *          CAN ID：
   *            0x477（Byte0=0x01）
   *          
   *          反馈通过 CAN ID 0x478（反馈当前末端速度/加速度参数）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Request end-effector velocity/acceleration limits
   * 
   * @details Sends parameter enquiry (param_enquiry = 0x01) via ARM_PARAM_ENQUIRY_AND_CONFIG command.
   *          Use get_cached_end_velocity_limits() to retrieve cached limits.
   *          
   *          CAN ID:
   *            0x477 (Byte0=0x01)
   *          
   *          Response via CAN ID 0x478 (feedback current end velocity/acceleration parameters)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult search_end_velocity_limits();
  
  /**
   * @brief 通用机械臂参数查询和配置（低层接口）
   * 
   * @details 发送带打包参数的机械臂配置/查询命令（通用多功能接口）。
   *          打包命令包含5个控制字节：
   *          - param_enquiry：参数查询码（8位）
   *          - param_setting：参数设置码（8位）
   *          - data_feedback：数据反馈标志（8位）
   *          - end_load_effective：末端负载有效标志（8位）
   *          - set_end_load：末端负载设置码（8位）
   *          
   *          CAN ID：
   *            0x477
   *          
   *          参数查询码（param_enquiry）功能：
   *          - 0x00：无查询
   *          - 0x01：查询末端速度/加速度限制 → 响应 CAN ID 0x478
   *          - 0x02：查询碰撞保护等级 → 响应 CAN ID 0x47B
   *          - 0x04：查询夹爪/示教器参数 → 响应 CAN ID 0x47E
   *          
   *          参数设置码（param_setting）功能：
   *          - 0x00：无设置
   *          - 0x02：恢复默认参数
   *          
   *          数据反馈标志（data_feedback）：
   *          - 0x00：不需要反馈0x48x数据
   *          - 0x01-0xFF：需要反馈特定数据（协议保留）
   *          
   *          末端负载有效标志（end_load_effective）：
   *          - 0x00：不应用负载设置
   *          - 0xAE：应用 set_end_load 中的负载设置
   *          
   *          末端负载码（set_end_load）：
   *          - 0x00：无负载（≤0 kg）
   *          - 0x01：轻负载（≤0.5 kg）
   *          - 0x02：重负载（>0.5 kg）
   *          - 0x03：默认/不改变当前设置
   * 
   * @param param_enquiry 参数查询码 uint8（默认0x00）
   * @param param_setting 参数设置码 uint8（默认0x00）
   * @param data_feedback 数据反馈标志 uint8（默认0x00）
   * @param end_load_effective 末端负载有效标志 uint8（0xAE以应用，默认0x00）
   * @param set_end_load 末端负载设置码 uint8（默认0x03）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Generic arm parameter enquiry and configuration (low-level interface)
   * 
   * @details Sends arm configuration/query command with packed parameters (general multi-function interface).
   *          The packed command contains 5 control bytes:
   *          - param_enquiry: Parameter enquiry code (8-bit)
   *          - param_setting: Parameter setting code (8-bit)
   *          - data_feedback: Data feedback flag (8-bit)
   *          - end_load_effective: End load effective flag (8-bit)
   *          - set_end_load: End load setting code (8-bit)
   *          
   *          CAN ID:
   *            0x477
   *          
   *          Parameter enquiry code (param_enquiry) functions:
   *          - 0x00: No query
   *          - 0x01: Query end velocity/acceleration limits → Response CAN ID 0x478
   *          - 0x02: Query collision protection level → Response CAN ID 0x47B
   *          - 0x04: Query gripper/teaching pendant parameters → Response CAN ID 0x47E
   *          
   *          Parameter setting code (param_setting) functions:
   *          - 0x00: No setting
   *          - 0x02: Restore default parameters
   *          
   *          Data feedback flag (data_feedback):
   *          - 0x00: No 0x48x data feedback needed
   *          - 0x01-0xFF: Specific data feedback required (protocol reserved)
   *          
   *          End load effective flag (end_load_effective):
   *          - 0x00: Do not apply load setting
   *          - 0xAE: Apply load setting from set_end_load
   *          
   *          End load code (set_end_load):
   *          - 0x00: No load (≤0 kg)
   *          - 0x01: Light load (≤0.5 kg)
   *          - 0x02: Heavy load (>0.5 kg)
   *          - 0x03: Default/unchanged current setting
   * 
   * @param param_enquiry Parameter enquiry code uint8 (default 0x00)
   * @param param_setting Parameter setting code uint8 (default 0x00)
   * @param data_feedback Data feedback flag uint8 (default 0x00)
   * @param end_load_effective End load effective flag uint8 (0xAE to apply, default 0x00)
   * @param set_end_load End load setting code uint8 (default 0x03)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult arm_param_enquiry_and_config(uint8_t param_enquiry = 0x00, uint8_t param_setting = 0x00,
                                             uint8_t data_feedback = 0x00,
                                             uint8_t end_load_effective = 0x00, uint8_t set_end_load = 0x03);

  //============================================================================
  // Limit Configuration | 限制配置
  //============================================================================

  /**
   * @brief 设置特定关节的电机速度限制
   * 
   * @details 通过电机角度限制/最大速度设置指令配置最大关节速度。
   *          协议使用 0.001 rad/s 单位（与 Python SDK 匹配）。
   *          有效范围：[0, 3000] → 0-3.0 rad/s
   *          
   *          CAN ID：
   *            0x474
   * 
   * @param motor_num 电机编号（1-6）
   * @param max_speed 最大速度，单位 0.001 rad/s，范围 [0, 3000]（对应0-3 rad/s）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Set motor speed limit for a specific joint
   * 
   * @details Configures maximum joint speed via ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET command.
   *          Protocol uses 0.001 rad/s units (matching Python SDK).
   *          Valid range: [0, 3000] → 0-3.0 rad/s
   *          
   *          CAN ID:
   *            0x474
   * 
   * @param motor_num Motor number (1-6)
   * @param max_speed Maximum speed in 0.001 rad/s, range [0, 3000] (corresponds to 0-3 rad/s)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult set_motor_speed_limit(uint8_t motor_num, int32_t max_speed);
  
  /**
   * @brief 设置特定关节的电机加速度限制
   * 
   * @details 通过关节设置指令配置最大关节加速度。
   *          协议使用 0.01 rad/s² 单位（与 Python SDK 匹配）。
   *          有效范围：[0, 500] → 0-5.0 rad/s²
   *          
   *          CAN ID：
   *            0x475
   * 
   * @param motor_num 电机编号（1-6）
   * @param max_acc 最大加速度，单位 0.01 rad/s²，范围 [0, 500]（对应0-5 rad/s²）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Set motor acceleration limit for a specific joint
   * 
   * @details Configures maximum joint acceleration via ARM_JOINT_CONFIG command.
   *          Protocol uses 0.01 rad/s² units (matching Python SDK).
   *          Valid range: [0, 500] → 0-5.0 rad/s²
   *          
   *          CAN ID:
   *            0x475
   * 
   * @param motor_num Motor number (1-6)
   * @param max_acc Maximum acceleration in 0.01 rad/s², range [0, 500] (corresponds to 0-5 rad/s²)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult set_motor_acceleration(uint8_t motor_num, int32_t max_acc);
  
  /**
   * @brief 设置末端执行器速度/加速度限制
   * 
   * @details 通过末端速度/加速度参数设置指令配置末端执行器运动限制。
   *          所有参数使用 0.001 单位（与 Python SDK 匹配）：
   *          - 线速度：0.001 m/s
   *          - 角速度：0.001 rad/s
   *          - 线加速度：0.001 m/s²
   *          - 角加速度：0.001 rad/s²
   *          
   *          有效范围：所有参数均为 [0, 0x7FFF]
   *          
   *          CAN ID：
   *            0x479
   * 
   * @param max_linear_vel 最大线速度，单位 0.001 m/s
   * @param max_angular_vel 最大角速度，单位 0.001 rad/s
   * @param max_linear_acc 最大线加速度，单位 0.001 m/s²
   * @param max_angular_acc 最大角加速度，单位 0.001 rad/s²
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Set end-effector velocity/acceleration limits
   * 
   * @details Configures end-effector motion limits via ARM_END_VEL_ACC_PARAM_CONFIG command.
   *          All parameters use 0.001 units (matching Python SDK):
   *          - Linear velocity: 0.001 m/s
   *          - Angular velocity: 0.001 rad/s
   *          - Linear acceleration: 0.001 m/s²
   *          - Angular acceleration: 0.001 rad/s²
   *          
   *          Valid range: [0, 0x7FFF] for all parameters
   *          
   *          CAN ID:
   *            0x479
   * 
   * @param max_linear_vel Maximum linear velocity in 0.001 m/s
   * @param max_angular_vel Maximum angular velocity in 0.001 rad/s
   * @param max_linear_acc Maximum linear acceleration in 0.001 m/s²
   * @param max_angular_acc Maximum angular acceleration in 0.001 rad/s²
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult set_end_velocity_limits(int32_t max_linear_vel,
                                        int32_t max_angular_vel,
                                        int32_t max_linear_acc,
                                        int32_t max_angular_acc);
  
  /**
   * @brief 使用 0.1 度单位设置关节角度限制
   * 
   * @details 通过电机角度限制/最大速度设置指令配置关节角度范围。
   *          协议使用 0.1° 单位（与 Python SDK 匹配）。
   *          有效范围：[-3600, 3600] → ±360.0°
   *          
   *          CAN ID：
   *            0x474
   * 
   * @param motor_num 电机编号（1-6）
   * @param min_deci_deg 最小角度，单位 0.1 度
   * @param max_deci_deg 最大角度，单位 0.1 度
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @example set_joint_angle_limits(1, -1800, 1800) 设置电机1的角度限制为±180.0°
   * 
   * @brief Set joint angle limits using 0.1 degree units
   * 
   * @details Configures joint angle range via ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET command.
   *          Protocol uses 0.1° units (matching Python SDK).
   *          Valid range: [-3600, 3600] → ±360.0°
   *          
   *          CAN ID:
   *            0x474
   * 
   * @param motor_num Motor number (1-6)
   * @param min_deci_deg Minimum angle in 0.1 deg
   * @param max_deci_deg Maximum angle in 0.1 deg
   * 
   * @return CommandResult Command execution result indicating success or failure
   * 
   * @example set_joint_angle_limits(1, -1800, 1800) sets ±180.0° limits for motor 1
   */
  CommandResult set_joint_angle_limits(uint8_t motor_num, int16_t min_deci_deg, int16_t max_deci_deg);
  
  /**
   * @brief 从向量配置关节限制
   * 
   * @details 批量配置助手，为所有关节设置速度和加速度限制。
   *          内部为每个关节调用 set_motor_speed_limit() 和 set_motor_acceleration()。
   *          
   *          CAN ID：
   *            0x474（速度限制）
   *            0x475（加速度限制）
   * 
   * @param max_speed 每个关节的最大速度，单位 0.001 rad/s
   * @param max_acc 每个关节的最大加速度，单位 0.01 rad/s²
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Configure joint limits from vectors
   * 
   * @details Batch configuration helper that sets speed and acceleration limits for all joints.
   *          Internally calls set_motor_speed_limit() and set_motor_acceleration() for each joint.
   *          
   *          CAN ID:
   *            0x474 (speed limits)
   *            0x475 (acceleration limits)
   * 
   * @param max_speed Maximum speed per joint in 0.001 rad/s
   * @param max_acc Maximum acceleration per joint in 0.01 rad/s²
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult joint_config(const std::vector<int32_t>& max_speed,
                             const std::vector<int32_t>& max_acc);
  
  /**
   * @brief 完整关节配置（镜像 Python JointConfig）
   * 
   * @details 通过关节设置指令进行多功能关节配置。
   *          
   *          支持功能：
   *          - 设置零位（set_zero = 0xAE）
   *          - 配置加速度限制（acc_param_is_effective = 0xAE）
   *          - 清除错误（clear_err = 0xAE）
   *          
   *          字节映射：
   *          - Byte 0：motor_num（1-7，其中7表示所有电机）
   *          - Byte 1：set_zero（0xAE表示设置当前位置为零位）
   *          - Byte 2：acc_param_is_effective（0xAE表示应用加速度参数）
   *          - Byte 3-4：max_joint_acc（uint16，单位 0.01 rad/s²）
   *          - Byte 5：clear_err（0xAE表示清除错误）
   *          
   *          CAN ID：
   *            0x475
   * 
   * @param motor_num 电机编号（1-7，其中7表示所有电机）
   * @param set_zero 设置零位标志（0xAE启用，默认0x00）
   * @param acc_param_is_effective 加速度参数有效标志（0xAE启用，默认0x00）
   * @param max_joint_acc 最大关节加速度，单位 0.01 rad/s²（范围[0, 500]，默认500）
   * @param clear_err 清除错误标志（0xAE启用，默认0x00）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Full joint configuration (mirroring Python JointConfig)
   * 
   * @details Multi-function joint configuration via ARM_JOINT_CONFIG command.
   *          
   *          Supported functions:
   *          - Setting zero position (set_zero = 0xAE)
   *          - Configuring acceleration limits (acc_param_is_effective = 0xAE)
   *          - Clearing errors (clear_err = 0xAE)
   *          
   *          Byte mapping:
   *          - Byte 0: motor_num (1-7, where 7 means all motors)
   *          - Byte 1: set_zero (0xAE to set current position as zero)
   *          - Byte 2: acc_param_is_effective (0xAE to apply acceleration parameter)
   *          - Byte 3-4: max_joint_acc (uint16, in 0.01 rad/s²)
   *          - Byte 5: clear_err (0xAE to clear errors)
   *          
   *          CAN ID:
   *            0x475
   * 
   * @param motor_num Motor number (1-7, where 7 means all motors)
   * @param set_zero Set zero position flag (0xAE to enable, default 0x00)
   * @param acc_param_is_effective Acceleration parameter effective flag (0xAE to enable, default 0x00)
   * @param max_joint_acc Maximum joint acceleration in 0.01 rad/s² (range [0, 500], default 500)
   * @param clear_err Clear error flag (0xAE to enable, default 0x00)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult joint_config(uint8_t motor_num, uint8_t set_zero = 0x00, 
                             uint8_t acc_param_is_effective = 0x00, 
                             uint16_t max_joint_acc = 500,
                             uint8_t clear_err = 0x00);
  
  /**
   * @brief 使用向量助手配置关节最大加速度
   * 
   * @details 批量配置助手，为所有关节设置加速度限制。
   *          内部为每个关节调用 set_motor_acceleration()。
   *          
   *          CAN ID：
   *            0x475
   * 
   * @param max_acc 每个关节的最大加速度，单位 0.01 rad/s²
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Configure joint max acceleration using vector helper
   * 
   * @details Batch configuration helper that sets acceleration limits for all joints.
   *          Internally calls set_motor_acceleration() for each joint.
   *          
   *          CAN ID:
   *            0x475
   * 
   * @param max_acc Maximum acceleration per joint in 0.01 rad/s²
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult joint_max_acc_config(const std::vector<int32_t>& max_acc);
  
  /**
   * @brief 设置电机最大速度（镜像 Python 的 MotorMaxSpdSet）
   * 
   * @details 批量配置助手，为所有关节设置速度限制。
   *          内部为每个关节调用 set_motor_speed_limit()。
   *          
   *          CAN ID：
   *            0x474
   * 
   * @param max_speed 每个关节的最大速度，单位 0.001 rad/s
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Set motor max speed (mirroring Python's MotorMaxSpdSet)
   * 
   * @details Batch configuration helper that sets speed limits for all joints.
   *          Internally calls set_motor_speed_limit() for each joint.
   *          
   *          CAN ID:
   *            0x474
   * 
   * @param max_speed Maximum speed per joint in 0.001 rad/s
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult motor_max_speed_set(const std::vector<int32_t>& max_speed);

  //============================================================================
  // Teaching Pendant Configuration | 示教器配置
  //============================================================================

  /**
   * @brief 配置夹爪示教器参数
   * 
   * @details 通过夹爪示教器参数配置指令配置夹爪/示教器参数。
   *          基于 V1.5-2 版本后。
   *          固件可能会在 CAN ID 0x47D 上回显此配置。
   *          查询响应通过 CAN ID 0x477 Byte 0 = 0x04 → 0x47E
   *          
   *          CAN ID：
   *            0x47D
   * 
   * @param teaching_range_per 示教器行程系数设置，仅适用于设置主从臂的主臂，用于放大控制行程给从臂，范围[100~200]
   * @param max_range_config 夹爪/示教器最大控制行程限制值设置，(0,70,100)
   *                         - 0：无效值
   *                         - 70：小夹爪 70mm
   *                         - 100：大夹爪 100mm
   * @param teaching_friction 示教器摩擦系数设置，范围[1, 10]
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Configure gripper teaching pendant parameters
   * 
   * @details Configures gripper/teaching pendant parameters via ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG command.
   *          Based on V1.5-2 version and later.
   *          The firmware may echo this configuration back on CAN ID 0x47D.
   *          Response query via CAN ID 0x477 Byte 0 = 0x04 → 0x47E
   *          
   *          CAN ID:
   *            0x47D
   * 
   * @param teaching_range_per Teaching pendant travel coefficient, only for master arm in master-slave setup, range [100~200]
   * @param max_range_config Gripper/teaching pendant maximum control travel limit, (0,70,100)
   *                         - 0: Invalid value
   *                         - 70: Small gripper 70mm
   *                         - 100: Large gripper 100mm
   * @param teaching_friction Teaching pendant friction coefficient setting, range [1, 10]
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult configure_gripper_teaching_pendant(uint8_t teaching_range_per = 100, 
                                                    uint8_t max_range_config = 70,
                                                    uint8_t teaching_friction = 1);
  
  /**
   * @brief Get cached gripper teaching pendant feedback (CAN ID: 0x47D/0x47E)
   *        获取缓存的夹爪示教器反馈（CAN ID：0x47D/0x47E）
   * 
   * @details Retrieves cached gripper/teaching pendant parameters.
   *          检索缓存的夹爪/示教器参数。
   *          
   *          Parameters are cached from:
   *          参数缓存来自：
   *          - CAN ID 0x47D: Echo of configuration command
   *          - CAN ID 0x47E: Response to query (0x477 Byte 0 = 0x04)
   *          
   *          ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK - 夹爪示教器参数反馈
   * 
   * @return std::optional<GripperTeachingPendantParam> Cached parameters if available
   *         如果可用则返回缓存的参数
   */
  std::optional<GripperTeachingPendantParam> get_gripper_teaching_pendant_param() const;

  /**
   * @brief 请求主臂移动到原点位置
   * 
   * @details 命令主臂返回原点/零位。
   *          
   *          CAN ID：
   *            0x191
   *          
   *          归零模式：
   *          - 0：模式0（Byte 0=0x01, Byte 1=0x00, Byte 2=0x00）
   *          - 1：模式1（Byte 0=0x01, Byte 1=0x01, Byte 2=0x01）
   *          - 2：模式2（Byte 0=0x01, Byte 1=0x00, Byte 2=0x01）
   * 
   * @param mode 归零模式（0-2，默认0x00）
   * 
   * @return CommandResult 命令执行结果，表示成功或失败
   * 
   * @brief Request master arm to move to home position
   * 
   * @details Commands the master arm to return to home/zero position.
   *          
   *          CAN ID:
   *            0x191
   *          
   *          Homing modes:
   *          - 0: Mode 0 (Byte 0=0x01, Byte 1=0x00, Byte 2=0x00)
   *          - 1: Mode 1 (Byte 0=0x01, Byte 1=0x01, Byte 2=0x01)
   *          - 2: Mode 2 (Byte 0=0x01, Byte 1=0x00, Byte 2=0x01)
   * 
   * @param mode Homing mode (0-2, default 0x00)
   * 
   * @return CommandResult Command execution result indicating success or failure
   */
  CommandResult request_master_arm_move_home(uint8_t mode = 0x00);

  //============================================================================
  // Connection Management | 连接管理
  //============================================================================

  /**
   * @brief Start background CAN reception with configurable initialization
   *        启动后台 CAN 接收，具有可配置的初始化
   * 
   * @param timeout Initial connection timeout | 初始连接超时
   * @param start_thread Whether to start receiver thread (default: true) | 是否启动接收线程（默认：true）
   * @param piper_init_flag Whether to execute piper_init() (default: true) | 是否执行 piper_init()（默认：true）
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult connect_port(std::chrono::milliseconds timeout = std::chrono::milliseconds(10),
                             bool start_thread = true,
                             bool piper_init_flag = true);
  
  /**
   * @brief Stop background reception thread
   *        停止后台接收线程
   */
  void disconnect_port();

  //============================================================================
  // State Accessors | 状态访问器
  //============================================================================

  /**
   * @brief Get comprehensive arm status with calculated frequencies
   *        获取带计算频率的综合机械臂状态
   * 
   * @return std::optional<ArmFeedback> Current arm state if available
   *         如果可用则返回当前机械臂状态
   */
  std::optional<ArmFeedback> get_arm_status() const;
  
  /**
   * @brief Get joint state with timestamp and frequency
   *        获取带时间戳和频率的关节状态
   * 
   * @return std::optional<TimedJointState> Joint state if available
   *         如果可用则返回关节状态
   */
  std::optional<TimedJointState> get_arm_joint_messages() const;
  
  /**
   * @brief Get gripper state with timestamp and frequency
   *        获取带时间戳和频率的夹爪状态
   * 
   * @return std::optional<TimedGripperState> Gripper state if available
   *         如果可用则返回夹爪状态
   */
  std::optional<TimedGripperState> get_arm_gripper_messages() const;
  
  /**
   * @brief Get end-effector pose with timestamp and frequency
   *        获取带时间戳和频率的末端执行器位姿
   * 
   * @return std::optional<TimedPoseState> End-effector pose if available
   *         如果可用则返回末端执行器位姿
   */
  std::optional<TimedPoseState> get_arm_end_pose_messages() const;
  
  /**
   * @brief Get motor states with timestamp and frequency
   *        获取带时间戳和频率的电机状态
   * 
   * @return std::optional<TimedMotorState> Motor states if available
   *         如果可用则返回电机状态
   */
  std::optional<TimedMotorState> get_motor_states() const;
  
  /**
   * @brief Get driver states with timestamp and frequency
   *        获取带时间戳和频率的驱动器状态
   * 
   * @return std::optional<TimedDriverState> Driver states if available
   *         如果可用则返回驱动器状态
   */
  std::optional<TimedDriverState> get_driver_states() const;

  //============================================================================
  // Convenience Query Helpers | 便捷查询助手
  //============================================================================

  /**
   * @brief Query and cache firmware version (CAN ID: 0x4AF)
   *        查询并缓存固件版本（CAN ID：0x4AF）
   * 
   * @details Convenience wrapper for request_firmware_version().
   *          便捷包装器，用于 request_firmware_version()。
   *          
   *          Sends ARM_FIRMWARE_READ command and caches the version string.
   *          发送固件读取指令并缓存版本字符串。
   *          
   *          Use get_cached_firmware_version() to retrieve the result.
   *          使用 get_cached_firmware_version() 检索结果。
   * 
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult get_piper_firmware_version();
  
  /**
   * @brief Query all motor angle/speed limits (CAN ID: 0x472)
   *        查询所有电机角度/速度限制（CAN ID：0x472）
   * 
   * @details Convenience wrapper for search_all_motor_max_angle_speed().
   *          便捷包装器，用于 search_all_motor_max_angle_speed()。
   *          
   *          Queries all 6 motors for angle/speed limits.
   *          查询所有 6 个电机的角度/速度限制。
   *          
   *          Response via CAN ID 0x473.
   *          通过 CAN ID 0x473 反馈。
   *          
   *          Use get_all_motor_limits() to retrieve aggregated results.
   *          使用 get_all_motor_limits() 检索聚合结果。
   * 
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult get_arm_motor_max_angle_speed();
  
  /**
   * @brief Query all motor acceleration limits (CAN ID: 0x472)
   *        查询所有电机加速度限制（CAN ID：0x472）
   * 
   * @details Convenience wrapper for search_all_motor_max_acc_limit().
   *          便捷包装器，用于 search_all_motor_max_acc_limit()。
   *          
   *          Queries all 6 motors for acceleration limits.
   *          查询所有 6 个电机的加速度限制。
   *          
   *          Response via CAN ID 0x47C.
   *          通过 CAN ID 0x47C 反馈。
   *          
   *          Use get_all_motor_acceleration_limits() to retrieve aggregated results.
   *          使用 get_all_motor_acceleration_limits() 检索聚合结果。
   * 
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult get_arm_motor_max_acc_limit();
  
  /**
   * @brief Query collision protection levels (CAN ID: 0x477, Byte0=0x02)
   *        查询碰撞保护等级（CAN ID：0x477，Byte0=0x02）
   * 
   * @details Convenience wrapper for arm_param_enquiry_and_config(0x02).
   *          便捷包装器，用于 arm_param_enquiry_and_config(0x02)。
   *          
   *          Queries current collision protection levels for all joints.
   *          查询所有关节的当前碰撞保护等级。
   *          
   *          Response via CAN ID 0x47B (ARM_CRASH_PROTECTION_RATING_FEEDBACK).
   *          通过 CAN ID 0x47B 反馈（碰撞防护等级反馈指令）。
   * 
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult get_crash_protection_level();
  
  /**
   * @brief Query end-effector velocity/acceleration parameters (CAN ID: 0x477, Byte0=0x01)
   *        查询末端执行器速度/加速度参数（CAN ID：0x477，Byte0=0x01）
   * 
   * @details Convenience wrapper for search_end_velocity_limits().
   *          便捷包装器，用于 search_end_velocity_limits()。
   *          
   *          Queries current end-effector velocity and acceleration limits.
   *          查询当前末端执行器速度和加速度限制。
   *          
   *          Response via CAN ID 0x478 (ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM).
   *          通过 CAN ID 0x478 反馈（反馈当前末端速度/加速度参数）。
   *          
   *          Use get_cached_end_velocity_limits() to retrieve cached limits.
   *          使用 get_cached_end_velocity_limits() 检索缓存的限制。
   * 
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult get_end_vel_acc_param();

  //============================================================================
  // Cached Data Accessors | 缓存数据访问器
  //============================================================================

  /**
   * @brief Get cached motor limits from recent feedback (CAN ID: 0x473)
   *        从最近的反馈中获取缓存的电机限制（CAN ID：0x473）
   * 
   * @details Returns the most recently received motor limit feedback.
   *          返回最近接收的电机限制反馈。
   *          
   *          Contains:
   *          包含：
   *          - motor_index: Motor number (1-6)
   *          - max_angle_limit: Maximum angle limit (0.1°)
   *          - min_angle_limit: Minimum angle limit (0.1°)
   *          - max_joint_speed: Maximum joint speed (0.001 rad/s)
   *          
   *          From ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD (0x473)
   *          来自反馈当前电机最大角度限制/最大速度 (0x473)
   * 
   * @return std::optional<MotorLimits> Cached limits if available
   *         如果可用则返回缓存的限制
   */
  std::optional<MotorLimits> get_cached_motor_limits() const;
  
  /**
   * @brief Get aggregated motor angle/speed limits for all motors (CAN ID: 0x473)
   *        获取所有电机的聚合角度/速度限制（CAN ID：0x473）
   * 
   * @details Returns aggregated limits after querying all 6 motors.
   *          返回查询所有 6 个电机后的聚合限制。
   *          
   *          Call get_arm_motor_max_angle_speed() first to populate data.
   *          首先调用 get_arm_motor_max_angle_speed() 以填充数据。
   *          
   *          Returns data only when all 6 motors have been queried.
   *          仅当查询了所有 6 个电机时才返回数据。
   * 
   * @return std::optional<AllMotorLimits> All motor limits if complete
   *         如果完整则返回所有电机限制
   */
  std::optional<AllMotorLimits> get_all_motor_limits() const;
  
  /**
   * @brief Get aggregated motor acceleration limits for all motors (CAN ID: 0x47C)
   *        获取所有电机的聚合加速度限制（CAN ID：0x47C）
   * 
   * @details Returns aggregated acceleration limits after querying all 6 motors.
   *          返回查询所有 6 个电机后的聚合加速度限制。
   *          
   *          Call get_arm_motor_max_acc_limit() first to populate data.
   *          首先调用 get_arm_motor_max_acc_limit() 以填充数据。
   *          
   *          Returns data only when all 6 motors have been queried.
   *          仅当查询了所有 6 个电机时才返回数据。
   *          
   *          From ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT (0x47C)
   *          来自反馈当前电机最大加速度限制 (0x47C)
   * 
   * @return std::optional<AllMotorLimits> All motor acceleration limits if complete
   *         如果完整则返回所有电机加速度限制
   */
  std::optional<AllMotorLimits> get_all_motor_acceleration_limits() const;
  
  /**
   * @brief Get cached end-effector velocity limits (CAN ID: 0x478)
   *        获取缓存的末端执行器速度限制（CAN ID：0x478）
   * 
   * @details Returns cached end-effector velocity and acceleration limits.
   *          返回缓存的末端执行器速度和加速度限制。
   *          
   *          Contains (all in 0.001 units):
   *          包含（所有单位为 0.001）：
   *          - max_linear_velocity: Maximum linear velocity (0.001 m/s)
   *          - max_angular_velocity: Maximum angular velocity (0.001 rad/s)
   *          - max_linear_acceleration: Maximum linear acceleration (0.001 m/s²)
   *          - max_angular_acceleration: Maximum angular acceleration (0.001 rad/s²)
   *          
   *          Call get_end_vel_acc_param() first to query current limits.
   *          首先调用 get_end_vel_acc_param() 以查询当前限制。
   *          
   *          From ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM (0x478)
   *          来自反馈当前末端速度/加速度参数 (0x478)
   * 
   * @return std::optional<EndVelocityLimits> Cached limits if available
   *         如果可用则返回缓存的限制
   */
  std::optional<EndVelocityLimits> get_cached_end_velocity_limits() const;
  
  /**
   * @brief Get cached firmware version string (CAN ID: 0x4AF)
   *        获取缓存的固件版本字符串（CAN ID：0x4AF）
   * 
   * @details Returns cached firmware version string (typically 8 bytes starting with "S-V").
   *          返回缓存的固件版本字符串（通常为 8 字节，以 "S-V" 开头）。
   *          
   *          Call get_piper_firmware_version() first to query version.
   *          首先调用 get_piper_firmware_version() 以查询版本。
   *          
   *          From ARM_FIRMWARE_READ (0x4AF)
   *          来自固件读取指令 (0x4AF)
   * 
   * @return std::optional<std::string> Firmware version if available
   *         如果可用则返回固件版本
   */
  std::optional<std::string> get_cached_firmware_version() const;
  
  /**
   * @brief Get SDK joint software limits configuration
   *        获取 SDK 关节软件限制配置
   * 
   * @return SoftwareLimitsConfig Current joint limits configuration
   *         当前关节限制配置
   */
  SoftwareLimitsConfig get_sdk_joint_limit_param() const;
  
  /**
   * @brief Get SDK gripper software range limits
   *        获取 SDK 夹爪软件范围限制
   * 
   * @return SoftwareLimitsConfig Current gripper limits configuration
   *         当前夹爪限制配置
   */
  SoftwareLimitsConfig get_sdk_gripper_range_param() const;
  
  /**
   * @brief Set SDK joint software limits
   *        设置 SDK 关节软件限制
   * 
   * @param limits New joint limits configuration | 新的关节限制配置
   */
  void set_sdk_joint_limit_param(const SoftwareLimitsConfig& limits);
  
  /**
   * @brief Set SDK gripper software range limits
   *        设置 SDK 夹爪软件范围限制
   * 
   * @param limits New gripper limits configuration | 新的夹爪限制配置
   */
  void set_sdk_gripper_range_param(const SoftwareLimitsConfig& limits);

  /**
   * @brief Read a feedback frame and update cached state
   *        读取反馈帧并更新缓存状态
   * 
   * @param timeout Maximum wait time for frame | 帧的最大等待时间
   * @return std::optional<ArmFeedback> Updated feedback if received
   *         如果接收到则返回更新的反馈
   */
  std::optional<ArmFeedback> read_feedback(std::chrono::milliseconds timeout);

  //============================================================================
  // Forward Kinematics | 正运动学
  //============================================================================

  /**
   * @brief Get forward kinematics for cached feedback or last control command
   *        获取缓存反馈或最后控制命令的正运动学
   * 
   * @details Returns poses in mm and degrees (matching Python SDK GetFK() output).
   *          返回以 mm 和度为单位的位姿（与 Python SDK GetFK() 输出匹配）。
   * 
   * @param mode FK calculation mode (feedback or control) | FK 计算模式（反馈或控制）
   * @return std::optional<std::array<LinkPose, 6>> Link poses if FK is enabled
   *         如果启用 FK 则返回连杆位姿
   */
  std::optional<std::array<LinkPose, 6>> get_forward_kinematics(FKMode mode) const;
  
  /**
   * @brief Enable automatic forward kinematics calculation
   *        启用自动正运动学计算
   */
  void enable_fk_cal();
  
  /**
   * @brief Disable automatic forward kinematics calculation
   *        禁用自动正运动学计算
   */
  void disable_fk_cal();
  
  /**
   * @brief Check if FK calculation is enabled
   *        检查是否启用 FK 计算
   * 
   * @return true if FK calculation is enabled | 如果启用 FK 计算则返回 true
   */
  bool is_cal_fk() const;

  //============================================================================
  // Status and Diagnostics | 状态和诊断
  //============================================================================

  /**
   * @brief Get enable status per joint from driver info
   *        从驱动器信息获取每个关节的使能状态
   * 
   * @return std::vector<bool> Enable status per joint | 每个关节的使能状态
   */
  std::vector<bool> get_arm_enable_status() const;
  
  /**
   * @brief Retrieve cached instruction response frame (CAN ID: 0x476)
   *        检索缓存的指令响应帧（CAN ID：0x476）
   * 
   * @details Returns response after sending a setting command (0x4XX prefix).
   *          返回发送设置指令后的应答帧（0x4XX 开头为设置指令 ID）。
   *          
   *          Response contains:
   *          应答包含：
   *          - code: instruction_index (应答指令索引，取设置指令 ID 最后一个字节)
   *            例如应答 0x471 设置指令时此位为 0x71
   *          - payload[0]: is_set_zero_successfully (零点是否设置成功)
   *            0x01: 零点成功设置 | Zero point successfully set
   *            0x00: 设置失败/未设置 | Failed to set/Not set
   * 
   * @return std::optional<InstructionResponse> Response if available
   *         如果可用则返回响应
   */
  std::optional<InstructionResponse> get_resp_instruction() const;
  
  /**
   * @brief Clear cached instruction response
   *        清除缓存的指令响应
   */
  void clear_resp_instruction();

  /**
   * @brief Enable abnormal data filtering
   *        启用异常数据过滤
   */
  void enable_filter_abnormal_data();
  
  /**
   * @brief Disable abnormal data filtering
   *        禁用异常数据过滤
   */
  void disable_filter_abnormal_data();
  
  /**
   * @brief Check if abnormal data filtering is active
   *        检查异常数据过滤是否活动
   * 
   * @return true if filtering is enabled | 如果启用过滤则返回 true
   */
  bool is_filter_abnormal_data() const;

  /**
   * @brief Quick helper to enable all joints and confirm state
   *        快速助手以使能所有关节并确认状态
   * 
   * @return true if all joints enabled successfully | 如果所有关节成功使能则返回 true
   */
  bool enable_piper();
  
  /**
   * @brief Quick helper to disable all joints and confirm state
   *        快速助手以禁用所有关节并确认状态
   * 
   * @return true if all joints disabled successfully | 如果所有关节成功禁用则返回 true
   */
  bool disable_piper();
  
  /**
   * @brief Check if CAN health monitor considers connection OK
   *        检查 CAN 健康监视器是否认为连接正常
   * 
   * @return true if connection is healthy | 如果连接健康则返回 true
   */
  bool is_ok() const;

  //============================================================================
  // Control Data Accessors | 控制数据访问器
  //============================================================================

  /**
   * @brief Get joint control feedback with timestamp and frequency
   *        获取带时间戳和频率的关节控制反馈
   * 
   * @return std::optional<TimedJointCtrl> Joint control data if available
   *         如果可用则返回关节控制数据
   */
  std::optional<TimedJointCtrl> get_arm_joint_ctrl() const;
  
  /**
   * @brief Get gripper control feedback with timestamp and frequency
   *        获取带时间戳和频率的夹爪控制反馈
   * 
   * @return std::optional<TimedGripperCtrl> Gripper control data if available
   *         如果可用则返回夹爪控制数据
   */
  std::optional<TimedGripperCtrl> get_arm_gripper_ctrl() const;
  
  /**
   * @brief Get mode control feedback with timestamp and frequency
   *        获取带时间戳和频率的模式控制反馈
   * 
   * @return std::optional<TimedModeCtrl> Mode control data if available
   *         如果可用则返回模式控制数据
   */
  std::optional<TimedModeCtrl> get_arm_mode_ctrl() const;
  
  /**
   * @brief Get motion control command 2 feedback (CAN ID 0x151)
   *        获取运动控制命令2反馈（CAN ID 0x151）
   * 
   * @return std::optional<TimedModeCtrl> Control code 151 data if available
   *         如果可用则返回控制码 151 数据
   */
  std::optional<TimedModeCtrl> get_arm_ctrl_code151() const;

  /**
   * @brief Get last joint command issued via this interface
   *        获取通过此接口发出的最后关节命令
   * 
   * @return std::optional<std::vector<int32_t>> Last joint positions in 0.001 deg
   *         最后关节位置，单位 0.001 deg
   */
  std::optional<std::vector<int32_t>> get_last_joint_command() const;
  
  /**
   * @brief Get last Cartesian command issued via this interface
   *        获取通过此接口发出的最后笛卡尔命令
   * 
   * @return std::optional<CartesianPose> Last Cartesian pose
   *         最后笛卡尔位姿
   */
  std::optional<CartesianPose> get_last_mode_command() const;
  
  /**
   * @brief Get last gripper command issued via this interface
   *        获取通过此接口发出的最后夹爪命令
   * 
   * @return std::optional<GripperCommand> Last gripper command
   *         最后夹爪命令
   */
  std::optional<GripperCommand> get_last_gripper_command() const;
  
  /**
   * @brief Get last motion control parameters (command 2 / 0x151)
   *        获取最后运动控制参数（命令2 / 0x151）
   * 
   * @return MotionControlParameters Last motion control parameters
   *         最后运动控制参数
   */
  MotionControlParameters get_last_motion_ctrl_code151() const;

  //============================================================================
  // Connection Health and Versioning | 连接健康和版本控制
  //============================================================================

  /**
   * @brief Get connection health status
   *        获取连接健康状态
   * 
   * @param stale_timeout Timeout for considering data stale | 考虑数据过期的超时时间
   * @return ConnectStatus Connection status information | 连接状态信息
   */
  ConnectStatus get_connect_status(std::chrono::milliseconds stale_timeout = std::chrono::milliseconds(500)) const;
  
  /**
   * @brief Get aggregate CAN bus FPS across message types
   *        获取跨消息类型的聚合 CAN 总线 FPS
   * 
   * @return double Average FPS | 平均 FPS
   */
  double get_can_fps() const;
  
  /**
   * @brief Get current SDK version string
   *        获取当前 SDK 版本字符串
   * 
   * @return std::string SDK version | SDK 版本
   */
  std::string get_current_sdk_version() const;
  
  /**
   * @brief Get current interface version string
   *        获取当前接口版本字符串
   * 
   * @return std::string Interface version | 接口版本
   */
  std::string get_current_interface_version() const;
  
  /**
   * @brief Get current protocol version string
   *        获取当前协议版本字符串
   * 
   * @return std::string Protocol version | 协议版本
   */
  std::string get_current_protocol_version() const;

  //============================================================================
  // CAN Bus Configuration | CAN 总线配置
  //============================================================================

  /**
   * @brief Create or update CAN bus configuration
   *        创建或更新 CAN 总线配置
   * 
   * @param can_name CAN interface name | CAN 接口名称
   * @param bustype Bus type (e.g., "socketcan", "slcan") | 总线类型（例如 "socketcan"、"slcan"）
   * @param expected_bitrate Expected CAN bitrate | 预期 CAN 位速率
   * @param judge_flag Enable bitrate verification | 启用位速率验证
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult create_can_bus(const std::string& can_name, const std::string& bustype,
                               uint32_t expected_bitrate, bool judge_flag);
  
  /**
   * @brief Get configured CAN bus info string
   *        获取配置的 CAN 总线信息字符串
   * 
   * @return std::string CAN bus information | CAN 总线信息
   */
  std::string get_can_bus() const;
  
  /**
   * @brief Get configured CAN interface name
   *        获取配置的 CAN 接口名称
   * 
   * @return std::string CAN interface name | CAN 接口名称
   */
  std::string get_can_name() const;


 private:
  //============================================================================
  // Private Helper Methods | 私有助手方法
  //============================================================================

  CommandResult send_motion_control_profile(MotionMode mode, const Limits& limits);
  CommandResult send_motion_command(const MotionCommand& command);
  CommandResult send_can_frame(const CanFrame& frame);
  CommandResult validate_joint_positions(const std::vector<int32_t>& positions_mdeg) const;
  CommandResult validate_gripper_command(const GripperCommand& command) const;
  int32_t apply_joint_sdk_limits(size_t joint_index, int32_t value) const;
  int32_t apply_gripper_sdk_limits(int32_t value) const;
  bool is_joint_feedback_valid(int32_t value) const;
  bool is_gripper_feedback_valid(int32_t value) const;
  bool is_end_pose_component_valid(int32_t value, bool is_orientation) const;
  void update_can_health_locked();
  double total_can_fps_unlocked() const;
  MitJointCommand pack_mit_command(double pos_ref, double vel_ref, double kp, double kd,
                                   double torque_ref) const;
  CommandResult send_motor_enable(uint8_t motor_num, uint8_t enable_flag);
  CommandResult search_motor_limit(uint8_t motor_num, uint8_t search_content);
  CommandResult piper_init();
  std::optional<ArmFeedback> process_feedback_frame(const CanFrame& frame);
  double fps_for_ids(const std::vector<uint32_t>& ids) const;
  void log(LogLevel level, const std::string& message) const;

  //============================================================================
  // Private Member Variables | 私有成员变量
  //============================================================================

  std::shared_ptr<CanTransport> transport_;       ///< CAN transport implementation | CAN 传输实现
  PiperConfig config_;                            ///< Configuration parameters | 配置参数
  mutable std::mutex mutex_;                      ///< Mutex for thread-safe access | 线程安全访问的互斥锁
  PiperForwardKinematics fk_;                     ///< Forward kinematics calculator | 正运动学计算器
  ArmFeedback last_feedback_{};                   ///< Last received feedback | 最后接收的反馈
  bool fk_enabled_{true};                         ///< FK calculation enabled flag | FK 计算启用标志
  mutable std::optional<std::array<LinkPose, 6>> cached_fk_feedback_{}; ///< Cached FK from feedback | 来自反馈的缓存 FK
  mutable std::optional<std::array<LinkPose, 6>> cached_fk_control_{};  ///< Cached FK from control | 来自控制的缓存 FK
  std::optional<MotorLimits> cached_motor_limits_{};                   ///< Cached motor limits | 缓存的电机限制
  AllMotorLimits all_motor_limits_{};                                   ///< All motor angle/speed limits | 所有电机角度/速度限制
  AllMotorLimits all_motor_accel_limits_{};                            ///< All motor acceleration limits | 所有电机加速度限制
  std::optional<EndVelocityLimits> cached_end_velocity_limits_{};      ///< Cached end velocity limits | 缓存的末端速度限制
  std::optional<std::string> cached_firmware_version_{};               ///< Cached firmware version | 缓存的固件版本
  std::optional<InstructionResponse> instruction_response_{};          ///< Cached instruction response | 缓存的指令响应
  std::optional<GripperTeachingPendantParam> gripper_teaching_param_{}; ///< Cached teaching param | 缓存的示教参数
  MotionControlParameters motion_ctrl_params_{};                       ///< Motion control parameters | 运动控制参数
  JointCtrlState joint_ctrl_feedback_{};                               ///< Joint control feedback | 关节控制反馈
  GripperCtrlState gripper_ctrl_feedback_{};                           ///< Gripper control feedback | 夹爪控制反馈
  MotionControlParameters mode_ctrl_feedback_{};                       ///< Mode control feedback | 模式控制反馈
  bool joint_ctrl_feedback_valid_{false};                              ///< Joint control valid flag | 关节控制有效标志
  bool gripper_ctrl_feedback_valid_{false};                            ///< Gripper control valid flag | 夹爪控制有效标志
  bool mode_ctrl_feedback_valid_{false};                               ///< Mode control valid flag | 模式控制有效标志
  std::chrono::steady_clock::time_point joint_ctrl_timestamp_{};       ///< Joint control timestamp | 关节控制时间戳
  std::chrono::steady_clock::time_point gripper_ctrl_timestamp_{};     ///< Gripper control timestamp | 夹爪控制时间戳
  std::chrono::steady_clock::time_point mode_ctrl_timestamp_{};        ///< Mode control timestamp | 模式控制时间戳
  MotionControl1Cache motion_ctrl1_cache_{};                           ///< Motion control 1 cache | 运动控制 1 缓存
  std::optional<std::vector<int32_t>> last_joint_command_{};           ///< Last joint command | 最后关节命令
  std::optional<CartesianPose> last_cartesian_command_{};              ///< Last Cartesian command | 最后笛卡尔命令
  std::optional<GripperCommand> last_gripper_command_{};               ///< Last gripper command | 最后夹爪命令
  std::unordered_map<uint32_t, FpsCounter> fps_counters_;              ///< FPS counters per CAN ID | 每个 CAN ID 的 FPS 计数器
  std::unique_ptr<Logger> logger_;                                     ///< Logger instance | 日志记录器实例
  mutable std::atomic<bool> running_{false};                           ///< Receiver thread running flag | 接收线程运行标志
  std::thread receiver_thread_;                                        ///< Background receiver thread | 后台接收线程
  std::chrono::milliseconds receive_timeout_{10};                      ///< Reception timeout | 接收超时
  bool filter_abnormal_data_{false};                                   ///< Abnormal data filter flag | 异常数据过滤标志
  bool is_ok_{true};                                                   ///< Connection health flag | 连接健康标志
  std::deque<double> can_fps_window_;                                  ///< CAN FPS sliding window | CAN FPS 滑动窗口
  std::string can_name_{};                                             ///< CAN interface name | CAN 接口名称
  std::string can_bus_type_{};                                         ///< CAN bus type | CAN 总线类型
  uint32_t expected_bitrate_{0};                                       ///< Expected bitrate | 预期位速率
  bool judge_flag_{false};                                             ///< Bitrate judge flag | 位速率判断标志
  bool bus_created_{false};                                            ///< Bus created flag | 总线创建标志
  bool piper_initialized_{false};                                      ///< Piper initialized flag | Piper 初始化标志
  static constexpr size_t kCanHealthWindowSize = 5;                    ///< CAN health window size | CAN 健康窗口大小
};

}  // namespace piper_sdk
