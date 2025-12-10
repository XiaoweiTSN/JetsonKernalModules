/**
 * @file kinematics.hpp
 * @brief Forward kinematics calculation for Piper robot
 *        Piper 机器人正运动学计算
 * 
 * @details Implements forward kinematics using DH (Denavit-Hartenberg) parameters.
 *          Calculates end-effector and intermediate link poses from joint angles.
 *          使用 DH（Denavit-Hartenberg）参数实现正运动学。
 *          从关节角度计算末端执行器和中间连杆位姿。
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

#include <array>

namespace piper_sdk {

/**
 * @class PiperForwardKinematics
 * @brief Forward kinematics calculator for Piper robot arm
 *        Piper 机械臂正运动学计算器
 * 
 * @details Implements standard DH parameter-based forward kinematics.
 *          Supports both standard and modified (offset) DH conventions.
 *          实现基于标准 DH 参数的正运动学。
 *          支持标准和修改（偏移）DH 约定。
 */
class PiperForwardKinematics {
 public:
  /**
   * @brief Construct forward kinematics calculator
   *        构造正运动学计算器
   * 
   * @param dh_is_offset Select DH parameter convention:
   *                     - true: Modified (offset) DH parameters
   *                     - false: Standard DH parameters
   *                     选择 DH 参数约定：
   *                     - true：修改（偏移）DH 参数
   *                     - false：标准 DH 参数
   */
  explicit PiperForwardKinematics(bool dh_is_offset = true);

  /**
   * @brief Calculate forward kinematics for all links
   *        计算所有连杆的正运动学
   * 
   * @details Computes the pose of each link relative to the robot base frame.
   *          Each pose is represented as [x, y, z, roll, pitch, yaw].
   *          计算每个连杆相对于机器人基座坐标系的位姿。
   *          每个位姿表示为 [x, y, z, roll, pitch, yaw]。
   * 
   * @param joint_radians Joint angles in radians [6 joints] | 关节角度，单位弧度 [6个关节]
   * @return std::array<std::array<double, 6>, 6> Poses for all 6 links
   *         - [0-2]: position (x, y, z) in mm | 位置 (x, y, z)，单位 mm
   *         - [3-5]: orientation (roll, pitch, yaw) in degrees | 姿态 (roll, pitch, yaw)，单位度
   *         返回所有6个连杆的位姿数组
   */
  std::array<std::array<double, 6>, 6> CalFK(const std::array<double, 6>& joint_radians) const;

  /**
   * @brief Get radian-to-degree conversion factor used internally
   *        获取内部使用的弧度到角度转换系数
   * @return Conversion factor | 转换系数
   */
  double radian_to_degree() const { return radian_to_degree_; }

 private:
  std::array<double, 6> a_{};           ///< DH parameter: link length | DH参数：连杆长度
  std::array<double, 6> alpha_{};       ///< DH parameter: link twist | DH参数：连杆扭转角
  std::array<double, 6> theta_{};       ///< DH parameter: joint angle offset | DH参数：关节角偏移
  std::array<double, 6> d_{};           ///< DH parameter: link offset | DH参数：连杆偏移
  double radian_to_degree_{0.0};        ///< Radian to degree conversion factor | 弧度到角度转换系数

  /**
   * @brief Compute homogeneous transformation matrix for a single link
   *        计算单个连杆的齐次变换矩阵
   * 
   * @param alpha Link twist (α) | 连杆扭转角 (α)
   * @param a Link length (a) | 连杆长度 (a)
   * @param theta Joint angle (θ) | 关节角 (θ)
   * @param d Link offset (d) | 连杆偏移 (d)
   * @return std::array<double, 16> 4x4 homogeneous transformation matrix (row-major)
   *         4x4 齐次变换矩阵（行主序）
   */
  static std::array<double, 16> LinkTransformation(double alpha, double a, double theta, double d);
  
  /**
   * @brief Multiply two 4x4 homogeneous transformation matrices
   *        乘以两个 4x4 齐次变换矩阵
   * 
   * @param lhs Left-hand side matrix | 左侧矩阵
   * @param rhs Right-hand side matrix | 右侧矩阵
   * @return std::array<double, 16> Result matrix (lhs * rhs) | 结果矩阵（lhs * rhs）
   */
  static std::array<double, 16> Multiply(const std::array<double, 16>& lhs,
                                         const std::array<double, 16>& rhs);
  
  /**
   * @brief Extract pose (position + Euler angles) from transformation matrix
   *        从变换矩阵中提取位姿（位置 + 欧拉角）
   * 
   * @param T 4x4 homogeneous transformation matrix | 4x4 齐次变换矩阵
   * @param rad_to_deg Radian to degree conversion factor | 弧度到角度转换系数
   * @return std::array<double, 6> [x, y, z, roll, pitch, yaw]
   *         - Position in mm | 位置，单位 mm
   *         - Orientation in degrees | 姿态，单位度
   */
  static std::array<double, 6> MatrixToEuler(const std::array<double, 16>& T, double rad_to_deg);
};

}  // namespace piper_sdk
