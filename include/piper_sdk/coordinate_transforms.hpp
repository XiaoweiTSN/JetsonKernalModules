/**
 * @file coordinate_transforms.hpp
 * @brief Coordinate transformation utilities
 *        坐标变换工具
 * 
 * @details Provides conversion functions between quaternions and Euler angles.
 *          提供四元数和欧拉角之间的转换函数。
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
 * @brief Convert quaternion to Euler angles
 *        将四元数转换为欧拉角
 * 
 * @details Converts a unit quaternion to Euler angles using ZYX (yaw-pitch-roll) convention.
 *          使用 ZYX（偏航-俯仰-滚转）约定将单位四元数转换为欧拉角。
 * 
 * @param qx Quaternion x component | 四元数 x 分量
 * @param qy Quaternion y component | 四元数 y 分量
 * @param qz Quaternion z component | 四元数 z 分量
 * @param qw Quaternion w component | 四元数 w 分量
 * @return std::array<double, 3> [roll, pitch, yaw] in radians | [滚转, 俯仰, 偏航]，单位弧度
 */
std::array<double, 3> quat_to_euler(double qx, double qy, double qz, double qw);

/**
 * @brief Convert Euler angles to quaternion
 *        将欧拉角转换为四元数
 * 
 * @details Converts Euler angles to a unit quaternion using ZYX (yaw-pitch-roll) convention.
 *          使用 ZYX（偏航-俯仰-滚转）约定将欧拉角转换为单位四元数。
 * 
 * @param roll Roll angle in radians | 滚转角，单位弧度
 * @param pitch Pitch angle in radians | 俯仰角，单位弧度
 * @param yaw Yaw angle in radians | 偏航角，单位弧度
 * @return std::array<double, 4> [qx, qy, qz, qw] quaternion components | [qx, qy, qz, qw] 四元数分量
 */
std::array<double, 4> euler_to_quat(double roll, double pitch, double yaw);

}  // namespace piper_sdk
