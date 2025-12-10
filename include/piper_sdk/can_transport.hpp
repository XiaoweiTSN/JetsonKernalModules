/**
 * @file can_transport.hpp
 * @brief CAN bus transport abstraction interface
 *        CAN总线传输抽象接口
 * 
 * @details Defines the abstract base class for CAN frame transmission and reception,
 *          along with command result and frame data structures.
 *          定义了CAN帧传输和接收的抽象基类，以及命令结果和帧数据结构。
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

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <array>

namespace piper_sdk {

/**
 * @struct CommandResult
 * @brief Result of a command execution
 *        命令执行结果
 * 
 * @details Contains success/failure status, descriptive message, and error code.
 *          包含成功/失败状态、描述性消息和错误代码。
 */
struct CommandResult {
  bool ok{false};          ///< Success flag | 成功标志
  std::string message;     ///< Result message | 结果消息
  int error_code{0};       ///< Error code (0 = success) | 错误代码（0 = 成功）

  /**
   * @brief Create a success result
   *        创建成功结果
   * @param msg Optional success message | 可选的成功消息
   * @return CommandResult indicating success | 表示成功的 CommandResult
   */
  static CommandResult success(std::string msg = "") {
    return CommandResult{true, std::move(msg), 0};
  }

  /**
   * @brief Create a failure result
   *        创建失败结果
   * @param msg Error message | 错误消息
   * @param code Error code (default: -1) | 错误代码（默认：-1）
   * @return CommandResult indicating failure | 表示失败的 CommandResult
   */
  static CommandResult failure(std::string msg, int code = -1) {
    return CommandResult{false, std::move(msg), code};
  }
};

/**
 * @struct CanFrame
 * @brief CAN bus frame representation
 *        CAN总线帧表示
 * 
 * @details Standard CAN 2.0 frame with 11 or 29-bit identifier and up to 8 data bytes.
 *          标准 CAN 2.0 帧，具有 11 位或 29 位标识符和最多 8 个数据字节。
 */
struct CanFrame {
  uint32_t id{0};              ///< CAN identifier (11-bit or 29-bit) | CAN标识符（11位或29位）
  std::array<uint8_t, 8> data{}; ///< Data payload (up to 8 bytes) | 数据载荷（最多8字节）
  size_t size{0};              ///< Actual data length (0-8) | 实际数据长度（0-8）
};

/**
 * @class CanTransport
 * @brief Abstract base class for CAN bus communication
 *        CAN总线通信的抽象基类
 * 
 * @details Provides interface for sending and receiving CAN frames.
 *          Implementations include SocketCAN (Linux), SLCAN (serial), and Mock (testing).
 *          提供发送和接收CAN帧的接口。
 *          实现包括 SocketCAN（Linux）、SLCAN（串口）和 Mock（测试）。
 */
class CanTransport {
 public:
  virtual ~CanTransport() = default;

  /**
   * @brief Send a CAN frame
   *        发送CAN帧
   * 
   * @param frame The CAN frame to send | 要发送的CAN帧
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  virtual CommandResult send(const CanFrame& frame) = 0;
  
  /**
   * @brief Receive a CAN frame with timeout
   *        在超时时间内接收CAN帧
   * 
   * @param timeout Maximum wait time for frame reception | 帧接收的最大等待时间
   * @return std::optional<CanFrame> containing the received frame if successful, std::nullopt on timeout or error
   *         如果成功则包含接收到的帧的 std::optional<CanFrame>，超时或错误时为 std::nullopt
   */
  virtual std::optional<CanFrame> receive(std::chrono::milliseconds timeout) = 0;
};

}  // namespace piper_sdk
