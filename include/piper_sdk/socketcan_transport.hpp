/**
 * @file socketcan_transport.hpp
 * @brief SocketCAN transport implementation for Linux
 *        Linux下的SocketCAN传输实现
 * 
 * @details Provides CAN bus communication using Linux SocketCAN interface.
 *          Supports native CAN hardware interfaces (e.g., can0, can1).
 *          使用Linux SocketCAN接口提供CAN总线通信。
 *          支持原生CAN硬件接口（例如 can0、can1）。
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

#include <string>

#include "piper_sdk/can_transport.hpp"

namespace piper_sdk {

/**
 * @class SocketCanTransport
 * @brief SocketCAN implementation of CAN transport
 *        CAN传输的SocketCAN实现
 * 
 * @details Uses Linux SocketCAN API to communicate with CAN hardware.
 *          Requires a properly configured CAN interface (e.g., via `ip link set can0 up type can bitrate 1000000`).
 *          使用Linux SocketCAN API与CAN硬件通信。
 *          需要正确配置的CAN接口（例如，通过 `ip link set can0 up type can bitrate 1000000`）。
 * 
 * @note Only available on Linux systems with SocketCAN support.
 *       仅在支持SocketCAN的Linux系统上可用。
 */
class SocketCanTransport : public CanTransport {
 public:
  /**
   * @brief Construct SocketCAN transport
   *        构造SocketCAN传输
   * 
   * @param interface_name CAN interface name (e.g., "can0", "can1") | CAN接口名称（例如 "can0"、"can1"）
   */
  explicit SocketCanTransport(std::string interface_name);
  
  /**
   * @brief Destructor - closes the socket
   *        析构函数 - 关闭套接字
   */
  ~SocketCanTransport();

  // Non-copyable | 不可复制
  SocketCanTransport(const SocketCanTransport&) = delete;
  SocketCanTransport& operator=(const SocketCanTransport&) = delete;
  
  // Movable | 可移动
  SocketCanTransport(SocketCanTransport&&) noexcept;
  SocketCanTransport& operator=(SocketCanTransport&&) noexcept;

  /**
   * @brief Send a CAN frame over SocketCAN
   *        通过SocketCAN发送CAN帧
   * 
   * @param frame The CAN frame to send | 要发送的CAN帧
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult send(const CanFrame& frame) override;
  
  /**
   * @brief Receive a CAN frame from SocketCAN
   *        从SocketCAN接收CAN帧
   * 
   * @param timeout Maximum wait time for reception | 接收的最大等待时间
   * @return std::optional<CanFrame> containing the received frame, or std::nullopt on timeout
   *         包含接收到的帧的 std::optional<CanFrame>，超时时为 std::nullopt
   */
  std::optional<CanFrame> receive(std::chrono::milliseconds timeout) override;

 private:
  /**
   * @brief Open and configure the SocketCAN socket
   *        打开并配置SocketCAN套接字
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult open_socket();
  
  /**
   * @brief Close the SocketCAN socket
   *        关闭SocketCAN套接字
   */
  void close_socket() noexcept;

  int socket_fd_{-1};              ///< Socket file descriptor | 套接字文件描述符
  std::string interface_name_;     ///< CAN interface name (e.g., "can0") | CAN接口名称（例如 "can0"）
};

}  // namespace piper_sdk
