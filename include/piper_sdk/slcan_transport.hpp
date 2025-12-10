/**
 * @file slcan_transport.hpp
 * @brief SLCAN (Serial Line CAN) transport implementation
 *        SLCAN（串行线CAN）传输实现
 * 
 * @details Provides CAN bus communication over serial ports using SLCAN protocol.
 *          Supports USB-CAN adapters and other SLCAN-compatible devices.
 *          使用SLCAN协议通过串口提供CAN总线通信。
 *          支持USB-CAN适配器和其他SLCAN兼容设备。
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
 * @class SlcanTransport
 * @brief SLCAN (Serial Line CAN) transport implementation
 *        SLCAN（串行线CAN）传输实现
 * 
 * @details Implements the standard SLCAN protocol for USB-CAN adapters.
 *          SLCAN encodes CAN frames as ASCII strings over a serial connection.
 *          实现USB-CAN适配器的标准SLCAN协议。
 *          SLCAN通过串口连接将CAN帧编码为ASCII字符串。
 * 
 * @note Supported on Linux, macOS, and Windows with appropriate serial port drivers.
 *       在具有适当串口驱动程序的Linux、macOS和Windows上受支持。
 */
class SlcanTransport : public CanTransport {
 public:
  /**
   * @brief Construct SLCAN transport
   *        构造SLCAN传输
   * 
   * @param port_name Serial port device path (e.g., "/dev/ttyUSB0", "/dev/tty.usbserial")
   *                  串口设备路径（例如 "/dev/ttyUSB0"、"/dev/tty.usbserial"）
   * @param baudrate Serial port baud rate (default: 115200)
   *                 串口波特率（默认：115200）
   */
  explicit SlcanTransport(std::string port_name, uint32_t baudrate = 115200);
  
  /**
   * @brief Destructor - closes the serial port
   *        析构函数 - 关闭串口
   */
  ~SlcanTransport();

  // Non-copyable | 不可复制
  SlcanTransport(const SlcanTransport&) = delete;
  SlcanTransport& operator=(const SlcanTransport&) = delete;
  
  // Movable | 可移动
  SlcanTransport(SlcanTransport&&) noexcept;
  SlcanTransport& operator=(SlcanTransport&&) noexcept;

  /**
   * @brief Send a CAN frame via SLCAN
   *        通过SLCAN发送CAN帧
   * 
   * @param frame The CAN frame to send | 要发送的CAN帧
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult send(const CanFrame& frame) override;
  
  /**
   * @brief Receive a CAN frame via SLCAN
   *        通过SLCAN接收CAN帧
   * 
   * @param timeout Maximum wait time for reception | 接收的最大等待时间
   * @return std::optional<CanFrame> containing the received frame, or std::nullopt on timeout
   *         包含接收到的帧的 std::optional<CanFrame>，超时时为 std::nullopt
   */
  std::optional<CanFrame> receive(std::chrono::milliseconds timeout) override;

 private:
  /**
   * @brief Open and configure the serial port
   *        打开并配置串口
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult open_port();
  
  /**
   * @brief Close the serial port
   *        关闭串口
   */
  void close_port() noexcept;
  
  /**
   * @brief Configure SLCAN adapter (set bitrate, open channel)
   *        配置SLCAN适配器（设置波特率、打开通道）
   * @return CommandResult indicating success or failure | 表示成功或失败的 CommandResult
   */
  CommandResult configure_slcan();
  
  /**
   * @brief Convert CAN frame to SLCAN ASCII format
   *        将CAN帧转换为SLCAN ASCII格式
   * @param frame The CAN frame to encode | 要编码的CAN帧
   * @return SLCAN string representation | SLCAN字符串表示
   */
  std::string frame_to_slcan(const CanFrame& frame);
  
  /**
   * @brief Parse SLCAN ASCII string to CAN frame
   *        解析SLCAN ASCII字符串为CAN帧
   * @param slcan_msg SLCAN string to parse | 要解析的SLCAN字符串
   * @return std::optional<CanFrame> containing the parsed frame, or std::nullopt if invalid
   *         包含解析后的帧的 std::optional<CanFrame>，无效时为 std::nullopt
   */
  std::optional<CanFrame> slcan_to_frame(const std::string& slcan_msg);

  int fd_{-1};                 ///< Serial port file descriptor | 串口文件描述符
  std::string port_name_;      ///< Serial port device path | 串口设备路径
  uint32_t baudrate_;          ///< Serial port baud rate | 串口波特率
  std::string rx_buffer_;      ///< Reception buffer for partial messages | 部分消息的接收缓冲区
};

}  // namespace piper_sdk
