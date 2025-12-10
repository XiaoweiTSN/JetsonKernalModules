/**
 * @file mock_can_transport.hpp
 * @brief Mock CAN transport for testing and simulation
 *        用于测试和仿真的模拟CAN传输
 * 
 * @details Provides in-memory CAN frame queues for unit testing without hardware.
 *          Allows injecting test frames and capturing sent frames.
 *          为单元测试提供内存中的CAN帧队列，无需硬件。
 *          允许注入测试帧并捕获发送的帧。
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

#include <condition_variable>
#include <mutex>
#include <queue>

#include "piper_sdk/can_transport.hpp"

namespace piper_sdk {

/**
 * @class MockCanTransport
 * @brief Mock implementation of CAN transport for testing
 *        用于测试的CAN传输模拟实现
 * 
 * @details Thread-safe mock transport using internal queues.
 *          Useful for unit testing, simulation, and debugging without CAN hardware.
 *          使用内部队列的线程安全模拟传输。
 *          用于单元测试、仿真和无CAN硬件的调试。
 * 
 * @note Not for production use - designed for testing only.
 *       不用于生产环境 - 仅用于测试。
 */
class MockCanTransport : public CanTransport {
 public:
  /**
   * @brief Send a CAN frame (adds to outbound queue)
   *        发送CAN帧（添加到发送队列）
   * 
   * @details Captures the frame in an internal queue for later inspection.
   *          将帧捕获到内部队列中以供稍后检查。
   * 
   * @param frame The CAN frame to send | 要发送的CAN帧
   * @return CommandResult Always returns success | 始终返回成功
   */
  CommandResult send(const CanFrame& frame) override;
  
  /**
   * @brief Receive a CAN frame (pops from inbound queue)
   *        接收CAN帧（从接收队列中弹出）
   * 
   * @details Waits for a frame to become available or timeout to expire.
   *          Use push_receive_frame() to inject test frames.
   *          等待帧可用或超时到期。
   *          使用 push_receive_frame() 注入测试帧。
   * 
   * @param timeout Maximum wait time | 最大等待时间
   * @return std::optional<CanFrame> containing the next frame, or std::nullopt on timeout
   *         包含下一个帧的 std::optional<CanFrame>，超时时为 std::nullopt
   */
  std::optional<CanFrame> receive(std::chrono::milliseconds timeout) override;

  /**
   * @brief Push a frame to be returned on next receive() call
   *        推送一个帧，在下次 receive() 调用时返回
   * 
   * @details Adds a frame to the inbound queue for simulating received data.
   *          将帧添加到接收队列以模拟接收的数据。
   * 
   * @param frame The frame to inject | 要注入的帧
   */
  void push_receive_frame(const CanFrame& frame);
  
  /**
   * @brief Retrieve a copy of all frames that have been sent
   *        检索已发送的所有帧的副本
   * 
   * @details Useful for verifying command sequences in tests.
   *          用于在测试中验证命令序列。
   * 
   * @return std::queue<CanFrame> Copy of the outbound frame queue | 发送帧队列的副本
   */
  std::queue<CanFrame> sent_frames() const;

 private:
  mutable std::mutex mutex_;           ///< Mutex for thread-safe access | 线程安全访问的互斥锁
  std::condition_variable cond_;       ///< Condition variable for receive blocking | 接收阻塞的条件变量
  std::queue<CanFrame> outbound_;      ///< Queue of sent frames | 已发送帧的队列
  std::queue<CanFrame> inbound_;       ///< Queue of frames to receive | 要接收的帧的队列
};

}  // namespace piper_sdk
