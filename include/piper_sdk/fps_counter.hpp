/**
 * @file fps_counter.hpp
 * @brief FPS (Frames Per Second) counter utility
 *        FPS（每秒帧数）计数器工具
 * 
 * @details Provides frequency calculation for periodic events (e.g., CAN message reception).
 *          为周期性事件（例如 CAN 消息接收）提供频率计算。
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

namespace piper_sdk {

/**
 * @class FpsCounter
 * @brief Simple FPS/frequency counter
 *        简单的 FPS/频率计数器
 * 
 * @details Calculates frequency (Hz) of tick() calls using a sliding window approach.
 *          Useful for monitoring CAN bus message rates and detecting communication issues.
 *          使用滑动窗口方法计算 tick() 调用的频率（Hz）。
 *          用于监视 CAN 总线消息速率和检测通信问题。
 */
class FpsCounter {
 public:
  /**
   * @brief Record an event occurrence
   *        记录事件发生
   * 
   * @details Call this method each time the event occurs (e.g., each time a CAN frame is received).
   *          每次事件发生时调用此方法（例如，每次接收到 CAN 帧时）。
   */
  void tick();
  
  /**
   * @brief Get current frequency in Hz
   *        获取当前频率（Hz）
   * 
   * @details Returns the calculated frequency based on recent tick() calls.
   *          Returns 0.0 if insufficient data is available.
   *          根据最近的 tick() 调用返回计算的频率。
   *          如果可用数据不足，则返回 0.0。
   * 
   * @return Frequency in Hz | 频率（Hz）
   */
  double hz() const;

 private:
  std::chrono::steady_clock::time_point last_mark_{};     ///< Last tick timestamp | 最后一次 tick 时间戳
  std::chrono::steady_clock::time_point window_start_{};  ///< Window start timestamp | 窗口开始时间戳
  std::size_t samples_{0};                                ///< Sample count in current window | 当前窗口中的样本计数
  double last_hz_{0.0};                                   ///< Last calculated frequency | 最后计算的频率
};

}  // namespace piper_sdk
