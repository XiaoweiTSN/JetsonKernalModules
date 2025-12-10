/**
 * @file logger.hpp
 * @brief Logging system for Piper SDK
 *        Piper SDK 日志系统
 * 
 * @details Provides thread-safe logging with configurable log levels and file output.
 *          支持控制台和文件输出，具有多个日志级别。
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

#include <fstream>
#include <mutex>
#include <optional>
#include <ostream>
#include <string>

namespace piper_sdk {

/**
 * @enum LogLevel
 * @brief Logging severity levels
 *        日志严重性级别
 */
enum class LogLevel {
  kDebug = 0,    ///< Debug information (most verbose) | 调试信息（最详细）
  kInfo = 1,     ///< Informational messages | 信息消息
  kWarning = 2,  ///< Warning messages | 警告消息
  kError = 3,    ///< Error messages | 错误消息
  kCritical = 4, ///< Critical errors | 严重错误
  kSilent = 5,   ///< No logging | 无日志
};

/**
 * @class Logger
 * @brief Thread-safe logging system
 *        线程安全的日志系统
 * 
 * @details Supports logging to console and/or file with configurable severity filtering.
 *          All operations are thread-safe using internal mutex.
 *          支持使用可配置的严重性过滤记录到控制台和/或文件。
 *          所有操作都使用内部互斥锁实现线程安全。
 */
class Logger {
 public:
  /**
   * @brief Construct logger with configuration
   *        使用配置构造日志记录器
   * 
   * @param level Minimum log level to output | 输出的最小日志级别
   * @param log_to_file Enable file logging | 启用文件日志记录
   * @param file_path Log file path (required if log_to_file is true) | 日志文件路径（如果 log_to_file 为 true 则必需）
   */
  Logger(LogLevel level = LogLevel::kInfo, bool log_to_file = false,
         std::string file_path = "");

  /**
   * @brief Set the minimum log level
   *        设置最小日志级别
   * 
   * @details Messages below this level will be filtered out.
   *          低于此级别的消息将被过滤掉。
   * 
   * @param level New minimum log level | 新的最小日志级别
   */
  void set_level(LogLevel level);
  
  /**
   * @brief Get the current log level
   *        获取当前日志级别
   * @return Current minimum log level | 当前最小日志级别
   */
  LogLevel level() const;

  /**
   * @brief Log a message with specified severity
   *        使用指定的严重性记录消息
   * 
   * @details Thread-safe. Messages are prefixed with timestamp and severity.
   *          线程安全。消息以时间戳和严重性为前缀。
   * 
   * @param level Severity level of the message | 消息的严重性级别
   * @param message Message content | 消息内容
   */
  void log(LogLevel level, const std::string& message);

 private:
  /**
   * @brief Check if a message should be logged based on level
   *        根据级别检查是否应记录消息
   * @param level Message severity level | 消息严重性级别
   * @return true if message should be logged | 如果应记录消息则返回 true
   */
  bool should_log(LogLevel level) const;

  LogLevel level_{LogLevel::kInfo};        ///< Current minimum log level | 当前最小日志级别
  bool log_to_file_{false};                ///< File logging enabled flag | 文件日志记录启用标志
  std::string file_path_;                  ///< Log file path | 日志文件路径
  mutable std::mutex mutex_;               ///< Mutex for thread-safe logging | 线程安全日志记录的互斥锁
  std::optional<std::ofstream> file_stream_; ///< File output stream | 文件输出流
};

}  // namespace piper_sdk
