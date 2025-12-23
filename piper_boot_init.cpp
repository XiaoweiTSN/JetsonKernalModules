/**
 * @file piper_boot_init.cpp
 * @brief Piper 机械臂开机自启初始化程序
 *
 * @details 该程序用于机器狗开机后自动执行 Piper 机械臂的初始化流程：
 *          1. 连接 CAN 总线
 *          2. 使能电机
 *          3. 等待使能确认
 *          4. 执行 homing（回零）
 *          5. 等待 homing 完成
 *          6. 退出，让 ROS2 或其他控制程序接管
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

// 错误码定义（用于 systemd 判断是否重试）
enum class InitError : int
{
    OK = 0,
    CAN_OPEN_FAILED = 1,
    CONNECT_FAILED = 2,
    ENABLE_FAILED = 3,
    ENABLE_TIMEOUT = 4,
    HOME_FAILED = 5,
    HOME_TIMEOUT = 6,
    SIGNAL_INTERRUPTED = 7,
    STATUS_CHECK_FAILED = 8,
    ARM_ERROR = 9
};

std::atomic<bool> g_shutdown_requested{false};

// 日志工具
class Logger
{
public:
    enum class Level
    {
        DEBUG,
        INFO,
        WARN,
        ERROR
    };

    static void log(Level level, const std::string &msg)
    {
        const char *level_str = "";
        switch (level)
        {
        case Level::DEBUG:
            level_str = "DEBUG";
            break;
        case Level::INFO:
            level_str = "INFO ";
            break;
        case Level::WARN:
            level_str = "WARN ";
            break;
        case Level::ERROR:
            level_str = "ERROR";
            break;
        }

        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::tm tm_buf{};
        localtime_r(&time_t_now, &tm_buf);

        std::cerr << "[" << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3)
                  << ms.count() << "] [" << level_str << "] " << msg << std::endl;
    }

    static void debug(const std::string &msg)
    {
        log(Level::DEBUG, msg);
    }
    static void info(const std::string &msg)
    {
        log(Level::INFO, msg);
    }
    static void warn(const std::string &msg)
    {
        log(Level::WARN, msg);
    }
    static void error(const std::string &msg)
    {
        log(Level::ERROR, msg);
    }
};

void signal_handler(int signum)
{
    Logger::warn("Received signal " + std::to_string(signum) + ", shutting down...");
    g_shutdown_requested = true;
}

void setup_signal_handlers()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
}

// Piper 初始化器
class PiperInitializer
{
public:
    struct Config
    {
        std::string can_interface{"can0"};
        std::chrono::milliseconds connect_timeout{5000};
        std::chrono::milliseconds enable_timeout{10000};
        std::chrono::milliseconds home_timeout{60000};
        std::chrono::milliseconds status_poll_interval{50};
        int home_retry_count{2};
        uint8_t motion_speed_percent{30};
        // 目标关节位置 (单位: 毫度 mdeg，即 0.001度)
        // 6个关节: {J1, J2, J3, J4, J5, J6}
        std::vector<int32_t> target_joints{0, 0, 0, 0, 0, 0};
    };

    explicit PiperInitializer(const Config &config) : config_(config)
    {
    }
    PiperInitializer() : config_()
    {
    }

    InitError run()
    {
        Logger::info("Piper arm init started, CAN: " + config_.can_interface);

        // Step 1: 创建 CAN 传输层和接口
        InitError err = createInterface();
        if (err != InitError::OK) return err;

        // Step 2: 连接端口
        err = connectPort();
        if (err != InitError::OK) return err;

        // Step 3: 等待通信建立
        err = waitForConnection();
        if (err != InitError::OK) return err;

        // Step 4: 检查机械臂状态
        err = checkArmStatus();
        if (err != InitError::OK) return err;

        // Step 5: 使能电机（重试直到成功）
        err = enableMotors();
        if (err != InitError::OK) return err;

        // Step 6: 执行回零
        err = performHoming();
        if (err != InitError::OK) return err;

        Logger::info("Piper arm init completed");
        return InitError::OK;
    }

private:
    Config config_;
    std::shared_ptr<piper_sdk::SocketCanTransport> transport_;
    std::unique_ptr<piper_sdk::PiperInterface> piper_;

    InitError createInterface()
    {
        Logger::info("[1/5] Creating CAN transport...");

        try
        {
            transport_ = std::make_shared<piper_sdk::SocketCanTransport>(config_.can_interface);

            piper_sdk::PiperConfig piper_config;
            piper_config.can_name = config_.can_interface;
            piper_config.logger_level = piper_sdk::LogLevel::kInfo;

            piper_ = std::make_unique<piper_sdk::PiperInterface>(transport_, piper_config);

            return InitError::OK;
        }
        catch (const std::exception &e)
        {
            Logger::error("Failed to create interface: " + std::string(e.what()));
            return InitError::CAN_OPEN_FAILED;
        }
    }

    InitError connectPort()
    {
        Logger::info("[2/5] Connecting CAN port...");

        auto result = piper_->connect_port(10ms, true, true);
        if (!result.ok)
        {
            Logger::error("Failed to connect port: " + result.message);
            return InitError::CONNECT_FAILED;
        }

        std::this_thread::sleep_for(200ms);
        return InitError::OK;
    }

    InitError waitForConnection()
    {
        Logger::info("[3/5] Waiting for communication...");

        auto deadline = std::chrono::steady_clock::now() + config_.connect_timeout;

        while (std::chrono::steady_clock::now() < deadline)
        {
            if (g_shutdown_requested)
            {
                Logger::warn("Interrupted");
                return InitError::SIGNAL_INTERRUPTED;
            }

            auto status = piper_->get_connect_status();
            if (status.connected && !status.stale)
            {
                Logger::info("Communication established");

                piper_->get_piper_firmware_version();
                std::this_thread::sleep_for(100ms);
                auto fw_version = piper_->get_cached_firmware_version();
                if (fw_version)
                {
                    Logger::info("Firmware: " + *fw_version);
                }

                return InitError::OK;
            }

            std::this_thread::sleep_for(config_.status_poll_interval);
        }

        Logger::error("Connection timeout");
        return InitError::CONNECT_FAILED;
    }

    InitError checkArmStatus()
    {
        Logger::info("[4/5] Checking arm status...");

        auto arm_status = piper_->get_arm_status();
        if (!arm_status)
        {
            Logger::warn("Cannot get arm status, continuing...");
            return InitError::OK;
        }

        if (arm_status->error_code != 0)
        {
            Logger::error("Arm error code: " + std::to_string(arm_status->error_code));

            Logger::info("Trying to reset...");
            piper_->reset_arm();
            std::this_thread::sleep_for(500ms);

            arm_status = piper_->get_arm_status();
            if (arm_status && arm_status->error_code != 0)
            {
                Logger::error("Reset failed");
                return InitError::ARM_ERROR;
            }
        }

        Logger::info("Connected: " + std::string(arm_status->enabled ? "yes" : "no") +
                     ", mode: " + std::to_string(arm_status->control_mode));

        return InitError::OK;
    }

    InitError enableMotors()
    {
        Logger::info("[5/5] Enabling arm...");

        // 先发送使能命令
        auto result = piper_->enable_arm(7);
        if (!result.ok)
        {
            Logger::warn("Enable command failed: " + result.message);
        }

        // 使用重试循环等待使能完成
        constexpr int MAX_RETRIES = 200; // 200 * 10ms = 2s，超时后还有 enable_timeout
        int retry_count = 0;

        while (!piper_->enable_piper() && retry_count < MAX_RETRIES)
        {
            if (g_shutdown_requested)
            {
                return InitError::SIGNAL_INTERRUPTED;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            retry_count++;
        }

        if (retry_count >= MAX_RETRIES)
        {
            // 继续用更长的超时等待
            auto deadline = std::chrono::steady_clock::now() + config_.enable_timeout;
            while (std::chrono::steady_clock::now() < deadline)
            {
                if (g_shutdown_requested)
                {
                    return InitError::SIGNAL_INTERRUPTED;
                }
                if (piper_->enable_piper())
                {
                    Logger::info("Arm enabled");
                    return InitError::OK;
                }
                std::this_thread::sleep_for(config_.status_poll_interval);
            }
            Logger::error("Enable timeout");
            return InitError::ENABLE_TIMEOUT;
        }

        Logger::info("Arm enabled");
        return InitError::OK;
    }

    InitError performHoming()
    {
        Logger::info("Performing homing...");

        for (int retry = 0; retry < config_.home_retry_count; retry++)
        {
            if (g_shutdown_requested)
            {
                return InitError::SIGNAL_INTERRUPTED;
            }

            if (retry > 0)
            {
                Logger::warn("Homing retry #" + std::to_string(retry));
                std::this_thread::sleep_for(1000ms);
            }

            // 设置运动模式
            auto mode_result = piper_->motion_control_2(0x01, 0x01, config_.motion_speed_percent, 0x00, 0x00, 0x00);

            if (!mode_result.ok)
            {
                Logger::warn("Set motion mode failed: " + mode_result.message);
                continue;
            }

            std::this_thread::sleep_for(100ms);

            auto move_result = piper_->move_joint(config_.target_joints);

            if (!move_result.ok)
            {
                Logger::warn("Homing command failed: " + move_result.message);
                continue;
            }

            Logger::info("Homing command sent, waiting...");

            InitError wait_result = waitHomingDone();
            if (wait_result == InitError::OK)
            {
                return InitError::OK;
            }
            else if (wait_result == InitError::SIGNAL_INTERRUPTED)
            {
                return wait_result;
            }
        }

        Logger::error("Homing failed");
        return InitError::HOME_FAILED;
    }

    InitError waitHomingDone()
    {
        auto deadline = std::chrono::steady_clock::now() + config_.home_timeout;
        constexpr int32_t kPositionTolerance = 1000; // 1度容差

        while (std::chrono::steady_clock::now() < deadline)
        {
            if (g_shutdown_requested)
            {
                return InitError::SIGNAL_INTERRUPTED;
            }

            auto arm_status = piper_->get_arm_status();
            if (arm_status)
            {
                if (arm_status->error_code != 0)
                {
                    Logger::error("Arm error: " + std::to_string(arm_status->error_code));
                    return InitError::ARM_ERROR;
                }

                bool all_at_target = true;
                for (size_t i = 0; i < arm_status->joints.position_mdeg.size() && i < config_.target_joints.size(); i++)
                {
                    int32_t diff = std::abs(arm_status->joints.position_mdeg[i] - config_.target_joints[i]);
                    if (diff > kPositionTolerance)
                    {
                        all_at_target = false;
                        break;
                    }
                }

                if (all_at_target)
                {
                    Logger::info("Target position reached");
                    return InitError::OK;
                }
            }

            std::this_thread::sleep_for(config_.status_poll_interval);
        }

        Logger::error("Move to target timeout");
        return InitError::HOME_TIMEOUT;
    }
};

int main()
{
    setup_signal_handlers();

    PiperInitializer::Config config;
    // 可以在这里硬编码配置，或通过环境变量读取
    // config.can_interface = "can0";
    
    // 设置目标关节位置 (单位: 毫度 mdeg)
    // 例如: 第一个关节转到 45度 = 45000 mdeg
    config.target_joints = {-90000, 0, 0, 0, 0, 0};

    PiperInitializer initializer(config);
    InitError result = initializer.run();

    int exit_code = static_cast<int>(result);
    if (result == InitError::OK)
    {
        Logger::info("Exit code: 0");
    }
    else
    {
        Logger::error("Exit code: " + std::to_string(exit_code));
    }

    return exit_code;
}
