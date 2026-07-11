#pragma once

#include <atomic>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp" 

namespace utils {

// 机器人失衡阈值 (单位：弧度)
const double ROLL_THRESHOLD = M_PI / 6.0;  // 30度
const double PITCH_THRESHOLD = M_PI / 6.0; // 30度

class SafetyStateManager {
public:
    static SafetyStateManager& getInstance() {
        static SafetyStateManager instance;
        return instance;
    }

    void setIsSafe(bool safe);

    void setProtectionEnabled(bool enabled) {
        protection_enabled_.store(enabled);
        if (!enabled) {
            is_safe_.store(true);
        }
    }

    // 返回 1 表示安全 (OK)，0 表示不安全 (Not OK)
    int safeok() const {
        return (!protection_enabled_.load() || is_safe_.load()) ? 1 : 0;
    }

private:
    SafetyStateManager() : is_safe_(true), protection_enabled_(true) {}
    ~SafetyStateManager() = default;

    SafetyStateManager(const SafetyStateManager&) = delete;
    SafetyStateManager& operator=(const SafetyStateManager&) = delete;

    std::atomic<bool> is_safe_; // 线程安全的状态标志
    std::atomic<bool> protection_enabled_;
};

// 暴露给 real_runner 调用的接口函数
inline int safeok() {
    return SafetyStateManager::getInstance().safeok();
}

} // namespace utils
