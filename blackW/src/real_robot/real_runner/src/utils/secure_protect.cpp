#include "utils/secure_protect.hpp"

namespace utils {

void SafetyStateManager::setIsSafe(bool safe) {
    bool previous_safe = is_safe_.load();
    is_safe_.store(safe);
    
    // 状态变化时打印日志
    if (!safe && previous_safe) {
        RCLCPP_ERROR(rclcpp::get_logger("SafetyStateManager"), ">>> 安全监控：切换至阻尼模式 (失衡)! <<<");
    } else if (safe && !previous_safe) {
        RCLCPP_DEBUG(rclcpp::get_logger("SafetyStateManager"), "<<< 安全监控：机器人恢复平衡。 >>>");
    }
}

} // namespace utils