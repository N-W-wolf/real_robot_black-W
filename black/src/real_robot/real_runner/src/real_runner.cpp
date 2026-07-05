#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "message/LowLevelCmd.hpp"
#include "message/LowLevelState.hpp"
#include "serialPort/SerialPort.h"

#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>
#include <cstdint>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "utils/set_zero.h"
#include "utils/secure_protect.hpp"
#include "utils/serial_packages.hpp"

using std::placeholders::_1;
const int motorNum = 12;

// 定义倾斜阈值 (30度)
const double ROLL_THRESHOLD = M_PI / 6.0;
const double PITCH_THRESHOLD = M_PI / 6.0;

class RealRunner : public rclcpp::Node {
public:
    RealRunner() : Node("realRunner"), SerialPack_("/dev/leg_0", "/dev/leg_1", "/dev/leg_2", "/dev/leg_3"),_lowState(motorNum),_lowCmd(motorNum) {
        last_imu_time_ = this->now();
        last_imu_time_ns_.store(last_imu_time_.nanoseconds(), std::memory_order_relaxed);
        // --- IMU 订阅与安全监控 ---
        auto qos = rclcpp::SensorDataQoS();
        rclcpp::SubscriptionOptions options;

        // 设置 QoS 事件回调：监控 IMU 是否存活
        options.event_callbacks.liveliness_callback =
            [this](rclcpp::QOSLivelinessChangedInfo & info) {
                if (info.alive_count == 0) {
                    RCLCPP_ERROR(this->get_logger(), "IMU 节点已掉线或未启动！");
                    imu_alive_ = false;
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "IMU 节点已上线");
                    imu_alive_ = true;
                }
            };

        // 订阅 IMU 数据 (这里合并了之前的 imuAliveSub 和安全检测逻辑)
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("/_lowState/imu", qos,
            std::bind(&RealRunner::_imuCallback, this, _1),
            options);

        jointStatePub_ = this->create_publisher<robot_msgs::msg::RobotState>("/_lowState/joint", 10);
        commandSub_ = this->create_subscription<robot_msgs::msg::RobotCommand>(
            "/_lowCmd/command", 10,
            std::bind(&RealRunner::_commandCallback, this, _1));
        exchangeThread_ = std::thread([this]() { this->_exchangeLoop(); });

    }
    ~RealRunner() {
        running_ = false;
        if (exchangeThread_.joinable()) exchangeThread_.join();
    }
private:
    // IMU 回调函数 (执行安全检查)
    void _imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto now = this->now();
        const int64_t now_ns = now.nanoseconds();
        const int64_t previous_imu_time_ns =
            last_imu_time_ns_.exchange(now_ns, std::memory_order_relaxed);
        last_imu_time_ = now; // 喂狗：更新时间戳
        const bool first_imu = !has_received_imu_.exchange(true, std::memory_order_relaxed);
        const bool was_alive = imu_alive_.exchange(true, std::memory_order_relaxed);
        imu_timeout_strikes_.store(0, std::memory_order_relaxed);

        if (first_imu) {
            RCLCPP_DEBUG(this->get_logger(), "收到第一帧 IMU，等待数据流稳定后启用超时监控。");
        } else if (!was_alive) {
            RCLCPP_WARN(this->get_logger(), "IMU 数据恢复，已清除超时计数；安全状态保持当前保护逻辑。");
        }

        if (!first_imu && previous_imu_time_ns > 0) {
            const double gap =
                static_cast<double>(now_ns - previous_imu_time_ns) * 1e-9;
            if (gap > 0.0 && gap <= IMU_STABLE_MAX_GAP_SEC) {
                const int stable_count =
                    imu_stable_frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (stable_count >= IMU_STABLE_FRAME_REQUIRED &&
                    !imu_timeout_monitor_enabled_.exchange(true, std::memory_order_relaxed)) {
                    RCLCPP_DEBUG(
                        this->get_logger(),
                        "IMU 数据流已稳定，启用超时监控。");
                }
            } else {
                imu_stable_frame_count_.store(0, std::memory_order_relaxed);
            }
        }

        // 1. 如果imu状态已经是不安全，则无需再计算
        if (!utils::safeok())
            return;
        
        // 2. 姿态解算 (四元数 -> 欧拉角)
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 3. 判断是否失衡
        bool currently_safe = true;
        if (std::abs(roll) > ROLL_THRESHOLD || std::abs(pitch) > PITCH_THRESHOLD) {
            currently_safe = false;
            // 限流打印，避免刷屏
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "危险：检测到姿态失衡 (Roll: %.2f, Pitch: %.2f) -> 切换阻尼模式", roll, pitch);
        }

        // 4. 更新单例状态
        // SerialPack 线程会读取这个单例状态
        utils::SafetyStateManager::getInstance().setIsSafe(currently_safe);
    }

    void _exchangeLoop() {
        rclcpp::Rate rate(1/dt_); // 1000Hz

        // 预先定义一个局部变量用于拷贝，避免重复创建对象
        LowLevelCmd local_cmd_copy(motorNum);

        while (rclcpp::ok() && running_) {
            // --- 看门狗检查 ---
            auto now = this->now();
            if (!has_received_imu_.load(std::memory_order_relaxed)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "等待第一帧 IMU，暂不启用超时监控...");
            } else if (!imu_timeout_monitor_enabled_.load(std::memory_order_relaxed)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "等待 IMU 数据流稳定，暂不启用超时监控...");
            } else {
                const int64_t last_imu_time_ns = last_imu_time_ns_.load(std::memory_order_relaxed);
                const double time_since_last_imu =
                    static_cast<double>(now.nanoseconds() - last_imu_time_ns) * 1e-9;

                if (time_since_last_imu > IMU_TIMEOUT_SEC) {
                    const int timeout_strikes =
                        imu_timeout_strikes_.fetch_add(1, std::memory_order_relaxed) + 1;

                    if (timeout_strikes < IMU_TIMEOUT_CONFIRM_COUNT) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), *this->get_clock(), 500,
                            "IMU 疑似超时 %.3fs，连续计数 %d/%d。",
                            time_since_last_imu, timeout_strikes, IMU_TIMEOUT_CONFIRM_COUNT);
                    } else if (imu_alive_.exchange(false, std::memory_order_relaxed)) {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "IMU 数据连续超时 (last gap %.3fs, strikes=%d)！触发阻尼模式！",
                            time_since_last_imu, timeout_strikes);
                        utils::SafetyStateManager::getInstance().setIsSafe(false);
                    }
                } else if (imu_timeout_strikes_.load(std::memory_order_relaxed) != 0) {
                    imu_timeout_strikes_.store(0, std::memory_order_relaxed);
                }
            }

            auto loop_start = this->now();
            {
                // 加锁，快速拷贝一份指令
                std::lock_guard<std::mutex> lock_cmd(cmd_mutex_);
                local_cmd_copy = _lowCmd; 
            }
 
            // 使用线程安全的副本进行发送
            SerialPack_.sendRecv(local_cmd_copy, _lowState);

            _statePublish();

            auto loop_end = this->now();
            auto actual_duration = loop_end - loop_start;
            if (actual_duration > rclcpp::Duration::from_seconds(dt_)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Control loop overtime: %.3fms ", actual_duration.seconds() * 1000);
            }

            rate.sleep();
        }
    }
    void _commandCallback(const robot_msgs::msg::RobotCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lock_cmd(cmd_mutex_);
        _lowCmd.robotCmd.motor_command=msg->motor_command;
        SerialPack_.not_first_command = true;
    }
    void _statePublish() {
        _lowState.imu.header.stamp = this->now();
        jointStatePub_->publish(_lowState.motorState);
    }

    SerialPack SerialPack_;
    std::vector<MotorCmd> motorCmdBuf_{motorNum};
    std::vector<MotorData> motorDataBuf_{motorNum};

    bool is_offset_initialized_ = false;
    std::mutex cmd_mutex_;
    std::mutex state_mutex_;
    //SerialPort SerialPort_;
    LowLevelState _lowState;
    LowLevelCmd _lowCmd;
    std::atomic<bool> running_{true};
    std::atomic<bool> imu_alive_{true};
    std::atomic<bool> has_received_imu_{false};
    std::atomic<bool> imu_timeout_monitor_enabled_{false};
    std::atomic<int> imu_timeout_strikes_{0};
    std::atomic<int> imu_stable_frame_count_{0};
    std::atomic<int64_t> last_imu_time_ns_{0};
    std::thread exchangeThread_;

    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr jointStatePub_;


    rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr commandSub_;
    // 修改为通用的 subscription，不再只作为 alive 检查
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Time last_imu_time_;

    // 阻尼模式参数（关节侧值，将在 _sendRecv 中转换为转子侧）
    const float DAMPING_KD = 5.0f; 

    // 转子与输出端转换系数
    float gear_ratio = 6.33f;
    float gear_ratio_squared = 40.0689f; // 6.33 * 6.33

    motor_zero motor_zero_;

    double dt_ = 0.005;
    const double IMU_TIMEOUT_SEC = 0.5;
    const int IMU_TIMEOUT_CONFIRM_COUNT = 3;
    const double IMU_STABLE_MAX_GAP_SEC = 0.02;
    const int IMU_STABLE_FRAME_REQUIRED = 250;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
#ifndef CALIBRATE
    auto node = std::make_shared<RealRunner>();
    rclcpp::spin(node);
#else
    auto node = std::make_shared<rclcpp::Node>("motor_calibration_node");
    RCLCPP_DEBUG(node->get_logger(), "Starting Motor Calibration...");
    RCLCPP_DEBUG(node->get_logger(), "Instructions:");
    RCLCPP_DEBUG(node->get_logger(), "  Type 's' + Enter: Record STRAIGHT position (站立/伸直姿态)");
    RCLCPP_DEBUG(node->get_logger(), "  Type 'c' + Enter: Record CREEP position (趴下/蹲伏姿态)");
    RCLCPP_DEBUG(node->get_logger(), "  Type 'm' + Enter: Show recorded positions (打印当前记录)");
    RCLCPP_DEBUG(node->get_logger(), "  Type 'q' + Enter: Quit");

    motor_zero calibrator;
    calibrator.record_position();
    calibrator.save_calibration_file();
#endif
    rclcpp::shutdown();
    return 0;
}
