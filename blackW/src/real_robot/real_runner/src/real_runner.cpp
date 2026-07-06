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
#include <cstring>
#include <cerrno>
#include <iostream>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <chrono>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "imu/vqf.hpp"
#include "utils/set_zero.h"
#include "utils/secure_protect.hpp"
#include "utils/serial_packages.hpp"

using std::placeholders::_1;
const int motorNum = rosMotorNum;

// 定义倾斜阈值 (60度)
const double ROLL_THRESHOLD = M_PI / 3.0;
const double PITCH_THRESHOLD = M_PI / 3.0;

namespace {
constexpr size_t AB5465_FRAME_LEN = 66;
constexpr uint8_t AB5465_HEADER[4] = {0xAB, 0x54, 0x65, 0x00};

struct Ab5465ImuSample {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
};

uint16_t readLeU16(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<uint16_t>(data[offset]) |
        (static_cast<uint16_t>(data[offset + 1]) << 8);
}

uint32_t readLeU32(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<uint32_t>(data[offset]) |
        (static_cast<uint32_t>(data[offset + 1]) << 8) |
        (static_cast<uint32_t>(data[offset + 2]) << 16) |
        (static_cast<uint32_t>(data[offset + 3]) << 24);
}

float readLeFloat(const std::vector<uint8_t>& data, size_t offset) {
    const uint32_t raw = readLeU32(data, offset);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

uint16_t crc16CcittFalse(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

speed_t baudToTermios(int baudrate) {
    switch (baudrate) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B460800;
    }
}

bool hasAb5465Header(const std::vector<uint8_t>& data) {
    return data.size() >= 4 &&
        std::equal(std::begin(AB5465_HEADER), std::end(AB5465_HEADER), data.begin());
}
} // namespace

class RealRunner : public rclcpp::Node {
public:
    RealRunner() : Node("realRunner"), SerialPack_("/dev/leg_0", "/dev/leg_1", "/dev/leg_2", "/dev/leg_3"),_lowState(motorNum),_lowCmd(motorNum) {
        imu_port_ = this->declare_parameter<std::string>("imu_port", "/dev/IMU_Link");
        imu_baudrate_ = this->declare_parameter<int>("imu_baudrate", 460800);
        imu_vqf_enabled_ = this->declare_parameter<bool>("imu_vqf_enabled", true);
        imu_vqf_tau_acc_ = this->declare_parameter<double>("imu_vqf_tau_acc", 3.0);
        imu_vqf_dt_ = this->declare_parameter<double>("imu_vqf_dt", 0.002);
        if (imu_vqf_dt_ <= 0.0) {
            imu_vqf_dt_ = 0.002;
        }
        if (imu_vqf_tau_acc_ <= 0.0) {
            imu_vqf_tau_acc_ = 3.0;
        }
        VQFParams vqf_params;
        vqf_params.tauAcc = imu_vqf_tau_acc_;
        imu_vqf_ = std::make_unique<VQF>(vqf_params, imu_vqf_dt_, imu_vqf_dt_);

        last_imu_time_ = this->now();
        last_imu_time_ns_.store(last_imu_time_.nanoseconds(), std::memory_order_relaxed);
        imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>("/_lowState/imu", rclcpp::SensorDataQoS());

        jointStatePub_ = this->create_publisher<robot_msgs::msg::RobotState>("/_lowState/joint", 10);
        commandSub_ = this->create_subscription<robot_msgs::msg::RobotCommand>(
            "/_lowCmd/command", 10,
            std::bind(&RealRunner::_commandCallback, this, _1));
        imuThread_ = std::thread([this]() { this->_imuLoop(); });
        exchangeThread_ = std::thread([this]() { this->_exchangeLoop(); });

    }
    ~RealRunner() {
        running_ = false;
        _closeImuSerial();
        if (imuThread_.joinable()) imuThread_.join();
        if (exchangeThread_.joinable()) exchangeThread_.join();
    }
private:
    void _processImuSafety(const sensor_msgs::msg::Imu& msg) {
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
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
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

    bool _openImuSerial() {
        _closeImuSerial();
        imu_fd_ = ::open(imu_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (imu_fd_ < 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "打开 IMU 串口失败: %s (%s)", imu_port_.c_str(), std::strerror(errno));
            return false;
        }

        termios tty{};
        if (tcgetattr(imu_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "读取 IMU 串口属性失败: %s", std::strerror(errno));
            _closeImuSerial();
            return false;
        }

        cfmakeraw(&tty);
        const speed_t speed = baudToTermios(imu_baudrate_);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
        tty.c_cflag |= CRTSCTS;
#endif
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(imu_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "配置 IMU 串口失败: %s", std::strerror(errno));
            _closeImuSerial();
            return false;
        }
        tcflush(imu_fd_, TCIFLUSH);
        RCLCPP_INFO(this->get_logger(), "IMU 串口已打开: %s @ %d", imu_port_.c_str(), imu_baudrate_);
        return true;
    }

    void _closeImuSerial() {
        if (imu_fd_ >= 0) {
            ::close(imu_fd_);
            imu_fd_ = -1;
        }
    }

    void _imuLoop() {
        std::array<uint8_t, 512> read_buffer{};
        while (rclcpp::ok() && running_.load(std::memory_order_relaxed)) {
            if (imu_fd_ < 0 && !_openImuSerial()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            const ssize_t n = ::read(imu_fd_, read_buffer.data(), read_buffer.size());
            if (n > 0) {
                _processImuBytes(read_buffer.data(), static_cast<size_t>(n));
                continue;
            }
            if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            RCLCPP_ERROR(this->get_logger(), "读取 IMU 串口失败: %s", std::strerror(errno));
            _closeImuSerial();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    void _processImuBytes(const uint8_t* data, size_t size) {
        imu_rx_buffer_.insert(imu_rx_buffer_.end(), data, data + size);
        if (imu_rx_buffer_.size() > 4096) {
            imu_rx_buffer_.erase(imu_rx_buffer_.begin(), imu_rx_buffer_.end() - 512);
        }

        while (imu_rx_buffer_.size() >= 4) {
            if (!hasAb5465Header(imu_rx_buffer_)) {
                imu_rx_buffer_.erase(imu_rx_buffer_.begin());
                continue;
            }
            if (imu_rx_buffer_.size() < AB5465_FRAME_LEN) {
                return;
            }

            const uint16_t frame_crc = readLeU16(imu_rx_buffer_, AB5465_FRAME_LEN - 2);
            const uint16_t calc_crc = crc16CcittFalse(imu_rx_buffer_.data(), AB5465_FRAME_LEN - 2);
            if (frame_crc != calc_crc) {
                imu_crc_error_count_++;
                imu_rx_buffer_.erase(imu_rx_buffer_.begin());
                continue;
            }

            Ab5465ImuSample sample;
            sample.roll = readLeFloat(imu_rx_buffer_, 11);
            sample.pitch = readLeFloat(imu_rx_buffer_, 15);
            sample.yaw = readLeFloat(imu_rx_buffer_, 19);
            sample.gyro_x = readLeFloat(imu_rx_buffer_, 23);
            sample.gyro_y = readLeFloat(imu_rx_buffer_, 27);
            sample.gyro_z = readLeFloat(imu_rx_buffer_, 31);
            sample.accel_x = readLeFloat(imu_rx_buffer_, 35);
            sample.accel_y = readLeFloat(imu_rx_buffer_, 39);
            sample.accel_z = readLeFloat(imu_rx_buffer_, 43);
            imu_rx_buffer_.erase(imu_rx_buffer_.begin(), imu_rx_buffer_.begin() + AB5465_FRAME_LEN);
            _handleImuSample(sample);
        }
    }

    void _handleImuSample(const Ab5465ImuSample& sample) {
        sensor_msgs::msg::Imu msg;
        msg.header.frame_id = "imu_link";
        msg.header.stamp = this->now();

        tf2::Quaternion raw_quat;
        raw_quat.setRPY(
            sample.roll * M_PI / 180.0,
            -sample.pitch * M_PI / 180.0,
            -sample.yaw * M_PI / 180.0);
        msg.orientation.x = raw_quat.x();
        msg.orientation.y = raw_quat.y();
        msg.orientation.z = raw_quat.z();
        msg.orientation.w = raw_quat.w();

        const double deg2rad = M_PI / 180.0;
        msg.angular_velocity.x = sample.gyro_x * deg2rad;
        msg.angular_velocity.y = -sample.gyro_y * deg2rad;
        msg.angular_velocity.z = -sample.gyro_z * deg2rad;
        msg.linear_acceleration.x = sample.accel_x;
        msg.linear_acceleration.y = -sample.accel_y;
        msg.linear_acceleration.z = -sample.accel_z;

        if (imu_vqf_enabled_) {
            const auto& gyr = msg.angular_velocity;
            const auto& acc = msg.linear_acceleration;
            const double acc_norm = std::sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
            if (std::isfinite(gyr.x) && std::isfinite(gyr.y) && std::isfinite(gyr.z) &&
                std::isfinite(acc.x) && std::isfinite(acc.y) && std::isfinite(acc.z) &&
                acc_norm > 1e-6) {
                const vqf_real_t vqf_gyr[3] = {gyr.x, gyr.y, gyr.z};
                const vqf_real_t vqf_acc[3] = {acc.x, acc.y, acc.z};
                imu_vqf_->update(vqf_gyr, vqf_acc);
                vqf_real_t quat[4];
                imu_vqf_->getQuat6D(quat);
                msg.orientation.w = quat[0];
                msg.orientation.x = quat[1];
                msg.orientation.y = quat[2];
                msg.orientation.z = quat[3];
            }
        }

        {
            std::lock_guard<std::mutex> lock(imu_state_mutex_);
            latest_robot_imu_.quaternion = {
                static_cast<float>(msg.orientation.w),
                static_cast<float>(msg.orientation.x),
                static_cast<float>(msg.orientation.y),
                static_cast<float>(msg.orientation.z)};
            latest_robot_imu_.gyroscope = {
                static_cast<float>(msg.angular_velocity.x),
                static_cast<float>(msg.angular_velocity.y),
                static_cast<float>(msg.angular_velocity.z)};
            latest_robot_imu_.accelerometer = {
                static_cast<float>(msg.linear_acceleration.x),
                static_cast<float>(msg.linear_acceleration.y),
                static_cast<float>(msg.linear_acceleration.z)};
        }

        imu_frame_count_++;
        imuPub_->publish(msg);
        _processImuSafety(msg);
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
        if (msg->motor_command.size() != motorNum) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Invalid motor command size: expected %d, got %zu",
                motorNum,
                msg->motor_command.size());
            return;
        }

        std::lock_guard<std::mutex> lock_cmd(cmd_mutex_);
        _lowCmd.robotCmd.motor_command=msg->motor_command;
        SerialPack_.not_first_command = true;
    }
    void _statePublish() {
        {
            std::lock_guard<std::mutex> lock(imu_state_mutex_);
            _lowState.motorState.imu = latest_robot_imu_;
        }
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
    std::thread imuThread_;
    int imu_fd_ = -1;
    std::string imu_port_;
    int imu_baudrate_ = 460800;
    bool imu_vqf_enabled_ = true;
    double imu_vqf_tau_acc_ = 3.0;
    double imu_vqf_dt_ = 0.002;
    std::unique_ptr<VQF> imu_vqf_;
    std::vector<uint8_t> imu_rx_buffer_;
    std::atomic<uint64_t> imu_frame_count_{0};
    std::atomic<uint64_t> imu_crc_error_count_{0};
    std::mutex imu_state_mutex_;
    robot_msgs::msg::IMU latest_robot_imu_;

    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr jointStatePub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;


    rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr commandSub_;
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
    std::cout
        << "Starting Motor Calibration...\n"
        << "Instructions:\n"
        << "  s + Enter: Record STRAIGHT position (站立/伸直姿态)\n"
        << "  c + Enter: Record CREEP position (趴下/蹲伏姿态)\n"
        << "  w + Enter: Save calibration file\n"
        << "  m + Enter: Show recorded positions (打印当前记录)\n"
        << "  q + Enter: Quit\n"
        << std::flush;

    motor_zero calibrator;
    calibrator.record_position();
    calibrator.save_calibration_file();
#endif
    rclcpp::shutdown();
    return 0;
}
