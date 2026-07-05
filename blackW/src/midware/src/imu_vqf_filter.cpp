#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "midware/vqf.hpp"

class ImuVqfFilter : public rclcpp::Node {
public:
    ImuVqfFilter() : Node("imu_vqf_filter") {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/_lowState/imu_raw");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/_lowState/imu");
        tau_acc_ = this->declare_parameter<double>("tau_acc", 3.0);
        initial_dt_ = this->declare_parameter<double>("initial_dt", 0.002);
        pass_through_ = this->declare_parameter<bool>("pass_through", false);
        log_period_ = this->declare_parameter<double>("log_period", 1.0);

        if (initial_dt_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "initial_dt <= 0, fallback to 0.002 s");
            initial_dt_ = 0.002;
        }
        if (tau_acc_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "tau_acc <= 0, fallback to 3.0 s");
            tau_acc_ = 3.0;
        }

        VQFParams params;
        params.tauAcc = tau_acc_;
        vqf_ = std::make_unique<VQF>(params, initial_dt_, initial_dt_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(200));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, qos);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic_, qos, std::bind(&ImuVqfFilter::imuCallback, this, std::placeholders::_1));

        RCLCPP_DEBUG(
            this->get_logger(),
            "VQF IMU filter: %s -> %s, tau_acc=%.3f, dt=%.6f, pass_through=%s",
            input_topic_.c_str(), output_topic_.c_str(), tau_acc_, initial_dt_, pass_through_ ? "true" : "false");
    }

private:
    static bool finiteVector(const geometry_msgs::msg::Vector3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (pass_through_) {
            imu_pub_->publish(*msg);
            return;
        }

        auto out = *msg;
        sample_count_++;
        output_count_++;

        if (has_last_stamp_) {
            const double dt = (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds();
            if (dt <= 0.0 || dt > 0.05) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "IMU stamp dt abnormal: %.6f s, VQF still uses fixed dt %.6f s",
                    dt, initial_dt_);
            }
        }
        last_stamp_ = rclcpp::Time(msg->header.stamp);
        has_last_stamp_ = true;

        const auto& gyr_msg = msg->angular_velocity;
        const auto& acc_msg = msg->linear_acceleration;
        const double acc_norm = std::sqrt(
            acc_msg.x * acc_msg.x + acc_msg.y * acc_msg.y + acc_msg.z * acc_msg.z);
        if (!finiteVector(gyr_msg) || !finiteVector(acc_msg) || acc_norm < 1e-6) {
            bad_sample_count_++;
            if (has_last_quat_) {
                out.orientation.w = last_quat_[0];
                out.orientation.x = last_quat_[1];
                out.orientation.y = last_quat_[2];
                out.orientation.z = last_quat_[3];
            }
            imu_pub_->publish(out);
            return;
        }

        const vqf_real_t gyr[3] = {gyr_msg.x, gyr_msg.y, gyr_msg.z};
        const vqf_real_t acc[3] = {acc_msg.x, acc_msg.y, acc_msg.z};
        vqf_->update(gyr, acc);

        vqf_real_t quat[4];
        vqf_->getQuat6D(quat);
        last_quat_[0] = quat[0];
        last_quat_[1] = quat[1];
        last_quat_[2] = quat[2];
        last_quat_[3] = quat[3];
        has_last_quat_ = true;

        out.orientation.w = quat[0];
        out.orientation.x = quat[1];
        out.orientation.y = quat[2];
        out.orientation.z = quat[3];
        imu_pub_->publish(out);

        const auto now = this->now();
        if (!has_last_log_ || (now - last_log_time_).seconds() >= log_period_) {
            const uint64_t sample_delta = sample_count_ - last_log_sample_count_;
            const uint64_t output_delta = output_count_ - last_log_output_count_;
            const uint64_t bad_delta = bad_sample_count_ - last_log_bad_sample_count_;
            last_log_time_ = now;
            has_last_log_ = true;
            last_log_sample_count_ = sample_count_;
            last_log_output_count_ = output_count_;
            last_log_bad_sample_count_ = bad_sample_count_;
            RCLCPP_INFO(
                this->get_logger(),
                "VQF monitor: input=%lu output=%lu bad=%lu total_input=%lu total_output=%lu acc_norm=%.3f quat=[%.6f %.6f %.6f %.6f]",
                sample_delta, output_delta, bad_delta, sample_count_, output_count_, acc_norm,
                out.orientation.w, out.orientation.x, out.orientation.y, out.orientation.z);
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    double tau_acc_;
    double initial_dt_;
    bool pass_through_;
    double log_period_;
    std::unique_ptr<VQF> vqf_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Time last_stamp_;
    bool has_last_stamp_ = false;
    rclcpp::Time last_log_time_;
    bool has_last_log_ = false;
    vqf_real_t last_quat_[4] = {1.0, 0.0, 0.0, 0.0};
    bool has_last_quat_ = false;
    uint64_t sample_count_ = 0;
    uint64_t output_count_ = 0;
    uint64_t bad_sample_count_ = 0;
    uint64_t last_log_sample_count_ = 0;
    uint64_t last_log_output_count_ = 0;
    uint64_t last_log_bad_sample_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuVqfFilter>());
    rclcpp::shutdown();
    return 0;
}
