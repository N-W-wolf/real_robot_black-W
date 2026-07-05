#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class MiddlewareNode : public rclcpp::Node {
public:
    MiddlewareNode() : Node("MiddlewareNode") {
        command_sub_ = this->create_subscription<robot_msgs::msg::RobotCommand>(
            "/robot_joint_controller/command", 10, std::bind(&MiddlewareNode::commandCallback, this, _1));
        state_pub_ = this->create_publisher<robot_msgs::msg::RobotState>("/robot_joint_controller/state", 10);

        auto imu_qos = rclcpp::SensorDataQoS();
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", imu_qos);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/_lowState/imu", imu_qos, std::bind(&MiddlewareNode::imuCallback, this, _1));
        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            "/_lowState/joint", 10, std::bind(&MiddlewareNode::stateCallback, this, _1));
        command_pub_ = this->create_publisher<robot_msgs::msg::RobotCommand>("/_lowCmd/command", 10);
    }

private:
    void commandCallback(const robot_msgs::msg::RobotCommand::SharedPtr msg) {
        command_pub_->publish(*msg);
    }

    void stateCallback(const robot_msgs::msg::RobotState::SharedPtr msg) {
        state_pub_->publish(*msg);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_pub_->publish(*msg);
    }

    rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr command_sub_;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr command_pub_;
    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiddlewareNode>());
    rclcpp::shutdown();
    return 0;
}
