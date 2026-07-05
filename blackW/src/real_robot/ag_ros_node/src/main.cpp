/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-11-07 16:51:18
 * @FilePath: /ros2-imu-dirve/src/main.cpp
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 23:35:00
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */
#include "AgRosNode.h"

using namespace AG;
using namespace ROSNode;

int main(int argc, char** argv)
{
    //初始化节点
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgRosNode>();
    node->initImuNode();

    //循环节点
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}