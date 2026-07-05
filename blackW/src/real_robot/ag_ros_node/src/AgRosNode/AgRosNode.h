/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-11-07 16:51:18
 * @FilePath: /ros2-imu-dirve/src/AgRosNode/AgRosNode.hpp
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 23:19:47
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */
#ifndef _AgROS_HPP_
#define _AgROS_HPP_

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <memory>
#include <cstring>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <serial/serial.h>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
/* ros2 msg */
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "AgProtocol.h"
#include "AB5465.h"

namespace AG{
namespace ROSNode{

using namespace std;
using namespace std::chrono_literals;
using namespace serial;


#define UART_PORT       "/dev/ttyUSB0"
#define UART_BAUDRATE   (115200)

class AgRosNode : public rclcpp::Node
{
public:
    AgRosNode();
    void initImuNode();
        
private:
    void mRos2Timer_callback();

    void openSerialPort();
    void readSerialPort();

    void bindUdp();
    void readUdp();

    void period1msMonitor();

private:
    /* serial port info */
    serial::Serial mSerialDev;
    std::string mSerialName;
    int mSerialBaud;
    int mUsbLatencyTime;

    /* UDP info */
    int mSocketFd;
    std::string mUdpAddr;
    int mUdpPort;
    struct sockaddr_in mServer;
    socklen_t mSockaddrLen;
    bool mIsBind;
        
    int mConnectionType;
    rclcpp::TimerBase::SharedPtr mRos2Timer_;

    /* log */
    bool mIsPrintLog = false;
    FILE* mLogFileFd = nullptr;

    std::string mLogPath;

    AgProtocolEngine mProtoEngine;
    uint64_t    mSerialDataSum;
    uint64_t    mUDPDataSum;
    uint64_t    mPeriod1msCnt;
    uint64_t    mRunTime1s;

    uint64_t    mMonitorImuCnt;


public:
    void pubROS2ImuMsg(sensor_msgs::msg::Imu &imuRosMsg);
    void cbkParserAb5465(AB5465DataT &data);
private:
    /* topic info */
    const std::string mTopicImu = "/_lowState/imu_raw";
    uint64_t mFrameIdImu;
    sensor_msgs::msg::Imu mRosImuMsg;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mPubImuPtr;
    bool mIsImuPubCreate;
};
}   //namespace ROSNode
}   //namespace AG
#endif /* _AgROS_HPP_ */
