/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-11-07 22:48:49
 * @FilePath: /ros2-imu-dirve/src/AgRosMsgPub/AgRosMsgPub.cpp
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 23:22:09
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */
#include "AgRosNode.h"

namespace AG{
namespace ROSNode{
    
void AgRosNode::pubROS2ImuMsg(sensor_msgs::msg::Imu &imuRosMsg)
{
    if((mPubImuPtr.get()==nullptr) || mIsImuPubCreate == false)
    {
        mIsImuPubCreate = true;
        /* publisher */
        mPubImuPtr = this->create_publisher<sensor_msgs::msg::Imu>(mTopicImu.c_str(), 500);
    }
    mFrameIdImu++;
    mMonitorImuCnt++;
    imuRosMsg.header.frame_id = "imu_link" ;
    imuRosMsg.header.stamp = this->now();
    mPubImuPtr->publish(imuRosMsg);
}

}   //namespace ROSNode
}   //namespace AG


