/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-11-07 22:34:32
 * @FilePath: /ros2-imu-dirve/src/AgRosMsgPub/AB5454Pub.cpp
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 23:20:13
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */
#include "AgRosNode.h"
#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace AG{
namespace ROSNode{
    void AgRosNode::cbkParserAb5465(AB5465DataT &data)
    {
        tf2::Quaternion curr_quater;
        curr_quater.setRPY(tf2Radians(data.Roll), tf2Radians(-data.Pitch), tf2Radians(-data.Yaw));
        mRosImuMsg.orientation.x = curr_quater.x();
        mRosImuMsg.orientation.y = curr_quater.y();
        mRosImuMsg.orientation.z = curr_quater.z();
        mRosImuMsg.orientation.w = curr_quater.w();
        
        /* gx gy gz , ax ay az */
        // 乘以 PI/180 转为弧度
        double deg2rad = M_PI / 180.0;
        mRosImuMsg.angular_velocity.x = data.GyroX * deg2rad;
        mRosImuMsg.angular_velocity.y = -data.GyroY * deg2rad;
        mRosImuMsg.angular_velocity.z = -data.GyroZ * deg2rad;
        mRosImuMsg.linear_acceleration.x = data.AccelX;
        mRosImuMsg.linear_acceleration.y = -data.AccelY;
        mRosImuMsg.linear_acceleration.z = -data.AccelZ;
        pubROS2ImuMsg(mRosImuMsg);
    }
}   //namespace ROSNode
}   //namespace AG
