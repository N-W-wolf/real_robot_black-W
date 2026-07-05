#ifndef __LOW_LEVEL_CMD_HPP__
#define __LOW_LEVEL_CMD_HPP__ 

#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serialPort/SerialPort.h"

struct LowLevelCmd {
public:
    LowLevelCmd(int motor_num=16){
        robotCmd.motor_command.resize(motor_num);
    }
    robot_msgs::msg::RobotCommand robotCmd;
};

#endif
