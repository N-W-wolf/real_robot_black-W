#ifndef LOW_LEVEL_STATE_H
#define LOW_LEVEL_STATE_H

#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serialPort/SerialPort.h"

struct LowLevelState{
public:
    LowLevelState(int motor_num=12){
        motorState.motor_state.resize(motor_num);
    }
    robot_msgs::msg::RobotState motorState;
    sensor_msgs::msg::Imu imu;
};


#endif
