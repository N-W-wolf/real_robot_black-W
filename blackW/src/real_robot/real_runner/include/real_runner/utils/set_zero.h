#ifndef SET_ZERO_H
#define SET_ZERO_H

#include "message/LowLevelCmd.hpp"
#include "message/LowLevelState.hpp"
#include "serialPort/SerialPort.h"
#include <vector>
#include <string>
#include <fstream> 
#include <iostream>

#define M_PI 3.14159265358979323846
const int leg_num = 4;
const int joint_num = 3;
const int bus_motor_num = 4;
const int motor_num = leg_num * joint_num;

// 定义校准文件保存路径
const std::string CALIBRATION_FILE = "./src/real_robot/real_runner/motor_calibration.conf";

class motor_zero{
public:
    motor_zero() : 
        // 初始化蹲伏姿态
        creep_position_(motor_num, 0.0),
        // 初始化站立姿态
        straight_position_(motor_num, 0.0),
        // 初始化补偿量
        off_set_(motor_num, 0.0)
    {
        // 启动时尝试自动加载配置文件
        load_calibration_file();
        offset_calf = 46.66*M_PI/180*6.33*2.5;

    }
    ~motor_zero(){}
    void get_motor_offset(const std::vector<MotorData> &motor_state,int leg_id);
    void process_offset_command(LowLevelCmd &cmd);
    void process_offset_state(LowLevelState &state);
    void record_position();
    // 保存和加载函数
    void save_calibration_file();
    bool load_calibration_file();

    // double get_offset_(int motor_id) {
    //     return off_set_[motor_id] - straight_position_[motor_id];
    // }
    double get_offset_(int motor_id) {
        double retval=off_set_[motor_id] - straight_position_[motor_id];
        if(motor_id==2||motor_id==8){
            retval -= offset_calf;
        }
        else if(motor_id==5||motor_id==11){
            retval += offset_calf;
        }
        return retval;
    }     

private:
    std::vector<double> creep_position_;
    std::vector<double> straight_position_;
    std::vector<double> off_set_;//针对state的offset
    double offset_calf;
};  


#endif // SET_ZERO_H
