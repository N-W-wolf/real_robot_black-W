#include "utils/set_zero.h"
#include "rclcpp/rclcpp.hpp"
#include <thread>

// 保存函数实现

void motor_zero::save_calibration_file() 
{
    std::ofstream file(CALIBRATION_FILE);
    if (file.is_open()) 
    {
        // 第一行保存 straight
        for (double val : straight_position_) file << val << " ";
        file << "\n";
        
        // 第二行保存 creep
        for (double val : creep_position_) file << val << " ";
        file << "\n";
        
        file.close();
        RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "校准参数已保存至: %s", CALIBRATION_FILE.c_str());
    } 
    else 
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_zero"), "无法打开文件进行写入: %s", CALIBRATION_FILE.c_str());
    }
}

// 加载函数实现
bool motor_zero::load_calibration_file() 
{
    std::ifstream file(CALIBRATION_FILE);
    if (file.is_open()) 
    {
        for (int i = 0; i < motor_num; i++) 
        {
            if (!(file >> straight_position_[i])) return false;
        }
        for (int i = 0; i < motor_num; i++) 
        {
            if (!(file >> creep_position_[i])) return false;
        }
        file.close();
        RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "成功加载校准文件!");
        return true;
    }
    RCLCPP_WARN(rclcpp::get_logger("motor_zero"), "未找到校准文件，使用默认值(0.0)");
    return false;
}

void motor_zero::get_motor_offset(const std::vector<MotorData> &motor_state,int leg_id)
{
    // for(int i=0;i<1;i++){
    //     double error = std::abs(motor_state[i].Pos - std::fmod(creep_position_[i], 2*M_PI));
    //     if(std::fmod(creep_position_[i], 2*M_PI) > 5/3*M_PI&&error > 5/3*M_PI){
    //         off_set_[i] =  2*M_PI;
    //     }
    //     else if(std::fmod(creep_position_[i], 2*M_PI) < 1/3*M_PI&&error > 5/3*M_PI){
    //         off_set_[i]=  -2*M_PI;
    //     }
    //     else{
    //         off_set_[i] =0;
    //     }
    // }

    // 使用新的逻辑
    for (int i = 0; i < 3; ++i)
    {
        // 获取上电时转子绝对位置
        double current_val = motor_state[i].Pos;

        // 获取标定的 Creep 转子位置
        double calibration_val = creep_position_[i+leg_id*3];

        // 计算直接差值
        double diff  = current_val - calibration_val;
        RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Motor %d Current Pos: %f, Calibration Pos: %f, Diff: %f", i+leg_id*3, current_val, calibration_val, diff);
        // [优化] 计算差了多少个整圈(四舍五入)
        int rounds = std::round(diff / (2 * M_PI));
        RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Motor %d Rounds: %d", i+leg_id*3, rounds);
        // 计算补偿量
        // off_set_ 记录的是：为了对齐到标定相位，需要补偿的值
        off_set_[i+leg_id*3] = -rounds * 2 * M_PI;
        //RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Motor %d Offset set to: %f", i, off_set_[i]);
    }
}

// 不使用这两个函数了
// void motor_zero::process_offset_command(LowLevelCmd &cmd){
//     for(int i=0;i<1;i++){
//         cmd.robotCmd.motor_command[i].q -= get_offset_()[i];
//     }
// }

// void motor_zero::process_offset_state(LowLevelState &state){
//     for(int i=0;i<1;i++){
//         state.motorState.motor_state[i].q += get_offset_()[i];
//     }
// }

void motor_zero::record_position(){
    std::vector<MotorCmd> motorCmdBuf{motor_num};
    std::vector<MotorData> motorDataBuf{motor_num};
    SerialPort serialPort("/dev/leg_0");
    bool set_straight_position = false;
    bool set_creep_position = false;
    bool exit_thread = false;
    std::thread get_signal([&](){
        char c;
        while(std::cin>>c){
            if(c=='s'){
                set_straight_position = true;
                RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "set straight position");
            }
            else if(c=='c'){
                set_creep_position = true;
                RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "set creep position");
            }
            else if(c=='m'){
                RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "straight position:");
                for(int i=0;i<motor_num;i++){
                    RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "%f ", straight_position_[i]);
                }
                RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "creep position:");
                for(int i=0;i<motor_num;i++){
                    RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "%f ", creep_position_[i]);
                }
            }
            else if(c=='q'){
                RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "exit record thread");
                exit_thread = true;
                break;
            }
        }
    });
    get_signal.detach();
    std::vector<double> current_position(motor_num, 0.0);
    while(!exit_thread){
        for(int i(0);i<motor_num;i++){
            motorCmdBuf[i].id=i;
            motorCmdBuf[i].mode=0;
            motorCmdBuf[i].K_P=0;
            motorCmdBuf[i].K_W=0;
            motorCmdBuf[i].Pos=0;
            motorCmdBuf[i].W=0;
            motorCmdBuf[i].T=0;
        }
        if(!serialPort.sendRecv(motorCmdBuf, motorDataBuf)){
            RCLCPP_ERROR(rclcpp::get_logger("motor_zero"), "sendRecv failed");
            continue;
        }
        for(int i=0;i<motor_num;i++){
            current_position[i] = motorDataBuf[i].Pos;
        }
        if(set_straight_position){
            for(int i(0);i<motor_num;i++){
                straight_position_[i] = current_position[i];
            }
            set_straight_position = false;
        }
        else if(set_creep_position){
            for(int i(0);i<motor_num;i++){
                creep_position_[i] = current_position[i];
            }
            set_creep_position = false;
        }
    }
}

// void motor_zero::set_motorZero(const LowLevelState &state){
//     for(int i=0;i<motor_num;i++){
//         float error = std::abs(state.motorState.motor_state[i].q - creep_position[i]);
//         if(creep_position[i] > 5/3*M_PI&&error > 5/3*M_PI){
//             if(error > 5/3*PI){
//                 off_set[i] = 2*M_PI;
//             }else if(error < 1/3*M_PI){
//                 off_set[i]= 0;
//             }
//         }else if(creep_position[i] < 1/3*M_PI){
//             if(error > 5/3*PI){
//                 off_set[i]= - 2*M_PI;
//             }else if(error < 1/3*PI){
//                 off_set[i] =0;
//             }
//         }
//     }
// }
//留存版本，忘记在最开始的时候留一份了，只能先这样了