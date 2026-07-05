#include "utils/set_zero.h"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm> 


void motor_zero::save_calibration_file() 
{
    std::ofstream file(CALIBRATION_FILE);
    if (file.is_open()) 
    {
        for (double val : straight_position_) file << val << " ";
        file << "\n";
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

bool motor_zero::load_calibration_file() 
{
    std::ifstream file(CALIBRATION_FILE);
    if (file.is_open()) 
    {
        for (int i = 0; i < motor_num; i++) {
            if (!(file >> straight_position_[i])) return false;
        }
        for (int i = 0; i < motor_num; i++) {
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

void motor_zero::record_position(){
    // 初始化总线数据容器
    std::vector<MotorCmd> motorCmdBuf(motor_num);
    std::vector<MotorData> motorDataBuf(motor_num);

    std::array<std::unique_ptr<SerialPort>, leg_num> serialPorts;
    for (int i = 0; i < leg_num; ++i) {
        serialPorts[i] = std::make_unique<SerialPort>("/dev/leg_" + std::to_string(i));
    }

    std::vector<std::vector<MotorCmd>> legCmds(leg_num, std::vector<MotorCmd>(joint_num));
    std::vector<std::vector<MotorData>> legDatas(leg_num, std::vector<MotorData>(joint_num));

    std::atomic<bool> set_straight_position{false};
    std::atomic<bool> set_creep_position{false};
    std::atomic<bool> save_flag{false};
    std::atomic<bool> exit_thread{false};


    // 键盘监听线程
    std::thread get_signal([&](){
        char c;
        while(std::cin >> c){
            if(c == 's'){
                set_straight_position = true;
                RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Command: Set STRAIGHT position");
            }
            else if(c == 'c'){
                set_creep_position = true;
                RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Command: Set CREEP position");
            }
            else if(c == 'w'){
                 save_flag = true;
                 RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Command: SAVE to file");
            }
            else if(c == 'm'){
                // 打印当前内存中的校准值
                std::cout << "Straight: ";
                for(auto v : straight_position_) std::cout << v << " ";
                std::cout << "\nCreep: ";
                for(auto v : creep_position_) std::cout << v << " ";
                std::cout << std::endl;
            }
            else if(c == 'q'){
                RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Exiting record thread...");
                exit_thread = true;
                break;
            }
        }
    });
    get_signal.detach();

    std::vector<double> current_position(motor_num, 0.0);

    // 定义一个静态时钟，保证它一直存在，用于控制打印频率
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    // 主循环
    while(!exit_thread){
        // 1. 准备所有电机的指令
        for(int i = 0; i < motor_num; i++){
            motorCmdBuf[i].id = i;
            motorCmdBuf[i].mode = 1;
            motorCmdBuf[i].K_P = 0;
            motorCmdBuf[i].K_W = 0;
            motorCmdBuf[i].Pos = 0;
            motorCmdBuf[i].W = 0;
            motorCmdBuf[i].T = 0;
        }

        for (int leg = 0; leg < leg_num; ++leg) {
            for (int j = 0; j < joint_num; ++j) {
                int motor_id = leg * joint_num + j;
                legCmds[leg][j] = motorCmdBuf[motor_id];
            }
        }

        for (int leg = 0; leg < leg_num; ++leg) {
            if (!serialPorts[leg]->sendRecv(legCmds[leg], legDatas[leg])) {
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("motor_zero"), steady_clock, 1000, "Serial %d Fail", leg);
            }
        }

        for(int leg = 0; leg < leg_num; ++leg) {
            for(int j = 0; j < joint_num; ++j) {
                int motor_id = leg * joint_num + j;
                motorDataBuf[motor_id] = legDatas[leg][j];
            }
        }

        // 5. 更新当前位置记录
        for(int i=0; i<motor_num; i++){
            current_position[i] = motorDataBuf[i].Pos;
        }

        // 6. 处理用户指令
        if(set_straight_position){
            for(int i=0; i<motor_num; i++){
                straight_position_[i] = current_position[i];
            }
            RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Straight position recorded in RAM.");
            set_straight_position = false;
        }
        else if(set_creep_position){
            for(int i=0; i<motor_num; i++){
                creep_position_[i] = current_position[i];
            }
            RCLCPP_INFO(rclcpp::get_logger("motor_zero"), "Creep position recorded in RAM.");
            set_creep_position = false;
        }
        
        // 处理保存到文件的请求
        if(save_flag){
            save_calibration_file();
            save_flag = false;
        }

        // 添加短暂延时，避免 CPU 占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}