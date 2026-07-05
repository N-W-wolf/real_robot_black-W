#ifndef __SERIAL_PACKAGES_HPP__
#define __SERIAL_PACKAGES_HPP__

#include "serialPort/SerialPort.h"
#include "message/LowLevelCmd.hpp"
#include "message/LowLevelState.hpp"
#include <vector>
#include <string>
#include <fstream> 
#include <iostream>
#include <thread>
#include <mutex>
#include <cstring>
#include <array>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "utils/set_zero.h"
#include "utils/secure_protect.hpp"
const int legNum = 4;
const int jointNum = 3;

class SerialPack{
public:
    // SerialPack(std::string portName0, std::string portName1, std::string portName2, std::string portName3):
    // legPorts_{SerialPort(portName0), SerialPort(portName1), SerialPort(portName2), SerialPort(portName3)}
    // SerialPack(std::string portName0) {
    //     legPorts_.resize(legNum);
    //     for(int i=0;i<legNum;i++){
    //         legPorts_[i] = std::make_unique<SerialPort>(portName0);
    //     }
    //     for(int i=0;i<legNum;i++){
    //         legThreads[i] = std::thread(&SerialPack::_sendRecvMotorGroup, this,std::ref(motorCmdBuf_[i]), std::ref(motorDataBuf_[i]),i,std::ref(*legPorts_[i]));
    //     }
    // }
    SerialPack(std::string portName0, std::string portName1 ,std::string portName2, std::string portName3) {
        legPorts_.resize(legNum);
        for(int i=0;i<legNum;i++){
            if(i==0) legPorts_[i] = std::make_unique<SerialPort>(portName0);
            else if(i==1) legPorts_[i] = std::make_unique<SerialPort>(portName1);
            else if(i==2) legPorts_[i] = std::make_unique<SerialPort>(portName2);
            else if(i==3) legPorts_[i] = std::make_unique<SerialPort>(portName3);
        }
        for(int i=0;i<legNum;i++){
            legThreads[i] = std::thread(&SerialPack::_sendRecvMotorGroup, this,std::ref(motorCmdBuf_[i]), std::ref(motorDataBuf_[i]),i,std::ref(*legPorts_[i]));
        }
        gear_ratio_={6.33,6.33,6.33*2.5};
        gear_ratio_squared_={6.33*6.33,6.33*6.33,6.33*6.33*2.5*2.5};
        not_first_command = false;
    }

    ~SerialPack(){
        running_=false;
        for(int i=0;i<legNum;i++){
            legThreads[i].join();
        }
    }

    void sendRecv(LowLevelCmd &cmd, LowLevelState &state){
        const auto sendrecv_start = std::chrono::steady_clock::now();
        std::array<double, legNum> cmd_stage_ms{};
        std::array<double, legNum> state_stage_ms{};
        for(int i=0;i<legNum;i++){
            const auto cmd_stage_start = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock_cmd(cmdMutex_[i]);
                
                // 使用 if-else 避免不必要的计算
                if(!utils::safeok()){
                    // 如果不安全，直接填阻尼，防止还在计算高力矩
                    for(int j=0;j<jointNum;j++){
                        motorCmdBuf_[i][j].id = j + i*jointNum;
                        motorCmdBuf_[i][j].mode = 1;
                        motorCmdBuf_[i][j].K_P = 0.0;
                        motorCmdBuf_[i][j].K_W = 6.0/gear_ratio_squared_[j];
                        motorCmdBuf_[i][j].T   = 0.0;
                        motorCmdBuf_[i][j].Pos = 0.0;
                        motorCmdBuf_[i][j].W   = 0.0;
                    }
                }
                else
                {
                    // 安全时才计算控制律
                    for(int j=0;j<jointNum;j++){
                        if(j==2){
                            // gear_ratio_=6.33*2.5;
                            // gear_ratio_squared_=gear_ratio_*gear_ratio_;
                            cmd.robotCmd.motor_command[j + i*jointNum].q = -cmd.robotCmd.motor_command[j + i*jointNum].q;
                            cmd.robotCmd.motor_command[j + i*jointNum].dq = -cmd.robotCmd.motor_command[j + i*jointNum].dq;
                            cmd.robotCmd.motor_command[j + i*jointNum].tau = -cmd.robotCmd.motor_command[j + i*jointNum].tau;
                        }
                        // else{
                        //     gear_ratio_=6.33;
                        //     gear_ratio_squared_=gear_ratio_*gear_ratio_;
                        // }
                        motorCmdBuf_[i][j].id = j + i*jointNum;
                        motorCmdBuf_[i][j].mode = 1;
                        motorCmdBuf_[i][j].K_P = cmd.robotCmd.motor_command[j + i*jointNum].kp / gear_ratio_squared_[j];
                        motorCmdBuf_[i][j].K_W = cmd.robotCmd.motor_command[j + i*jointNum].kd / gear_ratio_squared_[j];
                    
                        float target_pos = cmd.robotCmd.motor_command[j + i*jointNum].q * gear_ratio_[j];
                        if(is_offset_initialized_[i]){
                            target_pos -= motor_zero_.get_offset_(i*jointNum + j);
                        }

                        if(abs(motorDataBuf_[i][j].Pos - target_pos) > 8*M_PI && not_first_command&&std::abs(motorCmdBuf_[i][j].K_P)>0.0001)
                        {
                            utils::SafetyStateManager::getInstance().setIsSafe(false);
                            RCLCPP_ERROR(rclcpp::get_logger("SERIAL_PACKAGES"), "%d 号电机转动幅度过大：%.4f", 
                                         j + i*jointNum, motorDataBuf_[i][j].Pos - target_pos);

                            return;
                        }

                        motorCmdBuf_[i][j].Pos = target_pos;
                        motorCmdBuf_[i][j].W = cmd.robotCmd.motor_command[j + i*jointNum].dq * gear_ratio_[j];
                        motorCmdBuf_[i][j].T = cmd.robotCmd.motor_command[j + i*jointNum].tau / gear_ratio_[j];

                        // 初始化保护
                        if(!is_offset_initialized_[i]){
                            motorCmdBuf_[i][j].K_P = 0.0;
                            motorCmdBuf_[i][j].K_W = 0.0;
                            motorCmdBuf_[i][j].T   = 0.0;
                            motorCmdBuf_[i][j].Pos = 0.0; 
                            motorCmdBuf_[i][j].W   = 0.0;
                        }
                    }
                }
            }
            cmd_stage_ms[i] = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - cmd_stage_start).count();
            
            const auto state_stage_start = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock_state(stateMutex_[i]);
                for(int j=0;j<jointNum;j++){
                    // if(j==2){
                    //     gear_ratio_=6.33*2.5;
                    //     gear_ratio_squared_=gear_ratio_*gear_ratio_;
                    // }
                    // else{
                    //     gear_ratio_=6.33;
                    //     gear_ratio_squared_=gear_ratio_*gear_ratio_;
                    // }
                    float raw_pos = motorDataBuf_[i][j].Pos;
                    //printf("Motor %d Raw Pos: %f\n", j + i*jointNum, raw_pos);
                    if(is_offset_initialized_[i]){
                        raw_pos += motor_zero_.get_offset_(i*jointNum + j);
                    }
                    //printf("Motor %d Corrected Pos: %f\n", j + i*jointNum, raw_pos);

                    if(j == 2)
                    {
                        state.motorState.motor_state[j + i*jointNum].q = -(raw_pos / gear_ratio_[j]);
                        state.motorState.motor_state[j + i*jointNum].dq = -motorDataBuf_[i][j].W / gear_ratio_[j];
                        state.motorState.motor_state[j + i*jointNum].tau_est = -motorDataBuf_[i][j].T * gear_ratio_[j];
                    }
                    else
                    {
                        state.motorState.motor_state[j + i*jointNum].q = raw_pos / gear_ratio_[j];
                        state.motorState.motor_state[j + i*jointNum].dq = motorDataBuf_[i][j].W / gear_ratio_[j];
                        state.motorState.motor_state[j + i*jointNum].tau_est = motorDataBuf_[i][j].T * gear_ratio_[j];
                    }

                    //printf("Motor %d Output Pos: %f\n", j + i*jointNum, state.motorState.motor_state[j + i*jointNum].q);
                    // state.motorState.motor_state[j + i*jointNum].dq = motorDataBuf_[i][j].W / gear_ratio_[j];
                    // state.motorState.motor_state[j + i*jointNum].tau_est = motorDataBuf_[i][j].T * gear_ratio_[j];
                    state.motorState.motor_state[j + i*jointNum].ddq = 0;
                    state.motorState.motor_state[j + i*jointNum].cur = 0;
                }
            }
            state_stage_ms[i] = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - state_stage_start).count();
        }

        const double total_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - sendrecv_start).count();
        static auto last_slow_log = std::chrono::steady_clock::time_point{};
        const auto now = std::chrono::steady_clock::now();
        if (total_ms > 20.0 && now - last_slow_log > std::chrono::seconds(1)) {
            last_slow_log = now;
            RCLCPP_WARN(
                rclcpp::get_logger("SerialPack"),
                "SerialPack::sendRecv slow %.3f ms | cmd=[%.3f %.3f %.3f %.3f] state=[%.3f %.3f %.3f %.3f]",
                total_ms,
                cmd_stage_ms[0], cmd_stage_ms[1], cmd_stage_ms[2], cmd_stage_ms[3],
                state_stage_ms[0], state_stage_ms[1], state_stage_ms[2], state_stage_ms[3]);
        }
    }
    bool not_first_command;
private:
    void _sendRecvMotorGroup(std::vector<MotorCmd> &motorCmdGroup_, std::vector<MotorData> &motorDataGroup_,
                        int legIndex, SerialPort &port){
    std::vector<MotorCmd> localCmd;
	    std::vector<MotorData> localState;
	    // 先分配空间防止 sendRecv 失败
	    localCmd.resize(jointNum);
	    localState.resize(jointNum);
	    const auto motor_loop_period = std::chrono::milliseconds(2);
	    while(running_){
	        const auto loop_start = std::chrono::steady_clock::now();
	        // 1. 初始化 Offset (保持不变)
	        if(!is_offset_initialized_[legIndex]){
            // A. 先准备一组“空指令”（零力矩），仅用于获取状态
            // 【修正点】：必须使用索引循环，显式赋值 id
            for(int j = 0; j < jointNum; j++){
                localCmd[j].id = j + legIndex * jointNum;
                localCmd[j].mode = 1;
                localCmd[j].K_P = 0; 
                localCmd[j].K_W = 0; 
                localCmd[j].T = 0; 
                localCmd[j].Pos = 0; 
                localCmd[j].W = 0;
            }

            // B. 发送并接收一次数据
            if(port.sendRecv(localCmd, localState)){
                // C. 将读取到的硬件状态更新到 motorDataGroup_
                // 先加锁确保线程安全
                {
                    std::lock_guard<std::mutex> lock_state(stateMutex_[legIndex]);
                    motorDataGroup_.assign(localState.begin(), localState.end());
                }

                // D. 现在数据是最新的了，可以计算 Offset 了
                motor_zero_.get_motor_offset(motorDataGroup_,legIndex);
                
                is_offset_initialized_[legIndex] = true;
                RCLCPP_DEBUG(rclcpp::get_logger("SerialPack"), "Motor Offsets Initialized!");
	            } else {
	                // 如果通信失败，不标记为初始化成功，下一轮循环重试
	                RCLCPP_WARN(rclcpp::get_logger("SerialPack"), "Failed to read initial motor state, retrying...");
	                std::this_thread::sleep_for(std::chrono::milliseconds(10));
	            }
	            std::this_thread::sleep_until(loop_start + motor_loop_period);
	            continue;
	        }

        // 2. 复制指令 (保持锁机制)
        {
            std::lock_guard<std::mutex> lock_cmd(cmdMutex_[legIndex]);
            localCmd.assign(motorCmdGroup_.begin(), motorCmdGroup_.end());
        }

        // ============================================================
        // 在这里进行安全检查
        // ============================================================
        if(!utils::safeok()){
            for(auto &cmd : localCmd){
                cmd.mode = 1;
                cmd.K_P = 0.0;
                cmd.K_W = 6.0 / (6.33 * 6.33); 
                cmd.T   = 0.0;
                cmd.Pos = 0.0; 
                cmd.W   = 0.0;
            }
        }
        // ============================================================
        localState.resize(localCmd.size()); 
        if(!port.sendRecv(localCmd, localState)){ 
            //utils::SafetyStateManager::getInstance().setIsSafe(false);
            RCLCPP_ERROR(rclcpp::get_logger("SerialPack"), "sendRecv failed"); 
        }
	        {
	            std::lock_guard<std::mutex> lock_state(stateMutex_[legIndex]);
	            motorDataGroup_.assign(localState.begin(), localState.end());
	        }
	        std::this_thread::sleep_until(loop_start + motor_loop_period);
	    }
	}
    std::vector<std::unique_ptr<SerialPort>> legPorts_;
    std::vector<std::mutex> cmdMutex_{legNum};
    std::vector<std::mutex> stateMutex_{legNum};
    // double gear_ratio_=6.33;
    // double gear_ratio_squared_ = gear_ratio_ * gear_ratio_;
    std::vector<double> gear_ratio_{3};
    std::vector<double> gear_ratio_squared_{3};
    // double gear_ratio_calf = 6.33 * 2.5;
    // double gear_ratio_squaded_calf = gear_ratio_calf * gear_ratio_calf;
    std::vector<std::thread> legThreads{legNum};
    motor_zero motor_zero_;
    std::vector<bool> is_offset_initialized_ = std::vector<bool>(legNum, false);
    std::vector<std::vector<MotorCmd>> motorCmdBuf_{legNum, std::vector<MotorCmd>(jointNum)};
    std::vector<std::vector<MotorData>> motorDataBuf_{legNum, std::vector<MotorData>(jointNum)};
    std::atomic<bool> running_{true};
    bool safe_ = true;
};



#endif // __SERIAL_PACKAGES_HPP__
