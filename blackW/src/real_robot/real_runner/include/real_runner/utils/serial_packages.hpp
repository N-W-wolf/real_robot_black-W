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
#include <atomic>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "utils/set_zero.h"
#include "utils/secure_protect.hpp"
const int legNum = 4;
const int jointNum = 3;
const int motorPerLeg = 4;
const int wheelJointIndex = 3;
const int rosMotorNum = legNum * motorPerLeg;

inline int rosIndex(int legIndex, int jointIndex) {
    return legIndex * motorPerLeg + jointIndex;
}

inline int motorId(int legIndex, int jointIndex) {
    return legIndex * jointNum + jointIndex;
}

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
            if(i==0) legPorts_[i] = std::make_unique<SerialPort>(portName0, 16, 4000000, 5000);
            else if(i==1) legPorts_[i] = std::make_unique<SerialPort>(portName1, 16, 4000000, 5000);
            else if(i==2) legPorts_[i] = std::make_unique<SerialPort>(portName2, 16, 4000000, 5000);
            else if(i==3) legPorts_[i] = std::make_unique<SerialPort>(portName3, 16, 4000000, 5000);
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
        for(int i=0;i<legNum;i++){
            {
                std::lock_guard<std::mutex> lock_cmd(cmdMutex_[i]);
                
                // 使用 if-else 避免不必要的计算
                if(!utils::safeok()){
                    // 如果不安全，直接填阻尼，防止还在计算高力矩
                    for(int j=0;j<motorPerLeg;j++){
                        motorCmdBuf_[i][j].id = motorId(i, j);
                        motorCmdBuf_[i][j].mode = 1;
                        motorCmdBuf_[i][j].K_P = 0.0;
                        motorCmdBuf_[i][j].K_W = (j == wheelJointIndex) ? 6.0 : 6.0/gear_ratio_squared_[j];
                        motorCmdBuf_[i][j].T   = 0.0;
                        motorCmdBuf_[i][j].Pos = 0.0;
                        motorCmdBuf_[i][j].W   = 0.0;
                    }
                }
                else
                {
                    // 安全时才计算控制律
                    for(int j=0;j<jointNum;j++){
                        const int ros_id = rosIndex(i, j);
                        const int motor_id = motorId(i, j);
                        float q = cmd.robotCmd.motor_command[ros_id].q;
                        float dq = cmd.robotCmd.motor_command[ros_id].dq;
                        float tau = cmd.robotCmd.motor_command[ros_id].tau;
                        if(j==2){
                            // gear_ratio_=6.33*2.5;
                            // gear_ratio_squared_=gear_ratio_*gear_ratio_;
                            q = -q;
                            dq = -dq;
                            tau = -tau;
                        }
                        // else{
                        //     gear_ratio_=6.33;
                        //     gear_ratio_squared_=gear_ratio_*gear_ratio_;
                        // }
                        motorCmdBuf_[i][j].id = motor_id;
                        motorCmdBuf_[i][j].mode = 1;
                        motorCmdBuf_[i][j].K_P = cmd.robotCmd.motor_command[ros_id].kp / gear_ratio_squared_[j];
                        motorCmdBuf_[i][j].K_W = cmd.robotCmd.motor_command[ros_id].kd / gear_ratio_squared_[j];
                    
                        float target_pos = q * gear_ratio_[j];
                        if(is_offset_initialized_[i]){
                            target_pos -= motor_zero_.get_offset_(motor_id);
                        }

                        if(std::abs(motorDataBuf_[i][j].Pos - target_pos) > 8*M_PI && not_first_command&&std::abs(motorCmdBuf_[i][j].K_P)>0.0001)
                        {
                            utils::SafetyStateManager::getInstance().setIsSafe(false);
                            RCLCPP_ERROR(rclcpp::get_logger("SERIAL_PACKAGES"), "%d 号电机转动幅度过大：%.4f", 
                                         motor_id, motorDataBuf_[i][j].Pos - target_pos);

                            return;
                        }

                        motorCmdBuf_[i][j].Pos = target_pos;
                        motorCmdBuf_[i][j].W = dq * gear_ratio_[j];
                        motorCmdBuf_[i][j].T = tau / gear_ratio_[j];

                        // 初始化保护
                        if(!is_offset_initialized_[i]){
                            motorCmdBuf_[i][j].K_P = 0.0;
                            motorCmdBuf_[i][j].K_W = 0.0;
                            motorCmdBuf_[i][j].T   = 0.0;
                            motorCmdBuf_[i][j].Pos = 0.0; 
                            motorCmdBuf_[i][j].W   = 0.0;
                        }
                    }

                    const int wheel_ros_id = rosIndex(i, wheelJointIndex);
                    motorCmdBuf_[i][wheelJointIndex].id = motorId(i, wheelJointIndex);
                    motorCmdBuf_[i][wheelJointIndex].mode = 1;
                    motorCmdBuf_[i][wheelJointIndex].K_P = 0.0;
                    motorCmdBuf_[i][wheelJointIndex].K_W = cmd.robotCmd.motor_command[wheel_ros_id].kd;
                    motorCmdBuf_[i][wheelJointIndex].T = 0.0;
                    motorCmdBuf_[i][wheelJointIndex].Pos = 0.0;
                    motorCmdBuf_[i][wheelJointIndex].W = -cmd.robotCmd.motor_command[wheel_ros_id].dq;

                    if(!is_offset_initialized_[i]){
                        motorCmdBuf_[i][wheelJointIndex].K_W = 0.0;
                        motorCmdBuf_[i][wheelJointIndex].W = 0.0;
                    }
                }
            }
            
            {
                std::lock_guard<std::mutex> lock_state(stateMutex_[i]);
                for(int j=0;j<jointNum;j++){
                    const int ros_id = rosIndex(i, j);
                    const int motor_id = motorId(i, j);
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
                        raw_pos += motor_zero_.get_offset_(motor_id);
                    }
                    //printf("Motor %d Corrected Pos: %f\n", j + i*jointNum, raw_pos);

                    if(j == 2)
                    {
                        state.motorState.motor_state[ros_id].q = -(raw_pos / gear_ratio_[j]);
                        state.motorState.motor_state[ros_id].dq = -motorDataBuf_[i][j].W / gear_ratio_[j];
                        state.motorState.motor_state[ros_id].tau_est = -motorDataBuf_[i][j].T * gear_ratio_[j];
                    }
                    else
                    {
                        state.motorState.motor_state[ros_id].q = raw_pos / gear_ratio_[j];
                        state.motorState.motor_state[ros_id].dq = motorDataBuf_[i][j].W / gear_ratio_[j];
                        state.motorState.motor_state[ros_id].tau_est = motorDataBuf_[i][j].T * gear_ratio_[j];
                    }

                    //printf("Motor %d Output Pos: %f\n", j + i*jointNum, state.motorState.motor_state[j + i*jointNum].q);
                    // state.motorState.motor_state[j + i*jointNum].dq = motorDataBuf_[i][j].W / gear_ratio_[j];
                    // state.motorState.motor_state[j + i*jointNum].tau_est = motorDataBuf_[i][j].T * gear_ratio_[j];
                    state.motorState.motor_state[ros_id].ddq = 0;
                    state.motorState.motor_state[ros_id].cur = 0;
                }

                const int wheel_ros_id = rosIndex(i, wheelJointIndex);
                state.motorState.motor_state[wheel_ros_id].q = 0.0;
                state.motorState.motor_state[wheel_ros_id].dq = -motorDataBuf_[i][wheelJointIndex].W;
                state.motorState.motor_state[wheel_ros_id].ddq = 0.0;
                state.motorState.motor_state[wheel_ros_id].tau_est = -motorDataBuf_[i][wheelJointIndex].T;
                state.motorState.motor_state[wheel_ros_id].cur = 0.0;
            }
        }
    }
    bool not_first_command;
private:
    bool _initWheelMotor(int legIndex, SerialPort &port, std::vector<MotorData> &motorDataGroup){
        std::vector<MotorCmd> wheelCmd(1);
        std::vector<MotorData> wheelState(1);
        const int wheel_id = motorId(legIndex, wheelJointIndex);

        wheelCmd[0].id = wheel_id;
        wheelCmd[0].mode = 1;
        wheelCmd[0].K_P = 0.0;
        wheelCmd[0].K_W = 0.0;
        wheelCmd[0].T = 0.0;
        wheelCmd[0].Pos = 0.0;
        wheelCmd[0].W = 0.0;

        if(!port.sendRecv(wheelCmd, wheelState) || wheelState.empty()){
            RCLCPP_WARN(rclcpp::get_logger("SerialPack"), "Wheel motor init failed on leg %d, id %d. Retrying...", legIndex, wheel_id);
            return false;
        }

        if(!wheelState[0].correct){
            RCLCPP_WARN(rclcpp::get_logger("SerialPack"), "Wheel motor init got invalid response on leg %d, id %d. Retrying...", legIndex, wheel_id);
            return false;
        }

        if(static_cast<int>(wheelState[0].motor_id) != wheel_id){
            RCLCPP_WARN(rclcpp::get_logger("SerialPack"), "Wheel motor init id mismatch on leg %d: expected %d, got %d. Retrying...",
                        legIndex, wheel_id, static_cast<int>(wheelState[0].motor_id));
            return false;
        }

        {
            std::lock_guard<std::mutex> lock_state(stateMutex_[legIndex]);
            if(motorDataGroup.size() < motorPerLeg){
                motorDataGroup.resize(motorPerLeg);
            }
            motorDataGroup[wheelJointIndex] = wheelState[0];
        }

        RCLCPP_INFO(rclcpp::get_logger("SerialPack"), "Wheel motor initialized: leg=%d, id=%d", legIndex, wheel_id);
        return true;
    }

    void _sendRecvMotorGroup(std::vector<MotorCmd> &motorCmdGroup_, std::vector<MotorData> &motorDataGroup_,
                        int legIndex, SerialPort &port){
    std::vector<MotorCmd> localCmd;
    std::vector<MotorData> localState;

    int last_motor2_error = -1;

    // 先分配空间防止 sendRecv 失败
    localCmd.resize(motorPerLeg);
    localState.resize(motorPerLeg);
    while(running_){
        // 1. 初始化 Offset (保持不变)
        if(!is_offset_initialized_[legIndex]){
            // A. 先准备一组“空指令”（零力矩），仅用于获取状态
            // 【修正点】：必须使用索引循环，显式赋值 id
            for(int j = 0; j < motorPerLeg; j++){
                localCmd[j].id = motorId(legIndex, j);
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
                
                if(!_initWheelMotor(legIndex, port, motorDataGroup_)){
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                is_offset_initialized_[legIndex] = true;
                RCLCPP_INFO(rclcpp::get_logger("SerialPack"), "Motor Offsets Initialized!");
            } else {
                // 如果通信失败，不标记为初始化成功，下一轮循环重试
                RCLCPP_WARN(rclcpp::get_logger("SerialPack"), "Failed to read initial motor state, retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
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
            for(int j = 0; j < static_cast<int>(localCmd.size()); j++){
                localCmd[j].mode = 1;
                localCmd[j].K_P = 0.0;
                localCmd[j].K_W = (j == wheelJointIndex) ? 6.0 : 6.0 / gear_ratio_squared_[j]; 
                localCmd[j].T   = 0.0;
                localCmd[j].Pos = 0.0; 
                localCmd[j].W   = 0.0;
            }
        }
        // ============================================================
        localState.resize(localCmd.size()); 
        if(!port.sendRecv(localCmd, localState)){ 
            //utils::SafetyStateManager::getInstance().setIsSafe(false);
            RCLCPP_ERROR(rclcpp::get_logger("SerialPack"), "sendRecv failed"); 
        }
        else if(legIndex == 0 && localState.size() > 2 && localState[2].correct) {
            int err = localState[2].MError;
            if(err != last_motor2_error) {
                last_motor2_error = err;
                RCLCPP_ERROR(rclcpp::get_logger("SerialPack"),"motor 2 MError=%d temp=%d",err, localState[2].Temp);
            }
        }
        {
            std::lock_guard<std::mutex> lock_state(stateMutex_[legIndex]);
            motorDataGroup_.assign(localState.begin(), localState.end());
        }
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
    std::vector<std::vector<MotorCmd>> motorCmdBuf_{legNum, std::vector<MotorCmd>(motorPerLeg)};
    std::vector<std::vector<MotorData>> motorDataBuf_{legNum, std::vector<MotorData>(motorPerLeg)};
    std::atomic<bool> running_{true};
    bool safe_ = true;
};



#endif // __SERIAL_PACKAGES_HPP__
