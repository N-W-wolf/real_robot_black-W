#include <chrono>
#include "AgRosNode.h"
#include <utility>
#include "TinyLog.h"

#include <ctime>
#include <chrono>
#include <filesystem>
namespace AG{
namespace ROSNode{
const std::string copyright = \
"\n*******************************************************************************\n\
                    Copyright (C) Asensing (2025).                              \n\
All rights reserved. This software is Asensing property. Duplication or         \n\
disclosure without Asensing written authorization is prohibited.                \n\
********************************************************************************\n\
@Node Description   : ROS2 node dirver for Asensing Module                      \n\
@Author             : yfsw email:yfsw@asensing.com                              \n\
@Version            : VA1.01.01.00.00.00                                        \n\
@Release_Date       : 2025-11-07                                                \n\
********************************************************************************";

AgRosNode::AgRosNode() : Node("AgROS")
{

}
void AgRosNode::initImuNode()
{
    AGLOGI("AgROS");

    /* all parameter declare and init here */
    this->declare_parameter<int>("ConnectionType", 1);
    /* serial port */
    this->declare_parameter<std::string>("UART_Port", UART_PORT);
    this->declare_parameter<int>("UART_Baudrate", UART_BAUDRATE);
    /* UDP */
    this->declare_parameter<std::string>("UDP_Addr", "192.168.225.2");
    this->declare_parameter<int>("UDP_Port", 12300);
    this->declare_parameter<int>("USB_LatencyTime" , 16);
    /* log */
    this->declare_parameter<std::string>("mLogPath", ".");
    this->declare_parameter<int>("LogLevel", 1);
    /* range */
    this->declare_parameter<float>("Grange04", 250.0);
    this->declare_parameter<float>("Arange04", 4.0);
    this->declare_parameter<float>("Grange0B", 4.0);
    this->declare_parameter<float>("Arange0B", 4.0);
    this->declare_parameter<int>("IMUFreqFactor", 1);
    this->declare_parameter<int>("GPSFreqFactor", 5);
    this->declare_parameter<int>("OdomFreqFactor", 5);
    this->declare_parameter<int>("SpdFreqFactor", 5);

    /* init parameter */
    this->get_parameter("ConnectionType" , mConnectionType);
    this->get_parameter("UART_Port",mSerialName);
    this->get_parameter("UART_Baudrate",mSerialBaud);
    this->get_parameter("UDP_Addr" , mUdpAddr);
    this->get_parameter("UDP_Port" , mUdpPort);
    this->get_parameter("USB_LatencyTime" , mUsbLatencyTime);

    /* set log config */
    std::string Path = "";
    int32_t logLevel = 1;
    this->get_parameter("mLogPath",Path);
    std::filesystem::path p = Path;
    if(p.string().back() == '/')
    {
        mLogPath = Path + "AgROSLog";
    }
    else
    {
        mLogPath = Path + "/AgROSLog";
    }
    try
    {
        if(!std::filesystem::exists(mLogPath))
        {
            std::filesystem::create_directories(mLogPath);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout << "error mLogPath: " << mLogPath << std::endl;
        mLogPath = "./AgROSLog";
        std::cout << "reset Log path: " << mLogPath << std::endl;
        if(!std::filesystem::exists(mLogPath))
        {
            std::filesystem::create_directories(mLogPath);
        }
    }    
    this->get_parameter("LogLevel",logLevel);
    std::cout << "Curr mLogPath: " << mLogPath << std::endl;
    TinyLog::setStorageDir((mLogPath+"/AgROSRun.log").c_str());
    TinyLog::setStorageLevel(logLevel);
    //AGLOGI("%s",copyright.c_str());
    if(logLevel == 0)
    {
        mIsPrintLog = true;
        auto now = std::chrono::system_clock::now();
        auto timeT = std::chrono::system_clock::to_time_t(now - std::chrono::hours(24));
        std::tm *ptm = std::localtime(&timeT);
        std::ostringstream oss;
        oss << std::put_time(ptm, "%Y%m%d-%H%M%S");
        std::string filename = mLogPath + std::string("/AgROSDatAGLOG-") + oss.str() + std::string(".log");
        AGLOGI("IMU Log: %s",filename.c_str());
        mLogFileFd = fopen(filename.c_str(), "wb+");
        if (mLogFileFd == nullptr)
        {
            AGLOGE("open log path fail,please check your launch file!");
        }
    }

    if(0 == mConnectionType)
    {
        openSerialPort();
    }
    else if(1 == mConnectionType)
    {
        bindUdp();
    }


    mFrameIdImu = 0;

    // 注册IMU协议
    auto ab5465Proto = std::make_shared<AB5465>();
    ab5465Proto->setCbk(std::bind(&AgRosNode::cbkParserAb5465,this,std::placeholders::_1));
    mProtoEngine.addProtocol(ab5465Proto);
    mProtoEngine.start();
    mSerialDataSum = 0;
    mUDPDataSum = 0;
    mPeriod1msCnt = 0;
    mRunTime1s = 0;  
    mMonitorImuCnt = 0;
    mFrameIdImu = 0;

    mRos2Timer_ = this->create_wall_timer(1ms, std::bind(&AgRosNode::mRos2Timer_callback, this));

}

void AgRosNode::mRos2Timer_callback()
{
    if(0 == mConnectionType)
    {
        readSerialPort();
    }
    else if(1 == mConnectionType)
    {
        readUdp();
    }
    period1msMonitor();
}
void AgRosNode::openSerialPort()
{
    try
    {
        if((mUsbLatencyTime < 16) && (mUsbLatencyTime >= 1))
        {
            char command[128] = {0};
            sprintf(command , "echo %d > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer" , mUsbLatencyTime);
            system("sudo chmod 777 /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");
            system((const char*)command);
            system("sync");
        }

        mSerialDev.setPort(mSerialName);
        mSerialDev.setBaudrate(mSerialBaud);
        serial::Timeout to = serial::Timeout::simpleTimeout(0);
        mSerialDev.setTimeout(to);
        mSerialDev.setFlowcontrol(serial::flowcontrol_t::flowcontrol_hardware);
        mSerialDev.open();
    }
    catch (serial::IOException& e)
    {
        AGLOGE("mSerialDevial opened failed! please check your connection!");
    }

    if (mSerialDev.isOpen())
    {
        AGLOGI("mSerialDevial opened successfully!");
    }
    else 
    {
        static bool isFirstOpen = true;
        if(isFirstOpen)
        {
            isFirstOpen = false;
            exit(0);
        }
    }
}
void AgRosNode::readSerialPort()
{
    try
    {
        if (mSerialDev.isOpen())
        {
            if(mSerialDev.available())
            {
                std::vector<uint8_t> _HandleBuff;
                auto _SerialReads = mSerialDev.read(mSerialDev.available());
                _HandleBuff.clear();
                _HandleBuff.insert(_HandleBuff.end(),_SerialReads.c_str(),_SerialReads.c_str()+_SerialReads.size());
                mProtoEngine.processData(_HandleBuff);
                if(mIsPrintLog)
                {
                    fwrite(_HandleBuff.data() , _HandleBuff.size() , 1u , mLogFileFd);
                }
                mSerialDataSum+=_SerialReads.size();
            }
        }
        else
        {
            openSerialPort();
        }
    }
    catch (serial::IOException& e)
    {
        AGLOGE("Error reading from the serial port:%s close port!" , mSerialDev.getPort().c_str());
        mSerialDev.close();
    }
}

void AgRosNode::bindUdp()
{
    mSocketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if(mSocketFd < 0)
    {
        AGLOGE("open socket error! %s" , strerror(errno));
        return;
    }
    (void)fcntl(mSocketFd, F_SETFL, fcntl(mSocketFd, F_GETFL, 0)|O_NONBLOCK);
    
    mServer.sin_family = AF_INET;
    mServer.sin_addr.s_addr = inet_addr(mUdpAddr.c_str());
    mServer.sin_port = htons(mUdpPort);
    
    if(bind(mSocketFd, (sockaddr *)&mServer, sizeof(sockaddr)) < 0)
    {
        ::close(mSocketFd);
        AGLOGE("UDP bind %s:%d fail! %s" ,mUdpAddr.c_str() , mUdpPort, strerror(errno));
        mIsBind = false;
    }
    else
    {
        mIsBind = true;
        AGLOGI("UDP bind %s:%d success!" , mUdpAddr.c_str() , mUdpPort);
    }
}
void AgRosNode::readUdp()
{
    if(mIsBind)
    {
        mSockaddrLen = sizeof(sockaddr);
        int readLen = 0;
        uint8_t mReadBuffer[1024];
        std::vector<uint8_t> _HandleBuff;
        if(mSockaddrLen)
        {
            do
            {
                memset(mReadBuffer , 0 , sizeof(mReadBuffer));
                _HandleBuff.clear();
                readLen = recvfrom(mSocketFd , mReadBuffer, 1024*sizeof(char), 0, (sockaddr *)&mServer, &mSockaddrLen);
                if (readLen > 0)
                {
                    _HandleBuff.insert(_HandleBuff.end(),mReadBuffer,mReadBuffer+readLen);
                    mProtoEngine.processData(_HandleBuff);
                    if(mIsPrintLog)
                    {
                        fwrite(mReadBuffer , readLen , 1u , mLogFileFd);
                    }
                    mUDPDataSum += readLen; 
                }
            }while (readLen > 0);
        }
    }
}
void AgRosNode::period1msMonitor()
{
    mPeriod1msCnt++;
    if(mPeriod1msCnt>=1000)
    {
        mPeriod1msCnt = 0;
        mRunTime1s++;
        RCLCPP_INFO(
            this->get_logger(),
            "IMU monitor: runtime=%lu serial_bytes=%lu udp_bytes=%lu imu_frames=%lu",
            mRunTime1s,
            mSerialDataSum,
            mUDPDataSum,
            mMonitorImuCnt);
        if (mMonitorImuCnt == 0)
        {
            AGLOGW("IMU Data Lost Warning!");
        }
        mSerialDataSum = 0;
        mUDPDataSum = 0;
        mMonitorImuCnt = 0;
    }
}
}   //namespace ROSNode
}   //namespace AG
