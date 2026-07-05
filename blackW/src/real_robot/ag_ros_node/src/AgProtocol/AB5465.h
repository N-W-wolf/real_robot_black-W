/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-04-03 14:18:46
 * @FilePath: /ros2-imu-dirve/src/AgProtocol/AB5465.h
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 23:18:03
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */

#include "AgProtocol.h"
#include <iostream>
#include <stdio.h>

namespace AG{
namespace ROSNode{

typedef struct {
	uint32_t	FrameCnt;
    uint8_t		RollingCounter;
    float		Roll;
    float		Pitch;
    float		Yaw;
    float		GyroX;
    float		GyroY;
    float		GyroZ;
    float		AccelX;
    float		AccelY;
    float		AccelZ;
    float		Temperature;
    uint8_t     SelfCheck;
    uint8_t     IMUFlag[6];
    uint64_t    TimeStamp;
}AB5465DataT;

using Ab5465Cbk = std::function<void(AB5465DataT&)>;
using namespace NMEA_Utils;
class AB5465 : public AgProtocolBase {
    #define AB5465_LEN   66
    #define AB5465_ID    0xAB546500
public:
    bool checkHeader(const std::vector<uint8_t>& buffer) const override {
        bool ret = false;
        if(buffer.size() >= 4)
        {
            ret = (buffer[0] == 0xAB && 
                    buffer[1] == 0x54 &&
                    buffer[2] == 0x65 &&
                    buffer[3] == 0x00);
        }
        return ret;
    }

    bool checkFrameComplete(const std::vector<uint8_t>& buffer) const override {
        return buffer.size() >= AB5465_LEN;
    }


    bool validate(const std::vector<uint8_t>& frame) const override {
        if(frame.size() < AB5465_LEN) return false;
        uint16_t crcTmp,crcCal;
        (void)memcpy(&crcTmp,&frame[AB5465_LEN-2],2);
        
        crcCal = NMEA_Utils::crc16_ccitt_false((unsigned char*)&frame[0], (AB5465_LEN-2));
        if(crcCal != crcTmp)
        {
            // AGLOGD("AB54FF crcCal:%04X\t crcTmp:%04X\n",crcCal,crcTmp);
        }
        return (crcCal == crcTmp);
    }


    void parse(const std::vector<uint8_t>& frame) const override {
        AB5465DataT Ab5465T = {};
        uint8_t *pbufferin = (uint8_t *)&frame[0];
        Ab5465T.FrameCnt = GET_U32(pbufferin, 6);
        Ab5465T.RollingCounter  = GET_U8(pbufferin, 10);
        Ab5465T.Roll = GET_FLT(pbufferin, 11);
        Ab5465T.Pitch = GET_FLT(pbufferin, 15);
        Ab5465T.Yaw = GET_FLT(pbufferin, 19);
        Ab5465T.GyroX = GET_FLT(pbufferin, 23);
        Ab5465T.GyroY = GET_FLT(pbufferin, 27);
        Ab5465T.GyroZ = GET_FLT(pbufferin, 31);
        Ab5465T.AccelX = GET_FLT(pbufferin, 35);
        Ab5465T.AccelY = GET_FLT(pbufferin, 39);
        Ab5465T.AccelZ = GET_FLT(pbufferin, 43);
        Ab5465T.Temperature = GET_S16(pbufferin, 47) * (200/32768.0);
        Ab5465T.SelfCheck = GET_U8(pbufferin, 49);
        memcpy(Ab5465T.IMUFlag, pbufferin+50, 6);
        Ab5465T.TimeStamp = GET_U64(pbufferin, 56);

        AGLOGD("Ab5465: %d %d %f %f %f %f %f %f %f %f %f %f %02X %02X%02X%02X%02X%02X%02X %lu\r\n",
                	Ab5465T.FrameCnt,
                    Ab5465T.RollingCounter,
                    Ab5465T.Roll,
                    Ab5465T.Pitch,
                    Ab5465T.Yaw,
                    Ab5465T.GyroX,
                    Ab5465T.GyroY,
                    Ab5465T.GyroZ,
                    Ab5465T.AccelX,
                    Ab5465T.AccelY,
                    Ab5465T.AccelZ,
                    Ab5465T.Temperature,
                    Ab5465T.SelfCheck,
                    Ab5465T.IMUFlag[0],
                    Ab5465T.IMUFlag[1],
                    Ab5465T.IMUFlag[2],
                    Ab5465T.IMUFlag[3],
                    Ab5465T.IMUFlag[4],
                    Ab5465T.IMUFlag[5],
                    Ab5465T.TimeStamp);
        if(onParserData != nullptr)
        {
            onParserData(Ab5465T);
        }
    }
    
    void setCbk(Ab5465Cbk cbk_) 
    {
        onParserData = cbk_;
    }

    uint32_t signature() const override { 
        return AB5465_ID; // 协议特征标识
    }

    size_t minFrameLength() const override { return AB5465_LEN; }

    // // 数据到达回调
    Ab5465Cbk onParserData = nullptr;
};
}   //namespace ROSNode
}   //namespace AG