/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-04-11 15:44:13
 * @FilePath: /ros2-imu-dirve/src/AgUtility/AgUtility.cpp
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 21:47:34
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */

#include <time.h>
#include <sys/time.h>
#include <stdio.h>
 #include "AgUtility.h"
 
namespace AG{
namespace ROSNode{
//------------------------ 工具函数 ------------------------
namespace NMEA_Utils {
    std::vector<std::string> split(const std::string& s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        
        while (std::getline(tokenStream, token, delimiter)) {
            // if (!token.empty()) 
            {
                tokens.push_back(token);
            }
        }
        
        // 处理校验和
        if (!tokens.empty()) {
            size_t starPos = tokens.back().find('*');
            if (starPos != std::string::npos) {
                std::string checksum = tokens.back().substr(starPos + 1);
                tokens.back() = tokens.back().substr(0, starPos);
                tokens.push_back(checksum);
            }
        }
        return tokens;
    }

    double convert_to_degrees(const std::string& coord, char direction) {
        size_t dotPos = coord.find('.');
        if (dotPos == std::string::npos) return 0.0;
        
        int degrees = safe_stoi(coord.substr(0, dotPos - 2));
        double minutes = safe_stod(coord.substr(dotPos - 2));
        double result = degrees + minutes / 60.0;
        return result;
    }

    bool isInteger(const std::string& str) {
        if (str.empty()) {
            return false;
        }
        for (char const &c : str) {
            if (c != '-' && c != '+' && c != '.' && std::isdigit(c) == 0) {
                return false;
            }
        }
        return true;
    }

    int safe_stoi(const std::string& str){
        return isInteger(str) ? std::stoi(str) : 0;
    };

    double safe_stod(const std::string& str){
        return isInteger(str) ? std::stod(str) : 0;
    };

static const uint16_t Cal_Crc16Tab[256] =
{
    0x0000u, 0x1021u, 0x2042u, 0x3063u, 0x4084u, 0x50a5u, 0x60c6u, 0x70e7u,
    0x8108u, 0x9129u, 0xa14au, 0xb16bu, 0xc18cu, 0xd1adu, 0xe1ceu, 0xf1efu,
    0x1231u, 0x0210u, 0x3273u, 0x2252u, 0x52b5u, 0x4294u, 0x72f7u, 0x62d6u,
    0x9339u, 0x8318u, 0xb37bu, 0xa35au, 0xd3bdu, 0xc39cu, 0xf3ffu, 0xe3deu,
    0x2462u, 0x3443u, 0x0420u, 0x1401u, 0x64e6u, 0x74c7u, 0x44a4u, 0x5485u,
    0xa56au, 0xb54bu, 0x8528u, 0x9509u, 0xe5eeu, 0xf5cfu, 0xc5acu, 0xd58du,
    0x3653u, 0x2672u, 0x1611u, 0x0630u, 0x76d7u, 0x66f6u, 0x5695u, 0x46b4u,
    0xb75bu, 0xa77au, 0x9719u, 0x8738u, 0xf7dfu, 0xe7feu, 0xd79du, 0xc7bcu,
    0x48c4u, 0x58e5u, 0x6886u, 0x78a7u, 0x0840u, 0x1861u, 0x2802u, 0x3823u,
    0xc9ccu, 0xd9edu, 0xe98eu, 0xf9afu, 0x8948u, 0x9969u, 0xa90au, 0xb92bu,
    0x5af5u, 0x4ad4u, 0x7ab7u, 0x6a96u, 0x1a71u, 0x0a50u, 0x3a33u, 0x2a12u,
    0xdbfdu, 0xcbdcu, 0xfbbfu, 0xeb9eu, 0x9b79u, 0x8b58u, 0xbb3bu, 0xab1au,
    0x6ca6u, 0x7c87u, 0x4ce4u, 0x5cc5u, 0x2c22u, 0x3c03u, 0x0c60u, 0x1c41u,
    0xedaeu, 0xfd8fu, 0xcdecu, 0xddcdu, 0xad2au, 0xbd0bu, 0x8d68u, 0x9d49u,
    0x7e97u, 0x6eb6u, 0x5ed5u, 0x4ef4u, 0x3e13u, 0x2e32u, 0x1e51u, 0x0e70u,
    0xff9fu, 0xefbeu, 0xdfddu, 0xcffcu, 0xbf1bu, 0xaf3au, 0x9f59u, 0x8f78u,
    0x9188u, 0x81a9u, 0xb1cau, 0xa1ebu, 0xd10cu, 0xc12du, 0xf14eu, 0xe16fu,
    0x1080u, 0x00a1u, 0x30c2u, 0x20e3u, 0x5004u, 0x4025u, 0x7046u, 0x6067u,
    0x83b9u, 0x9398u, 0xa3fbu, 0xb3dau, 0xc33du, 0xd31cu, 0xe37fu, 0xf35eu,
    0x02b1u, 0x1290u, 0x22f3u, 0x32d2u, 0x4235u, 0x5214u, 0x6277u, 0x7256u,
    0xb5eau, 0xa5cbu, 0x95a8u, 0x8589u, 0xf56eu, 0xe54fu, 0xd52cu, 0xc50du,
    0x34e2u, 0x24c3u, 0x14a0u, 0x0481u, 0x7466u, 0x6447u, 0x5424u, 0x4405u,
    0xa7dbu, 0xb7fau, 0x8799u, 0x97b8u, 0xe75fu, 0xf77eu, 0xc71du, 0xd73cu,
    0x26d3u, 0x36f2u, 0x0691u, 0x16b0u, 0x6657u, 0x7676u, 0x4615u, 0x5634u,
    0xd94cu, 0xc96du, 0xf90eu, 0xe92fu, 0x99c8u, 0x89e9u, 0xb98au, 0xa9abu,
    0x5844u, 0x4865u, 0x7806u, 0x6827u, 0x18c0u, 0x08e1u, 0x3882u, 0x28a3u,
    0xcb7du, 0xdb5cu, 0xeb3fu, 0xfb1eu, 0x8bf9u, 0x9bd8u, 0xabbbu, 0xbb9au,
    0x4a75u, 0x5a54u, 0x6a37u, 0x7a16u, 0x0af1u, 0x1ad0u, 0x2ab3u, 0x3a92u,
    0xfd2eu, 0xed0fu, 0xdd6cu, 0xcd4du, 0xbdaau, 0xad8bu, 0x9de8u, 0x8dc9u,
    0x7c26u, 0x6c07u, 0x5c64u, 0x4c45u, 0x3ca2u, 0x2c83u, 0x1ce0u, 0x0cc1u,
    0xef1fu, 0xff3eu, 0xcf5du, 0xdf7cu, 0xaf9bu, 0xbfbau, 0x8fd9u, 0x9ff8u,
    0x6e17u, 0x7e36u, 0x4e55u, 0x5e74u, 0x2e93u, 0x3eb2u, 0x0ed1u, 0x1ef0u
};

uint16_t crc16_ccitt_false(unsigned char *data, int len)
{
    uint16_t crc_ret = 0xFFFFu;
    int i;
    //printf("data[%d]:",len);
    for (i = 0; i < len; i++)
    {
        //printf("%02X ",data[i]);
    	crc_ret = Cal_Crc16Tab[(((crc_ret >> 8u) ^ ((uint16_t)data[i]))) & 0xFFu] ^ (crc_ret << 8u);
    }
    //printf("%02X %02X %04X\n",data[i],data[i+1],crc_ret);
    return crc_ret;
} 



int16_t GET_S16(uint8_t* buff, uint32_t offset)
{
	int16_t ret;
	uint16_t temp = GET_U16(buff, offset);

	memcpy(&ret, &temp, 2);

	return ret;
}

int32_t GET_S32(uint8_t* buff, uint32_t offset)
{
	int32_t ret;
	uint32_t temp = GET_U32(buff, offset);

	memcpy(&ret, &temp, 4);

	return ret;
}

int64_t GET_S64(uint8_t* buff, uint32_t offset)
{
	int64_t ret;
	uint64_t temp = GET_U64(buff, offset);

	memcpy(&ret, &temp, 8);

	return ret;
}

float GET_FLT(uint8_t* buff, uint32_t offset)
{
	float ret;
	uint32_t temp = GET_U32(buff, offset);

	memcpy(&ret, &temp, 4);

	return ret;
}

double GET_DBL(uint8_t* buff, uint32_t offset)
{
	double ret;
	uint64_t temp = GET_U64(buff, offset);

	memcpy(&ret, &temp, 8);

	return ret;
}

uint64_t get_sysrun_ts_us()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)(t.tv_sec*1000u*1000u + t.tv_nsec/1000u);
}
}
}   //namespace ROSNode
}   //namespace AG