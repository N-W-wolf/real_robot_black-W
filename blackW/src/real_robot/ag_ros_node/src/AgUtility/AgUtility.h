/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-04-11 15:44:04
 * @FilePath: /ros2-imu-dirve/src/AgUtility/AgUtility.h
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 20:44:49
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */
#ifndef AG_UTILITY_HPP
#define AG_UTILITY_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include <string.h>
#include <stdio.h>
#include <stdint.h>
namespace AG{
namespace ROSNode{
namespace NMEA_Utils {
    extern std::vector<std::string> split(const std::string& s, char delimiter);
    extern double convert_to_degrees(const std::string& coord, char direction);
    extern int safe_stoi(const std::string& str);
    extern double safe_stod(const std::string& str);
    unsigned short crc16_ccitt_false(unsigned char *data, int len);


 // ------------------------ 卫星系统类型定义 ------------------------
 enum class SatelliteSystem {
    GPS,        // GP
    GLONASS,    // GL
    GALILEO,    // GA
    BEIDOU,     // BD
    QZSS,       // GQ
    IRNSS,      // GI
    UNKNOWN
};

#define GET_U8(buff, offset)      (buff[offset])
#define GET_U16(buff, offset)     (buff[offset] | ((uint16_t)buff[offset + 1] << 8))
#define GET_U32(buff, offset)     (buff[offset] | ((uint32_t)buff[offset + 1] << 8) | ((uint32_t)buff[offset + 2] << 16) | ((uint32_t)buff[offset + 3] << 24))
#define GET_U64(buff, offset)     (buff[offset] | ((uint64_t)buff[offset + 1] << 8) | ((uint64_t)buff[offset + 2] << 16) | ((uint64_t)buff[offset + 3] << 24) | \
								   ((uint64_t)buff[offset + 4] << 32)  | ((uint64_t)buff[offset + 5] << 40)  | ((uint64_t)buff[offset + 6] << 48) | ((uint64_t)buff[offset + 7] << 56))

extern int16_t GET_S16(uint8_t* buff, uint32_t offset);
extern int32_t GET_S32(uint8_t* buff, uint32_t offset);
extern int64_t GET_S64(uint8_t* buff, uint32_t offset);
extern float GET_FLT(uint8_t* buff, uint32_t offset);
extern double GET_DBL(uint8_t* buff, uint32_t offset);

extern uint64_t get_sysrun_ts_us();
}
}   //namespace ROSNode
}   //namespace AG
#endif
