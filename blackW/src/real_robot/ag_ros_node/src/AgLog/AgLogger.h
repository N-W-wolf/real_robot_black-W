/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-04-08 11:11:40
 * @FilePath: /ros2-imu-dirve/src/AgLog/AgLogger.h
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 21:45:35
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */
#ifndef _AGLOGGER_H_
#define _AGLOGGER_H_
#include "TinyLog.h"
namespace AG{
namespace ROSNode{
#define AGLOGD  TinyLog::debug   
#define AGLOGI  TinyLog::info   
#define AGLOGW  TinyLog::warning   
#define AGLOGE  TinyLog::error   
#define AGLOGF  TinyLog::fatal   
}   //namespace ROSNode
}   //namespace AG
#endif
