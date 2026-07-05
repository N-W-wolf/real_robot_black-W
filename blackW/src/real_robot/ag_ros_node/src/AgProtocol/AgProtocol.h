/*
 * @Author: zengweiqing zengweiqing@asensing.com
 * @Date: 2025-04-03 13:45:50
 * @FilePath: /ros2-imu-dirve/src/AgProtocol/AgProtocol.h
 * @version: V01.00.00
 * @LastEditTime: 2025-11-07 21:45:18
 * @LastEditors: zengweiqing zengweiqing@asensing.com
 * @copyright: asensing.co
 */
#ifndef AG_PROTOCOL_HPP
#define AG_PROTOCOL_HPP

#include <vector>
#include <memory>
#include <functional>
#include <deque>
#include <unordered_map>
#include <mutex>

#include <string.h>
#include <stdio.h>
#include "AgUtility.h"
#include "AgLogger.h"
namespace AG{
namespace ROSNode{

// 协议基础抽象类
class AgProtocolBase {
public:
    virtual ~AgProtocolBase() = default;

    // 协议特征检测
    virtual bool checkHeader(const std::vector<uint8_t>& buffer) const = 0;
    
    // 完整帧检测
    virtual bool checkFrameComplete(const std::vector<uint8_t>& buffer) const = 0;
    
    // 帧校验验证
    virtual bool validate(const std::vector<uint8_t>& frame) const = 0;
    
    // 数据解析（返回任意类型）
    virtual void parse(const std::vector<uint8_t>& frame) const = 0;
    
    // 获取协议特征标识（用于快速匹配）
    virtual uint32_t signature() const = 0;
    
    // 最小帧长度要求
    virtual size_t minFrameLength() const = 0;
};

class AgProtocolEngine {
public:
    void addProtocol(std::shared_ptr<AgProtocolBase> proto) {
        protocols_.push_back(proto);
        // 建立快速索引
        auto first_byte = static_cast<uint8_t>(proto->signature() >> 24);
        headerIndex_[first_byte].push_back(proto);
    }

    void start()
    {
        isStart.store(true);
    }
    void stop()
    {
        isStart.store(false);
    }

    void processData(std::vector<uint8_t>& data) {
        std::unique_lock<std::mutex> lock(mMutex);
        try
        {
            if (!isStart.load())
            {
                buffer.clear();
                return;
            }
    
            buffer.insert(buffer.end(), data.begin(), data.end());
    
            while (!buffer.empty() && isStart.load()) {
                // 快速匹配候选协议
                uint8_t first_byte = buffer.front();
                auto candidates = headerIndex_.find(first_byte);
                if (candidates == headerIndex_.end()) {
                    buffer.erase(buffer.begin());
                    continue;
                }
    
                if (buffer.size() < 4) {
                    return;
                }

                // 遍历候选协议
                bool matchedHead = false;
                bool frameHandled = false;
                bool needMoreData = false;
                for (auto& proto : candidates->second) {
                    if (proto->checkHeader(buffer)) {
                        matchedHead = true;
                        size_t required_len = proto->minFrameLength();
                        if (buffer.size() < required_len) {
                            // Not enough data yet, wait for more
                            needMoreData = true;
                            break;
                        }
                            
                        const std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + required_len);
                        if(proto->validate(frame)) {
                            proto->parse(frame);
                            if(required_len >= buffer.size())
                            {
                                buffer.clear();
                            }
                            else
                            {
                                buffer.erase(buffer.begin(), buffer.begin() + required_len);
                            }
                            frameHandled = true;
                            break;
                        } else {
                            // CRC失败处理
                            buffer.erase(buffer.begin());
                            frameHandled = true;
                            break;
                        }
                    }
                }
                if (needMoreData) {
                    return;
                }
                if (frameHandled) {
                    continue;
                }
                if ((!matchedHead) && (buffer.size() > 0)) {
                    buffer.erase(buffer.begin());
                }
                else
                {
                    return;
                }
            }
    
            if (!isStart.load())
            {
                buffer.clear();
            }
        }
        catch(...)
        {
            AGLOGW("exp: AgProtocol unknown exception");
            buffer.clear();
        }
    }
private:
    std::vector<std::shared_ptr<AgProtocolBase>> protocols_;
    std::unordered_map<uint8_t, std::vector<std::shared_ptr<AgProtocolBase>>> headerIndex_;
    std::vector<uint8_t> buffer;
    std::atomic<bool> isStart = false;
    mutable std::mutex mMutex;
};

}   //namespace ROSNode
}   //namespace AG

#endif
