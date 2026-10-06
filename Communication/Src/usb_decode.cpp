//
// Created by 20852 on 2026/1/6.
//

#include "../Inc/usb_decode.h"
#include <cstring>

#include "CRC16.h"
#include <algorithm>

// 辅助函数：从字节数组手动解析VisionToGimbal结构体
// 避免直接reinterpret_cast可能的内存对齐问题
static void parse_vision_to_gimbal_manual(const uint8_t* data, USBProtocol::VisionToGimbal& pkt) {
    // 复制头字节
    pkt.head[0] = data[0];
    pkt.head[1] = data[1];
    
    // 模式
    pkt.mode = data[2];
    
    // 解析浮点数（假设是小端序）
    // 注意：STM32是小端架构
    memcpy(&pkt.yaw, &data[3], sizeof(float));
    memcpy(&pkt.yaw_vel, &data[7], sizeof(float));
    memcpy(&pkt.yaw_acc, &data[11], sizeof(float));
    memcpy(&pkt.pitch, &data[15], sizeof(float));
    memcpy(&pkt.pitch_vel, &data[19], sizeof(float));
    memcpy(&pkt.pitch_acc, &data[23], sizeof(float));
    
    // CRC16（最后2个字节）
    pkt.crc16 = (data[27] << 8) | data[28];  // 注意：协议可能是小端序，需要确认
    // 如果CRC是小端序，则应该是：
    // pkt.crc16 = data[27] | (data[28] << 8);
}

USBDecode usb_decoder;

USBDecode::USBDecode()
    : state_(State::WAIT_HEAD_0),
      index_(0),
      error_count_(0)
{
}

bool USBDecode::feed(uint8_t byte)
{
    switch (state_) {
        case State::WAIT_HEAD_0:
            if (byte == USBProtocol::FRAME_HEAD_0) {
                buffer_[0] = byte;
                state_ = State::WAIT_HEAD_1;
            }
            break;

        case State::WAIT_HEAD_1:
            if (byte == USBProtocol::FRAME_HEAD_1) {
                buffer_[1] = byte;
                index_ = 2;
                state_ = State::COLLECT;
            } else {
                state_ = State::WAIT_HEAD_0;
                error_count_++;
            }
            break;

        case State::COLLECT:
            buffer_[index_++] = byte;

            if (index_ >= USBProtocol::VISION_TO_GIMBAL_LEN) {
                state_ = State::WAIT_HEAD_0;
                index_ = 0;

                // // 手动解析数据包，避免reinterpret_cast可能的内存对齐问题
                // USBProtocol::VisionToGimbal pkt;
                // parse_vision_to_gimbal_manual(buffer_, pkt);
                const auto& pkt =
                                 *reinterpret_cast<const USBProtocol::VisionToGimbal*>(buffer_);
                // 计算CRC（不包括最后2个字节的CRC字段）
                uint16_t calc = crc16_calc(
                    buffer_,
                    USBProtocol::VISION_TO_GIMBAL_LEN - 2,
                    0xFFFF
                );

                if (calc == pkt.crc16)
                {
                    // 额外验证：检查模式字段是否有效
                    if (pkt.mode <= 2) { // 有效模式为0,1,2
                        // 将解析的数据复制到内部缓冲区
                        memcpy(buffer_, &pkt, sizeof(pkt));
                        error_count_ = 0;
                        return true;
                    } else {
                        // 模式无效，视为错误
                        error_count_++;
                        return false;
                    }
                }
                else //冗余，但我在这里建议表意清晰，所以这样写
                {
                    error_count_++;
                }
            }
            break;
    }

    return false;
}

const USBProtocol::VisionToGimbal& USBDecode::packet() const
{
    return *reinterpret_cast<const USBProtocol::VisionToGimbal*>(buffer_);
}

uint32_t USBDecode::errorCount() const
{
    return error_count_;
}

void USBDecode::resetErrorCount()
{
    error_count_ = 0;
}
