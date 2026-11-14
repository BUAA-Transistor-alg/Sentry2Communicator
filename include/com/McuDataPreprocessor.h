#pragma once

#include "serial_comm/SerialCommProtocol.hpp"

namespace s2c {
namespace com {

// ============================================================================
// McuDataPreprocessor — MCU 收发数据预处理
//
// 当前阶段不做任何处理: 收发包原样返回。后续标定映射（编码器/指令 → 角度、
// 电控单位换算、方向翻转等）在实现文件里加入，接口保持不变。
// ============================================================================
class McuDataPreprocessor {
public:
    // ── 发送包预处理 ──
    static mcu::SendPacket processSend(const mcu::SendPacket& packet);

    // ── 接收包预处理 ──
    static mcu::ReceivePacket processReceive(const mcu::ReceivePacket& packet);
};

}  // namespace com
}  // namespace s2c
