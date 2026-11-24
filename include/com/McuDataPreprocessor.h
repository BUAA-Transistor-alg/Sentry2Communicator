#pragma once

#include "serial_comm/SerialCommProtocol.hpp"

namespace s2c {
namespace com {

// ============================================================================
// McuDataPreprocessor — MCU 收发数据预处理
// ============================================================================
class McuDataPreprocessor {
public:
    // ── 发送包预处理 ──
    mcu::SendPacket processSend(const mcu::SendPacket& packet);

    // ── 接收包预处理 ──
    mcu::ReceivePacket processReceive(const mcu::ReceivePacket& packet);
    
    float small_yaw_offset = -0.546097f;

    float max_big_yaw_torque = -6.561f;
    float max_roll_torque = 6.640f;
    float max_small_yaw_torque = 2.000f;
    float max_pitch_torque = -3.992f;

    float big_yaw_torque_receive_multiple = 8.0f;
    float roll_torque_receive_multiple = 8.0f;
    float small_yaw_torque_receive_multiple = 1.0f;
    float pitch_torque_receive_multiple = 4.0f;
};

}  // namespace com
}  // namespace s2c
