#include "McuDataPreprocessor.h"

namespace s2c {
namespace com {

mcu::SendPacket McuDataPreprocessor::processSend(const mcu::SendPacket& packet) {
    mcu::SendPacket temp_packet = packet;

    temp_packet.yaw_small_target_position += small_yaw_offset;

    temp_packet.yaw_big_torque /= max_big_yaw_torque;
    temp_packet.roll_torque /= max_roll_torque;
    temp_packet.yaw_small_torque /= max_small_yaw_torque;
    temp_packet.pitch_torque /= max_pitch_torque;

    return temp_packet;
}

mcu::ReceivePacket McuDataPreprocessor::processReceive(const mcu::ReceivePacket& packet) {
    mcu::ReceivePacket temp_packet = packet;

    temp_packet.yaw_small_position -= small_yaw_offset;

    temp_packet.yaw_big_torque *= max_big_yaw_torque / big_yaw_torque_receive_multiple;
    temp_packet.roll_torque *= max_roll_torque / roll_torque_receive_multiple;
    temp_packet.yaw_small_torque *= max_small_yaw_torque / small_yaw_torque_receive_multiple;
    temp_packet.pitch_torque *= max_pitch_torque / pitch_torque_receive_multiple;

    return temp_packet;
}

}  // namespace com
}  // namespace s2c
