// test_preprocessor.cpp — MCU 预处理测试
//
// 当前阶段预处理器不做任何处理: 验证收发包逐字节原样返回（四轴、温度、
// 状态位与裁判系统数据段都不被改动）。
#include "McuDataPreprocessor.h"
#include "test_support.hpp"

#include <cstring>
#include <iostream>

namespace {

using namespace s2c;
using namespace s2c_test;

void testSendPassthrough() {
    std::cout << "[test] 发送包原样返回" << std::endl;

    com::mcu::SendPacket packet{};
    packet.friction_wheel_enable = 1;
    packet.fire = 1;
    packet.yaw_big_mode = com::mcu::AXIS_MODE_TORQUE_PLUS_PID;
    packet.yaw_big_target_position = 1.25;
    packet.yaw_big_target_velocity = -0.5f;
    packet.yaw_big_torque = 2.0f;
    packet.roll_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    packet.roll_target_position = -0.375f;
    packet.roll_target_velocity = 0.625f;
    packet.roll_torque = -1.5f;
    packet.yaw_small_mode = com::mcu::AXIS_MODE_TORQUE_PLUS_PID;
    packet.yaw_small_target_position = -0.25f;
    packet.yaw_small_target_velocity = 0.75f;
    packet.yaw_small_torque = 0.5f;
    packet.pitch_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    packet.pitch_target_position = 0.125f;
    packet.pitch_target_velocity = -1.25f;
    packet.pitch_torque = 3.0f;
    packet.deform_control = 0x05;
    packet.displacement_target_velocity_x = 0.75f;
    packet.displacement_target_velocity_y = -1.5f;

    const com::mcu::SendPacket processed = com::McuDataPreprocessor::processSend(packet);
    S2C_CHECK(std::memcmp(&processed, &packet, sizeof(packet)) == 0);
    S2C_CHECK(processed.yaw_big_target_position == 1.25);
    S2C_CHECK(processed.pitch_target_position == 0.125f);
    S2C_CHECK(processed.deform_control == 0x05);
    S2C_CHECK(processed.displacement_target_velocity_y == -1.5f);
    S2C_CHECK(processed.yaw_small_mode == com::mcu::AXIS_MODE_TORQUE_PLUS_PID);
}

void testReceivePassthrough() {
    std::cout << "[test] 接收包原样返回" << std::endl;

    com::mcu::ReceivePacket packet{};
    packet.data_size = 237;
    packet.bullet_velocity = 12.0f;
    packet.yaw_big_position = 1.0;
    packet.yaw_big_velocity = 2.0f;
    packet.yaw_big_torque = 3.0f;
    packet.yaw_big_temperature = 45;
    packet.roll_position = 0.25f;
    packet.roll_velocity = 0.75f;
    packet.roll_torque = -1.25f;
    packet.roll_temperature = 46;
    packet.yaw_small_position = 2.0f;
    packet.yaw_small_velocity = 3.0f;
    packet.yaw_small_torque = -4.0f;
    packet.yaw_small_temperature = 47;
    packet.pitch_position = 32780.0f;
    packet.pitch_velocity = 5.0f;
    packet.pitch_torque = 0.5f;
    packet.pitch_temperature = 48;
    packet.chassis_imu_yaw = 0.5f;
    packet.chassis_imu_omega = 4.0f;
    packet.mark = 7;
    packet.color = 3;
    packet.auto_aim_switch = 1;
    packet.game_status.game_progress = 4;
    packet.game_status.stage_remain_time = 200;
    packet.game_robot_hp.ally_7_robot_HP = 350;
    packet.event_data.event_data_2 = 0x12345678;
    packet.referee_warning.level = 2;
    packet.dart_info.dart_info = 0x0123;
    packet.robot_status.current_HP = 180;
    packet.power_heat_data.buffer_energy = 55;
    packet.robot_position.x = 4.5f;
    packet.buff.attack_buff = 25;
    packet.hurt_data.hit_position_x = -120;
    packet.rfid_status.rfid_status_2 = 0x1A;
    packet.ground_robot_position.heavyengineer_y = -6.5f;
    packet.sentry_info.sentry_info = 0x7F;

    const com::mcu::ReceivePacket processed = com::McuDataPreprocessor::processReceive(packet);
    S2C_CHECK(std::memcmp(&processed, &packet, sizeof(packet)) == 0);
    // 底盘 yaw 角速度此前会被取负，现在也必须原样返回
    S2C_CHECK(processed.chassis_imu_omega == 4.0f);
    S2C_CHECK(processed.pitch_position == 32780.0f);
    S2C_CHECK(processed.pitch_temperature == 48);
    S2C_CHECK(processed.robot_status.current_HP == 180);
    S2C_CHECK(processed.sentry_info.sentry_info == 0x7F);
}

}  // namespace

int main() {
    testSendPassthrough();
    testReceivePassthrough();
    std::cout << "所有预处理测试通过" << std::endl;
    return 0;
}
