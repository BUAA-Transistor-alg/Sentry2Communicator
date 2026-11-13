// test_robot_communication.cpp — MCU/IMU 两路串口组合类端到端测试
//
// 用两个 pty 对分别接在 MCU 与 IMU 串口上，验证:
//   · 收到的 MCU 原始包进入快照，处理前原始包同时保留（预处理器当前原样返回）
//   · 四轴温度、底盘 IMU、状态位与裁判系统数据段原样透传
//   · IMU 包原样进入快照
//   · 发送 MCU 包时补 CRC8，字段原样发出
//   · IMU 心跳发送帧的帧头与 CRC32
#include "Communications.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

namespace {

using namespace s2c;
using namespace s2c_test;

com::mcu::ReceivePacket sampleRawMcuPacket() {
    com::mcu::ReceivePacket packet{};
    packet.data_size = 237;
    packet.bullet_velocity = 12.0f;
    packet.pitch_position = 32780.0f;   // 关节角 0 对应的原始读数
    packet.pitch_velocity = 5.0f;
    packet.pitch_torque = 0.5f;
    packet.pitch_temperature = 47;
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
    packet.yaw_small_temperature = 48;
    packet.chassis_imu_yaw = 0.5f;
    packet.chassis_imu_omega = 4.0f;
    packet.mark = 11;
    packet.color = 1;
    packet.auto_aim_switch = 1;
    // 裁判系统数据段样例
    packet.game_status.game_progress = 4;
    packet.game_status.stage_remain_time = 200;
    packet.game_robot_hp.ally_7_robot_HP = 350;
    packet.event_data.event_data_2 = 0x12345678;
    packet.robot_status.current_HP = 180;
    packet.robot_status.maximum_HP = 400;
    packet.robot_status.power_management_gimbal_output = 1;
    packet.power_heat_data.buffer_energy = 55;
    packet.robot_position.x = 4.5f;
    packet.robot_position.y = -3.5f;
    packet.robot_position.angle = 45.0f;
    packet.buff.attack_buff = 25;
    packet.hurt_data.armor_id = 3;
    packet.hurt_data.hit_position_x = -120;
    packet.rfid_status.rfid_status_2 = 0x1A;
    packet.ground_robot_position.heavyengineer_y = -6.5f;
    packet.sentry_info.sentry_info = 0x7F;
    return packet;
}

com::imu::ReceivePacket sampleImuPacket() {
    com::imu::ReceivePacket packet{};
    packet.gx = 0.1f;
    packet.gy = 0.2f;
    packet.gz = 0.3f;
    packet.ax = 0.4f;
    packet.ay = 0.5f;
    packet.az = 9.81f;
    packet.euler_yaw = 1.125;
    packet.euler_pitch = -0.25;
    packet.euler_roll = 0.0625;
    packet.dt_one_tenth_ms = 1234;
    return packet;
}

void testReceiveAndSend() {
    std::cout << "[test] MCU/IMU 接收映射与发送" << std::endl;
    PtyPair mcu_pty = makePtyPair();
    PtyPair imu_pty = makePtyPair();

    com::RobotCommunication comm(
        com::RobotCommunication::PortNames{mcu_pty.slave_path, imu_pty.slave_path});

    // 从未收到数据时快照的两路 valid 均为 false
    const com::RobotCommunication::LatestData empty = comm.getLatestData();
    S2C_CHECK(!empty.mcu_valid);
    S2C_CHECK(!empty.imu_valid);

    // ── 接收: MCU 原始包与 IMU 包 ──
    const auto mcu_frame = makeMcuReceiveFrame(sampleRawMcuPacket());
    const auto imu_frame = makeImuReceiveFrame(sampleImuPacket());
    writeAll(mcu_pty.master_fd, mcu_frame.data(), mcu_frame.size());
    writeAll(imu_pty.master_fd, imu_frame.data(), imu_frame.size());

    S2C_CHECK(waitFor([&] {
        const com::RobotCommunication::LatestData data = comm.getLatestData();
        return data.mcu_valid && data.imu_valid;
    }, std::chrono::milliseconds(1000)));

    const com::RobotCommunication::LatestData data = comm.getLatestData();
    // 原始包访问器返回同一份数据
    S2C_CHECK(comm.lastRawMcuPacket().pitch_position == 32780.0f);
    S2C_CHECK(comm.lastRawMcuPacket().yaw_big_position == 1.0);
    S2C_CHECK(comm.lastImuPacket().gz == 0.3f);
    S2C_CHECK(comm.lastImuPacket().dt_one_tenth_ms == 1234);
    // 处理前原始包
    S2C_CHECK(data.raw_mcu_packet.pitch_position == 32780.0f);
    S2C_CHECK(data.raw_mcu_packet.yaw_big_position == 1.0);
    // 处理后包（预处理器当前原样返回）
    S2C_CHECK(data.mcu_packet.pitch_position == 32780.0f);
    S2C_CHECK(nearlyEqual(data.mcu_packet.pitch_velocity, 5.0, 1e-6));
    S2C_CHECK(nearlyEqual(data.mcu_packet.pitch_torque, 0.5, 1e-6));
    S2C_CHECK(data.mcu_packet.yaw_big_position == 1.0);
    S2C_CHECK(data.mcu_packet.yaw_big_velocity == 2.0f);
    S2C_CHECK(data.mcu_packet.yaw_big_torque == 3.0f);
    S2C_CHECK(nearlyEqual(data.mcu_packet.roll_position, 0.25, 1e-6));
    S2C_CHECK(nearlyEqual(data.mcu_packet.roll_velocity, 0.75, 1e-6));
    S2C_CHECK(nearlyEqual(data.mcu_packet.roll_torque, -1.25, 1e-6));
    S2C_CHECK(data.mcu_packet.yaw_small_position == 2.0f);
    S2C_CHECK(nearlyEqual(data.mcu_packet.yaw_small_velocity, 3.0, 1e-6));
    S2C_CHECK(nearlyEqual(data.mcu_packet.yaw_small_torque, -4.0, 1e-6));
    // 温度、底盘 IMU 与状态位
    S2C_CHECK(data.mcu_packet.pitch_temperature == 47);
    S2C_CHECK(data.mcu_packet.yaw_big_temperature == 45);
    S2C_CHECK(data.mcu_packet.roll_temperature == 46);
    S2C_CHECK(data.mcu_packet.yaw_small_temperature == 48);
    S2C_CHECK(data.mcu_packet.chassis_imu_omega == 4.0f);
    S2C_CHECK(data.mcu_packet.chassis_imu_yaw == 0.5f);
    S2C_CHECK(data.mcu_packet.bullet_velocity == 12.0f);
    S2C_CHECK(data.mcu_packet.mark == 11);
    // 裁判系统数据段原样透传
    S2C_CHECK(data.mcu_packet.game_status.game_progress == 4);
    S2C_CHECK(data.mcu_packet.game_status.stage_remain_time == 200);
    S2C_CHECK(data.mcu_packet.game_robot_hp.ally_7_robot_HP == 350);
    S2C_CHECK(data.mcu_packet.event_data.event_data_2 == 0x12345678);
    S2C_CHECK(data.mcu_packet.robot_status.current_HP == 180);
    S2C_CHECK(data.mcu_packet.robot_status.power_management_gimbal_output == 1);
    S2C_CHECK(data.mcu_packet.power_heat_data.buffer_energy == 55);
    S2C_CHECK(data.mcu_packet.robot_position.x == 4.5f);
    S2C_CHECK(data.mcu_packet.robot_position.angle == 45.0f);
    S2C_CHECK(data.mcu_packet.buff.attack_buff == 25);
    S2C_CHECK(data.mcu_packet.hurt_data.armor_id == 3);
    S2C_CHECK(data.mcu_packet.hurt_data.hit_position_x == -120);
    S2C_CHECK(data.mcu_packet.rfid_status.rfid_status_2 == 0x1A);
    S2C_CHECK(data.mcu_packet.ground_robot_position.heavyengineer_y == -6.5f);
    S2C_CHECK(data.mcu_packet.sentry_info.sentry_info == 0x7F);
    // IMU 包不经预处理
    S2C_CHECK(nearlyEqual(data.imu_packet.euler_yaw, 1.125, 1e-12));
    S2C_CHECK(nearlyEqual(data.imu_packet.euler_pitch, -0.25, 1e-12));
    S2C_CHECK(nearlyEqual(data.imu_packet.gz, 0.3, 1e-6));
    S2C_CHECK(data.imu_packet.dt_one_tenth_ms == 1234);
    S2C_CHECK(nearlyEqual(data.raw_imu_packet.euler_yaw, 1.125, 1e-12));

    // ── 发送 MCU 包: 映射后补 CRC8 ──
    com::mcu::SendPacket outgoing{};
    outgoing.friction_wheel_enable = 1;
    outgoing.pitch_target_position = 1.0f;
    outgoing.yaw_big_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    outgoing.yaw_big_target_position = -0.5;
    outgoing.yaw_small_target_position = 0.25f;
    outgoing.roll_target_position = -0.125f;
    outgoing.deform_control = 0x0A;
    outgoing.displacement_target_velocity_x = 0.5f;
    outgoing.displacement_target_velocity_y = -0.25f;
    S2C_CHECK(comm.sendToMcu(outgoing));

    const std::vector<uint8_t> sent =
        readWithTimeout(mcu_pty.master_fd, sizeof(com::mcu::SendPacket),
                        std::chrono::milliseconds(1000));
    S2C_CHECK(sent.size() == sizeof(com::mcu::SendPacket));
    S2C_CHECK(sent[0] == 0x42);
    S2C_CHECK(sent[1] == 0x52);
    S2C_CHECK(sent[3] == 67);
    S2C_CHECK(sent.back() == com::CRC8_Check_Sum(sent.data(), sent.size() - 1));

    com::mcu::SendPacket sent_packet{};
    std::memcpy(&sent_packet, sent.data(), sizeof(sent_packet));
    S2C_CHECK(sent_packet.pitch_target_position == 1.0f);
    S2C_CHECK(sent_packet.yaw_big_target_position == -0.5);
    S2C_CHECK(sent_packet.yaw_small_target_position == 0.25f);
    S2C_CHECK(nearlyEqual(sent_packet.roll_target_position, -0.125, 1e-6));
    S2C_CHECK(sent_packet.deform_control == 0x0A);
    S2C_CHECK(sent_packet.displacement_target_velocity_x == 0.5f);
    S2C_CHECK(sent_packet.displacement_target_velocity_y == -0.25f);

    // ── 发送 IMU 心跳: 前导 + data_size=0 + CRC32 ──
    com::imu::SendPacket heartbeat;
    S2C_CHECK(comm.sendToImu(heartbeat));
    const std::vector<uint8_t> imu_sent =
        readWithTimeout(imu_pty.master_fd, sizeof(com::imu::SendPacket),
                        std::chrono::milliseconds(1000));
    S2C_CHECK(imu_sent.size() == sizeof(com::imu::SendPacket));
    S2C_CHECK(imu_sent[0] == 0xA7);
    S2C_CHECK(imu_sent[1] == 0xB6);
    S2C_CHECK(imu_sent[2] == 0xC5);
    S2C_CHECK(imu_sent[3] == 0);
    const uint32_t imu_crc =
        com::CRC32_Calculate(imu_sent.data(), imu_sent.size() - 4);
    S2C_CHECK(std::memcmp(imu_sent.data() + imu_sent.size() - 4, &imu_crc, 4) == 0);

    comm.stop();
    closePtyPair(mcu_pty);
    closePtyPair(imu_pty);
}

// 多次接收时快照取到最新一帧
void testLatestSampleWins() {
    std::cout << "[test] 快照取最新一帧" << std::endl;
    PtyPair mcu_pty = makePtyPair();
    PtyPair imu_pty = makePtyPair();
    com::RobotCommunication comm(
        com::RobotCommunication::PortNames{mcu_pty.slave_path, imu_pty.slave_path});

    for (uint8_t step = 1; step <= 3; ++step) {
        com::mcu::ReceivePacket packet = sampleRawMcuPacket();
        packet.yaw_big_position = static_cast<double>(step);
        const auto frame = makeMcuReceiveFrame(packet);
        writeAll(mcu_pty.master_fd, frame.data(), frame.size());
        S2C_CHECK(waitFor([&] {
            return comm.getLatestData().mcu_packet.yaw_big_position
                == static_cast<double>(step);
        }, std::chrono::milliseconds(1000)));
    }

    const com::RobotCommunication::LatestData data = comm.getLatestData();
    S2C_CHECK(data.mcu_packet.yaw_big_position == 3.0);

    comm.stop();
    closePtyPair(mcu_pty);
    closePtyPair(imu_pty);
}

}  // namespace

int main() {
    testReceiveAndSend();
    testLatestSampleWins();
    std::cout << "所有组合类测试通过" << std::endl;
    return 0;
}
