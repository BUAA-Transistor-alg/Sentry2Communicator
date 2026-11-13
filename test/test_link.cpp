// test_link.cpp — 串口协议层测试
//
// 用 pty 对建立真实链路，验证: 帧同步与 CRC 校验、校验失败后丢弃并重新同步、
// 一次写入多帧、发送帧的帧头/data_size/CRC8，以及 IMU 的 CRC32 帧。
#include "Communications.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

namespace {

using namespace s2c;
using namespace s2c_test;

com::mcu::ReceivePacket sampleMcuPacket(float pitch_position) {
    com::mcu::ReceivePacket packet{};
    packet.data_size = 237;
    packet.bullet_velocity = 15.5f;
    packet.yaw_big_position = 2.5;
    packet.yaw_big_velocity = -1.25f;
    packet.yaw_big_torque = 0.75f;
    packet.yaw_big_temperature = 43;
    packet.roll_position = 0.125f;
    packet.roll_velocity = 0.5f;
    packet.roll_torque = -0.25f;
    packet.roll_temperature = 44;
    packet.yaw_small_position = -0.375f;
    packet.yaw_small_velocity = 1.5f;
    packet.yaw_small_torque = -2.0f;
    packet.yaw_small_temperature = 45;
    packet.pitch_position = pitch_position;
    packet.pitch_velocity = -0.5f;
    packet.pitch_torque = 1.0f;
    packet.pitch_temperature = 46;
    packet.chassis_imu_yaw = 1.5f;
    packet.chassis_imu_omega = -2.0f;
    packet.mark = 5;
    packet.color = 1;
    packet.auto_aim_switch = 1;
    // 裁判系统数据段样例
    packet.game_status.game_type = 1;
    packet.game_status.game_progress = 4;
    packet.game_status.stage_remain_time = 123;
    packet.game_status.SyncTimeStamp = 1700000000ULL;
    packet.game_robot_hp.ally_7_robot_HP = 400;
    packet.event_data.event_data = 0xDEADBEEF;
    packet.referee_warning.level = 2;
    packet.dart_info.dart_info = 0x0123;
    packet.robot_status.robot_id = 7;
    packet.robot_status.current_HP = 200;
    packet.robot_status.bullet_speed_limit = 30.0f;
    packet.power_heat_data.buffer_energy = 60;
    packet.robot_position.x = 1.5f;
    packet.robot_position.y = -2.5f;
    packet.robot_position.angle = 90.0f;
    packet.buff.cooling_buff = 20;
    packet.hurt_data.hit_sequence = 12;
    packet.rfid_status.rfid_status = 0x5;
    packet.ground_robot_position.standard_3_x = 3.5f;
    packet.sentry_info.sentry_info_3 = 0x0102030405060708ULL;
    return packet;
}

// 合法帧被完整送达回调
void testMcuReceiveValidFrame() {
    std::cout << "[test] MCU 合法帧送达回调" << std::endl;
    PtyPair pty = makePtyPair();
    PacketSink<com::mcu::ReceivePacket> sink;
    com::McuCommunication serial(
        [&](const com::mcu::ReceivePacket& packet) { sink.push(packet); },
        true, pty.slave_path);

    S2C_CHECK(serial.isOpen());
    S2C_CHECK(serial.portName() == pty.slave_path);

    const auto frame = makeMcuReceiveFrame(sampleMcuPacket(0.75f));
    writeAll(pty.master_fd, frame.data(), frame.size());
    S2C_CHECK(waitFor([&] { return sink.size() >= 1; }, std::chrono::milliseconds(1000)));

    const com::mcu::ReceivePacket received = sink.at(0);
    S2C_CHECK(received.data_size == 237);
    S2C_CHECK(received.bullet_velocity == 15.5f);
    // 四个关节
    S2C_CHECK(received.yaw_big_position == 2.5);
    S2C_CHECK(received.yaw_big_velocity == -1.25f);
    S2C_CHECK(received.yaw_big_torque == 0.75f);
    S2C_CHECK(received.yaw_big_temperature == 43);
    S2C_CHECK(received.roll_position == 0.125f);
    S2C_CHECK(received.roll_velocity == 0.5f);
    S2C_CHECK(received.roll_torque == -0.25f);
    S2C_CHECK(received.roll_temperature == 44);
    S2C_CHECK(received.yaw_small_position == -0.375f);
    S2C_CHECK(received.yaw_small_velocity == 1.5f);
    S2C_CHECK(received.yaw_small_torque == -2.0f);
    S2C_CHECK(received.yaw_small_temperature == 45);
    S2C_CHECK(received.pitch_position == 0.75f);
    S2C_CHECK(received.pitch_velocity == -0.5f);
    S2C_CHECK(received.pitch_torque == 1.0f);
    S2C_CHECK(received.pitch_temperature == 46);
    // 底盘 IMU 与状态位
    S2C_CHECK(received.chassis_imu_yaw == 1.5f);
    S2C_CHECK(received.chassis_imu_omega == -2.0f);
    S2C_CHECK(received.mark == 5);
    S2C_CHECK(received.color == 1);
    S2C_CHECK(received.auto_aim_switch == 1);
    // 裁判系统数据段
    S2C_CHECK(received.game_status.game_type == 1);
    S2C_CHECK(received.game_status.game_progress == 4);
    S2C_CHECK(received.game_status.stage_remain_time == 123);
    S2C_CHECK(received.game_status.SyncTimeStamp == 1700000000ULL);
    S2C_CHECK(received.game_robot_hp.ally_7_robot_HP == 400);
    S2C_CHECK(received.event_data.event_data == 0xDEADBEEF);
    S2C_CHECK(received.referee_warning.level == 2);
    S2C_CHECK(received.dart_info.dart_info == 0x0123);
    S2C_CHECK(received.robot_status.robot_id == 7);
    S2C_CHECK(received.robot_status.current_HP == 200);
    S2C_CHECK(received.robot_status.bullet_speed_limit == 30.0f);
    S2C_CHECK(received.power_heat_data.buffer_energy == 60);
    S2C_CHECK(received.robot_position.x == 1.5f);
    S2C_CHECK(received.robot_position.y == -2.5f);
    S2C_CHECK(received.robot_position.angle == 90.0f);
    S2C_CHECK(received.buff.cooling_buff == 20);
    S2C_CHECK(received.hurt_data.hit_sequence == 12);
    S2C_CHECK(received.rfid_status.rfid_status == 0x5);
    S2C_CHECK(received.ground_robot_position.standard_3_x == 3.5f);
    S2C_CHECK(received.sentry_info.sentry_info_3 == 0x0102030405060708ULL);
    S2C_CHECK(received.crc8 == frame.back());

    serial.stopWorker();
    closePtyPair(pty);
}

// CRC 错误的帧被丢弃，随后的垃圾字节之后仍能重新同步到合法帧
void testMcuCrcFailureAndResync() {
    std::cout << "[test] CRC 校验失败丢弃与垃圾字节后重新同步" << std::endl;
    PtyPair pty = makePtyPair();
    PacketSink<com::mcu::ReceivePacket> sink;
    com::McuCommunication serial(
        [&](const com::mcu::ReceivePacket& packet) { sink.push(packet); },
        true, pty.slave_path);

    const auto first = makeMcuReceiveFrame(sampleMcuPacket(0.25f));
    writeAll(pty.master_fd, first.data(), first.size());
    S2C_CHECK(waitFor([&] { return sink.size() >= 1; }, std::chrono::milliseconds(1000)));

    // 破坏帧尾 CRC
    auto corrupted = makeMcuReceiveFrame(sampleMcuPacket(1.25f));
    corrupted.back() ^= 0xFF;
    writeAll(pty.master_fd, corrupted.data(), corrupted.size());
    S2C_CHECK(!waitFor([&] { return sink.size() >= 2; }, std::chrono::milliseconds(200)));

    // 垃圾字节（末两字节故意是前导的前两个字节）之后跟一帧合法数据
    const std::vector<uint8_t> junk = {0x00, 0x11, 0x22, 0x33, 0x44, 0x42, 0x52};
    const auto second = makeMcuReceiveFrame(sampleMcuPacket(2.25f));
    std::vector<uint8_t> combined = junk;
    combined.insert(combined.end(), second.begin(), second.end());
    writeAll(pty.master_fd, combined.data(), combined.size());
    S2C_CHECK(waitFor([&] { return sink.size() >= 2; }, std::chrono::milliseconds(1000)));
    S2C_CHECK(sink.at(1).pitch_position == 2.25f);

    serial.stopWorker();
    closePtyPair(pty);
}

// 一次写入两帧（黏包）时两帧都被处理
void testMcuMultipleFramesInOneWrite() {
    std::cout << "[test] 一次写入多帧全部处理" << std::endl;
    PtyPair pty = makePtyPair();
    PacketSink<com::mcu::ReceivePacket> sink;
    com::McuCommunication serial(
        [&](const com::mcu::ReceivePacket& packet) { sink.push(packet); },
        true, pty.slave_path);

    const auto frame_a = makeMcuReceiveFrame(sampleMcuPacket(0.1f));
    const auto frame_b = makeMcuReceiveFrame(sampleMcuPacket(0.2f));
    std::vector<uint8_t> combined = frame_a;
    combined.insert(combined.end(), frame_b.begin(), frame_b.end());
    writeAll(pty.master_fd, combined.data(), combined.size());

    S2C_CHECK(waitFor([&] { return sink.size() >= 2; }, std::chrono::milliseconds(1000)));
    S2C_CHECK(sink.at(0).pitch_position == 0.1f);
    S2C_CHECK(sink.at(1).pitch_position == 0.2f);

    serial.stopWorker();
    closePtyPair(pty);
}

// 发送帧的帧头、data_size 与 CRC8 正确
void testMcuSendFrame() {
    std::cout << "[test] MCU 发送帧 CRC8" << std::endl;
    PtyPair pty = makePtyPair();
    com::McuCommunication serial(
        [](const com::mcu::ReceivePacket&) {}, false, pty.slave_path);

    com::mcu::SendPacket packet;
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

    S2C_CHECK(serial.sendData(packet));
    const std::vector<uint8_t> sent =
        readWithTimeout(pty.master_fd, sizeof(packet), std::chrono::milliseconds(1000));
    S2C_CHECK(sent.size() == sizeof(packet));
    S2C_CHECK(sizeof(packet) == 72);
    S2C_CHECK(sent[0] == 0x42);
    S2C_CHECK(sent[1] == 0x52);
    S2C_CHECK(sent[2] == com::mcu::PROTOCOL_VERSION);
    S2C_CHECK(sent[3] == 67);
    const uint8_t crc = com::CRC8_Check_Sum(sent.data(), sent.size() - 1);
    S2C_CHECK(sent.back() == crc);
    S2C_CHECK(std::memcmp(sent.data(),
                          reinterpret_cast<const uint8_t*>(&packet),
                          sizeof(packet)) == 0);

    closePtyPair(pty);
}

// IMU 接收帧（CRC32）送达回调；心跳发送帧的 CRC32 正确
void testImuReceiveAndSend() {
    std::cout << "[test] IMU 接收帧与心跳帧 CRC32" << std::endl;
    PtyPair pty = makePtyPair();
    PacketSink<com::imu::ReceivePacket> sink;
    com::ImuCommunication serial(
        [&](const com::imu::ReceivePacket& packet) { sink.push(packet); },
        true, pty.slave_path);

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

    const auto frame = makeImuReceiveFrame(packet);
    writeAll(pty.master_fd, frame.data(), frame.size());
    S2C_CHECK(waitFor([&] { return sink.size() >= 1; }, std::chrono::milliseconds(1000)));

    const com::imu::ReceivePacket received = sink.at(0);
    S2C_CHECK(received.data_size == sizeof(packet) - 3 - 1 - 4);
    S2C_CHECK(received.gx == 0.1f);
    S2C_CHECK(received.gz == 0.3f);
    S2C_CHECK(received.az == 9.81f);
    S2C_CHECK(received.euler_yaw == 1.125);
    S2C_CHECK(received.euler_pitch == -0.25);
    S2C_CHECK(received.euler_roll == 0.0625);
    S2C_CHECK(received.dt_one_tenth_ms == 1234);

    com::imu::SendPacket heartbeat;
    S2C_CHECK(serial.sendData(heartbeat));
    const std::vector<uint8_t> sent =
        readWithTimeout(pty.master_fd, sizeof(heartbeat), std::chrono::milliseconds(1000));
    S2C_CHECK(sent.size() == sizeof(heartbeat));
    S2C_CHECK(sent[0] == 0xA7);
    S2C_CHECK(sent[1] == 0xB6);
    S2C_CHECK(sent[2] == 0xC5);
    S2C_CHECK(sent[3] == 0);
    const uint32_t crc = com::CRC32_Calculate(sent.data(), sent.size() - 4);
    S2C_CHECK(std::memcmp(sent.data() + sent.size() - 4, &crc, 4) == 0);

    serial.stopWorker();
    closePtyPair(pty);
}

}  // namespace

int main() {
    testMcuReceiveValidFrame();
    testMcuCrcFailureAndResync();
    testMcuMultipleFramesInOneWrite();
    testMcuSendFrame();
    testImuReceiveAndSend();
    std::cout << "所有串口协议层测试通过" << std::endl;
    return 0;
}
