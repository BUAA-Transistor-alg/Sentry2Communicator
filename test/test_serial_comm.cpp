// test_serial_comm.cpp — 串口通信服务端/客户端端到端测试
//
// 用 pty 对模拟电控串口与 IMU 串口，验证:
//   · 视觉写入火控/四轴与视觉向导航通信、导航写入变形与底盘位移与导航向视觉通信
//   · 服务端按 300Hz 向串口发送维护的数据（内容与频率）
//   · 服务端接收电控/IMU 数据并更新最新版本与时间戳
//   · 客户端两种读取方法都拿到 6 个结构体的完整快照
//   · 视觉/导航写入范围互不影响、并发写入互不破坏
#include "SerialCommClient.hpp"
#include "serial_comm/SerialCommServer.hpp"
#include "test_support.hpp"

#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace s2c;
using s2c_test::waitFor;

constexpr double kFrequencyToleranceHz = 50.0;   // 300Hz 允许的测试偏差

// 从 pty 主端持续读取并解析服务端发往电控的帧
class McuSendFrameReader {
public:
    struct Snapshot {
        com::mcu::SendPacket packet{};
        uint64_t count = 0;
        uint64_t crc_errors = 0;
        bool has_packet = false;
    };

    explicit McuSendFrameReader(int fd) : fd_(fd), thread_([this] { run(); }) {}
    ~McuSendFrameReader() { stop(); }

    void stop() {
        if (!stop_.exchange(true)) thread_.join();
    }

    Snapshot snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

private:
    void run() {
        std::vector<uint8_t> buffer;
        while (!stop_.load(std::memory_order_acquire)) {
            struct pollfd pfd;
            pfd.fd = fd_;
            pfd.events = POLLIN;
            pfd.revents = 0;
            if (::poll(&pfd, 1, 20) <= 0) continue;

            uint8_t temp[4096];
            const ssize_t n = ::read(fd_, temp, sizeof(temp));
            if (n <= 0) continue;
            buffer.insert(buffer.end(), temp, temp + n);

            while (true) {
                size_t prefix = 0;
                bool found = false;
                while (prefix + 1 < buffer.size()) {
                    if (buffer[prefix] == 0x42 && buffer[prefix + 1] == 0x52) {
                        found = true;
                        break;
                    }
                    ++prefix;
                }
                if (!found) {
                    if (buffer.size() > 1) {
                        buffer.erase(buffer.begin(), buffer.end() - 1);
                    }
                    break;
                }
                if (prefix > 0) {
                    buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(prefix));
                }
                if (buffer.size() < 4) break;
                if (buffer[2] != com::mcu::PROTOCOL_VERSION) {
                    buffer.erase(buffer.begin());
                    continue;
                }
                const size_t data_size = buffer[3];
                if (data_size != sizeof(com::mcu::SendPacket) - 5) {
                    buffer.erase(buffer.begin());
                    continue;
                }
                const size_t frame_size = 3 + 1 + data_size + 1;
                if (buffer.size() < frame_size) break;

                const uint8_t crc = com::CRC8_Check_Sum(buffer.data(), frame_size - 1);
                if (crc != buffer[frame_size - 1]) {
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        ++snapshot_.crc_errors;
                    }
                    buffer.erase(buffer.begin());
                    continue;
                }
                com::mcu::SendPacket packet{};
                std::memcpy(&packet, buffer.data(), sizeof(packet));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++snapshot_.count;
                    snapshot_.packet = packet;
                    snapshot_.has_packet = true;
                }
                buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(frame_size));
            }
        }
    }

    int fd_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    Snapshot snapshot_;
};

serial_comm::VisWrite makeInitialVisWrite() {
    serial_comm::VisWrite write{};
    write.serial.fire_control.friction_wheel_enable = 1;
    write.serial.fire_control.fire = 1;
    write.serial.axes.yaw_big_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    write.serial.axes.yaw_big_target_position = 1.25;
    write.serial.axes.yaw_big_target_velocity = 0.5f;
    write.serial.axes.yaw_big_torque = -0.25f;
    write.serial.axes.roll_mode = com::mcu::AXIS_MODE_TORQUE_PLUS_PID;
    write.serial.axes.roll_target_position = 0.125f;
    write.serial.axes.roll_target_velocity = 0.25f;
    write.serial.axes.roll_torque = -0.5f;
    write.serial.axes.yaw_small_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    write.serial.axes.yaw_small_target_position = -0.375f;
    write.serial.axes.yaw_small_target_velocity = 0.75f;
    write.serial.axes.yaw_small_torque = 0.5f;
    write.serial.axes.pitch_mode = com::mcu::AXIS_MODE_TORQUE_PLUS_PID;
    write.serial.axes.pitch_target_position = 0.0625f;
    write.serial.axes.pitch_target_velocity = -0.5f;
    write.serial.axes.pitch_torque = 1.5f;
    for (uint32_t i = 0; i < serial_comm::kTargetCount; ++i) {
        serial_comm::TargetInfo& target = write.vision_to_nav.targets[i];
        target.recognized = (i % 2 == 0) ? 1 : 0;
        target.enemy_x = static_cast<float>(i) + 0.5f;
        target.enemy_y = -static_cast<float>(i) - 1.5f;
        target.enemy_z = 0.25f;
    }
    write.vision_to_nav.auto_aim_state = 3;
    write.vision_to_nav.target_id = 4;
    return write;
}

serial_comm::NavWrite makeInitialNavWrite() {
    serial_comm::NavWrite write{};
    write.serial.deform_chassis.deform_control = 0x0A;
    write.serial.deform_chassis.displacement_target_velocity_x = 0.5f;
    write.serial.deform_chassis.displacement_target_velocity_y = -0.25f;
    write.nav_to_vision.auto_aim_mode = serial_comm::AutoAimMode::kEnergyRune;
    write.nav_to_vision.suggested_target_id = 4;
    write.nav_to_vision.current_velocity[0] = 1.0f;
    write.nav_to_vision.current_velocity[1] = 2.0f;
    write.nav_to_vision.current_velocity[2] = 3.0f;
    write.nav_to_vision.current_position[0] = 4.0f;
    write.nav_to_vision.current_position[1] = 5.0f;
    write.nav_to_vision.current_position[2] = 6.0f;
    write.nav_to_vision.orientation_quaternion[0] = 1.0f;
    write.nav_to_vision.orientation_quaternion[1] = 0.0f;
    write.nav_to_vision.orientation_quaternion[2] = 0.0f;
    write.nav_to_vision.orientation_quaternion[3] = 0.0f;
    for (uint32_t i = 0; i < serial_comm::kPredictionPointCount; ++i) {
        write.nav_to_vision.predicted[i].position[0] = static_cast<float>(i);
        write.nav_to_vision.predicted[i].position[1] = static_cast<float>(i) + 0.5f;
        write.nav_to_vision.predicted[i].position[2] = static_cast<float>(i) + 1.0f;
        write.nav_to_vision.predicted[i].velocity[0] = static_cast<float>(i) * 0.1f;
        write.nav_to_vision.predicted[i].velocity[1] = static_cast<float>(i) * 0.2f;
        write.nav_to_vision.predicted[i].velocity[2] = static_cast<float>(i) * 0.3f;
    }
    write.nav_to_vision.suggested_big_yaw = 0.75f;
    write.nav_to_vision.request_big_yaw_change = 1;
    return write;
}

com::mcu::ReceivePacket makeMcuSample() {
    com::mcu::ReceivePacket packet{};
    packet.data_size = 237;
    packet.bullet_velocity = 12.5f;
    packet.yaw_big_position = 1.5;
    packet.yaw_big_velocity = 0.25f;
    packet.yaw_big_torque = 0.75f;
    packet.yaw_big_temperature = 43;
    packet.roll_position = 0.125f;
    packet.roll_velocity = 0.5f;
    packet.roll_torque = -0.25f;
    packet.roll_temperature = 44;
    packet.yaw_small_position = -0.5f;
    packet.yaw_small_velocity = 0.75f;
    packet.yaw_small_torque = 0.5f;
    packet.yaw_small_temperature = 45;
    packet.pitch_position = 0.25f;
    packet.pitch_velocity = -0.5f;
    packet.pitch_torque = 1.5f;
    packet.pitch_temperature = 46;
    packet.chassis_imu_yaw = 1.25f;
    packet.chassis_imu_omega = -0.5f;
    packet.mark = 9;
    packet.color = 1;
    packet.auto_aim_switch = 1;
    packet.robot_status.current_HP = 250;
    return packet;
}

com::imu::ReceivePacket makeImuSample() {
    com::imu::ReceivePacket packet{};
    packet.data_size = sizeof(com::imu::ReceivePacket) - 8;
    packet.gx = 0.1f;
    packet.gy = 0.2f;
    packet.gz = 0.3f;
    packet.ax = 0.4f;
    packet.ay = 0.5f;
    packet.az = 9.81f;
    packet.euler_yaw = 0.5;
    packet.euler_pitch = -0.25;
    packet.euler_roll = 0.125;
    packet.dt_one_tenth_ms = 100;
    return packet;
}

com::mcu::SendPacket buildExpectedSendPacket(const serial_comm::VisWrite& vis_write,
                                             const serial_comm::NavWrite& nav_write) {
    com::mcu::SendPacket packet{};
    packet.friction_wheel_enable = vis_write.serial.fire_control.friction_wheel_enable;
    packet.fire = vis_write.serial.fire_control.fire;
    const serial_comm::AxisControlData& axes = vis_write.serial.axes;
    packet.yaw_big_mode = axes.yaw_big_mode;
    packet.yaw_big_target_position = axes.yaw_big_target_position;
    packet.yaw_big_target_velocity = axes.yaw_big_target_velocity;
    packet.yaw_big_torque = axes.yaw_big_torque;
    packet.roll_mode = axes.roll_mode;
    packet.roll_target_position = axes.roll_target_position;
    packet.roll_target_velocity = axes.roll_target_velocity;
    packet.roll_torque = axes.roll_torque;
    packet.yaw_small_mode = axes.yaw_small_mode;
    packet.yaw_small_target_position = axes.yaw_small_target_position;
    packet.yaw_small_target_velocity = axes.yaw_small_target_velocity;
    packet.yaw_small_torque = axes.yaw_small_torque;
    packet.pitch_mode = axes.pitch_mode;
    packet.pitch_target_position = axes.pitch_target_position;
    packet.pitch_target_velocity = axes.pitch_target_velocity;
    packet.pitch_torque = axes.pitch_torque;
    packet.deform_control = nav_write.serial.deform_chassis.deform_control;
    packet.displacement_target_velocity_x =
        nav_write.serial.deform_chassis.displacement_target_velocity_x;
    packet.displacement_target_velocity_y =
        nav_write.serial.deform_chassis.displacement_target_velocity_y;
    return packet;
}

}  // namespace

int main() {
    using namespace s2c;

    const std::string shm_name = "/s2c_test_serial_comm_" + std::to_string(::getpid());
    s2c_test::PtyPair mcu_pty = s2c_test::makePtyPair();
    s2c_test::PtyPair imu_pty = s2c_test::makePtyPair();

    serial_comm::SerialCommServer::Config config;
    config.shm_name = shm_name;
    config.mcu_port = mcu_pty.slave_path;
    config.imu_port = imu_pty.slave_path;

    serial_comm::SerialCommServer server(config);
    McuSendFrameReader reader(mcu_pty.master_fd);
    server.start();

    serial_comm::SerialCommClient vis_client(shm_name);
    serial_comm::SerialCommClient nav_client(shm_name);

    const serial_comm::VisWrite vis_write = makeInitialVisWrite();
    const serial_comm::NavWrite nav_write = makeInitialNavWrite();
    S2C_CHECK(vis_client.vis_write(vis_write));
    S2C_CHECK(nav_client.nav_write(nav_write));

    serial_comm::AllInfo info{};
    S2C_CHECK(waitFor([&] {
        return vis_client.vis_get_all_info(info)
            && info.server_info.last_vision_write_time > 0.0
            && info.server_info.last_nav_write_time > 0.0
            && info.server_info.last_mcu_receive_time == 0.0
            && info.server_info.last_imu_receive_time == 0.0;
    }, std::chrono::milliseconds(2000)));

    // 1. 要发送的数据（视觉部分 + 导航部分合并）
    S2C_CHECK(info.send_data.fire_control.friction_wheel_enable == 1);
    S2C_CHECK(info.send_data.fire_control.fire == 1);
    S2C_CHECK(std::memcmp(&info.send_data.axes, &vis_write.serial.axes,
                          sizeof(vis_write.serial.axes)) == 0);
    S2C_CHECK(std::memcmp(&info.send_data.deform_chassis, &nav_write.serial.deform_chassis,
                          sizeof(nav_write.serial.deform_chassis)) == 0);

    // 2/3. 电控与 IMU 数据此时尚未收到
    S2C_CHECK(info.mcu_data.mark == 0);
    S2C_CHECK(info.imu_data.dt_one_tenth_ms == 0);

    // 4/5. 两份互通信内容
    S2C_CHECK(std::memcmp(&info.vis_to_nav, &vis_write.vision_to_nav,
                          sizeof(vis_write.vision_to_nav)) == 0);
    S2C_CHECK(std::memcmp(&info.nav_to_vision, &nav_write.nav_to_vision,
                          sizeof(nav_write.nav_to_vision)) == 0);

    // 6. 服务端信息
    S2C_CHECK(info.server_info.timestamp > 0.0);
    S2C_CHECK(info.server_info.last_vision_write_time > 0.0);
    S2C_CHECK(info.server_info.last_nav_write_time > 0.0);
    S2C_CHECK(info.server_info.timestamp >= info.server_info.last_vision_write_time);

    // 导航部分的读取方法拿到同一份完整信息
    serial_comm::AllInfo nav_info{};
    S2C_CHECK(nav_client.nav_get_all_info(nav_info));
    S2C_CHECK(std::memcmp(&nav_info.send_data, &info.send_data, sizeof(info.send_data)) == 0);

    // 服务端按 300Hz 发出的帧内容正确
    S2C_CHECK(waitFor([&] {
        const McuSendFrameReader::Snapshot snapshot = reader.snapshot();
        return snapshot.has_packet
            && snapshot.packet.friction_wheel_enable == 1
            && snapshot.packet.deform_control == 0x0A
            && snapshot.packet.yaw_big_target_position == 1.25;
    }, std::chrono::milliseconds(2000)));

    const com::mcu::SendPacket expected = buildExpectedSendPacket(vis_write, nav_write);
    {
        const McuSendFrameReader::Snapshot snapshot = reader.snapshot();
        S2C_CHECK(snapshot.packet.data_size == sizeof(com::mcu::SendPacket) - 5);
        // 帧尾 CRC 由发送层填写，比较时排除最后一个字节
        S2C_CHECK(std::memcmp(&snapshot.packet, &expected, sizeof(expected) - 1) == 0);
        S2C_CHECK(snapshot.crc_errors == 0);
    }

    // 频率: 300Hz
    const auto frequency_start = std::chrono::steady_clock::now();
    const uint64_t frames_before = reader.snapshot().count;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    const uint64_t frames_after = reader.snapshot().count;
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - frequency_start).count();
    const double rate_hz = static_cast<double>(frames_after - frames_before) / elapsed;
    std::printf("[test] 串口发送频率 = %.1f Hz\n", rate_hz);
    S2C_CHECK(rate_hz > serial_comm::kSendRateHz - kFrequencyToleranceHz);
    S2C_CHECK(rate_hz < serial_comm::kSendRateHz + kFrequencyToleranceHz);

    // 服务端统计的 sendLoop 实际帧率（滑动窗口）应在合理范围内
    serial_comm::AllInfo fps_info{};
    S2C_CHECK(waitFor([&] {
        return vis_client.vis_get_all_info(fps_info)
            && fps_info.server_info.send_rate_hz > 0.0;
    }, std::chrono::milliseconds(2000)));
    S2C_CHECK(fps_info.server_info.send_rate_hz > serial_comm::kSendRateHz - kFrequencyToleranceHz);
    S2C_CHECK(fps_info.server_info.send_rate_hz < serial_comm::kSendRateHz + kFrequencyToleranceHz);

    // 电控与 IMU 数据: 写帧到 pty 主端
    const com::mcu::ReceivePacket mcu_sample = makeMcuSample();
    const com::imu::ReceivePacket imu_sample = makeImuSample();
    const std::vector<uint8_t> mcu_frame = s2c_test::makeMcuReceiveFrame(mcu_sample);
    const std::vector<uint8_t> imu_frame = s2c_test::makeImuReceiveFrame(imu_sample);
    s2c_test::writeAll(mcu_pty.master_fd, mcu_frame.data(), mcu_frame.size());
    s2c_test::writeAll(imu_pty.master_fd, imu_frame.data(), imu_frame.size());

    S2C_CHECK(waitFor([&] {
        return vis_client.vis_get_all_info(info)
            && info.server_info.last_mcu_receive_time > 0.0
            && info.server_info.last_imu_receive_time > 0.0
            && info.mcu_data.mark == mcu_sample.mark
            && info.imu_data.dt_one_tenth_ms == imu_sample.dt_one_tenth_ms;
    }, std::chrono::milliseconds(2000)));
    // 帧尾 CRC 字段由串口链路填写，比较时排除
    S2C_CHECK(std::memcmp(&info.mcu_data, &mcu_sample, sizeof(mcu_sample) - 1) == 0);
    S2C_CHECK(std::memcmp(&info.imu_data, &imu_sample, sizeof(imu_sample) - 4) == 0);
    S2C_CHECK(info.server_info.last_mcu_receive_time <= info.server_info.timestamp);
    S2C_CHECK(info.server_info.last_imu_receive_time <= info.server_info.timestamp);

    // 服务端时间戳持续更新
    serial_comm::AllInfo later{};
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    S2C_CHECK(vis_client.vis_get_all_info(later));
    S2C_CHECK(later.server_info.timestamp > info.server_info.timestamp);

    // 写入范围互不影响: 只改视觉内容后变形部分保持，反之亦然
    serial_comm::VisWrite vis_only{};
    vis_only.serial.fire_control.friction_wheel_enable = 0;
    vis_only.serial.axes.yaw_big_mode = com::mcu::AXIS_MODE_TORQUE_PLUS_PID;
    vis_only.serial.axes.yaw_big_target_position = 2.5;
    vis_only.vision_to_nav.targets[0].recognized = 1;
    S2C_CHECK(vis_client.vis_write(vis_only));
    S2C_CHECK(waitFor([&] {
        return vis_client.vis_get_all_info(info)
            && info.send_data.fire_control.friction_wheel_enable == 0
            && info.send_data.axes.yaw_big_target_position == 2.5;
    }, std::chrono::milliseconds(2000)));
    S2C_CHECK(std::memcmp(&info.send_data.deform_chassis, &nav_write.serial.deform_chassis,
                          sizeof(nav_write.serial.deform_chassis)) == 0);
    S2C_CHECK(std::memcmp(&info.nav_to_vision, &nav_write.nav_to_vision,
                          sizeof(nav_write.nav_to_vision)) == 0);

    serial_comm::NavWrite nav_only{};
    nav_only.serial.deform_chassis.deform_control = 0x05;
    nav_only.serial.deform_chassis.displacement_target_velocity_x = -1.0f;
    nav_only.serial.deform_chassis.displacement_target_velocity_y = 2.0f;
    nav_only.nav_to_vision.suggested_big_yaw = 1.5f;
    S2C_CHECK(nav_client.nav_write(nav_only));
    S2C_CHECK(waitFor([&] {
        return nav_client.nav_get_all_info(info)
            && info.send_data.deform_chassis.deform_control == 0x05
            && info.send_data.deform_chassis.displacement_target_velocity_y == 2.0f;
    }, std::chrono::milliseconds(2000)));
    S2C_CHECK(info.send_data.fire_control.friction_wheel_enable == 0);
    S2C_CHECK(info.send_data.axes.yaw_big_mode == com::mcu::AXIS_MODE_TORQUE_PLUS_PID);
    S2C_CHECK(info.send_data.axes.yaw_big_target_position == 2.5);

    // 视觉与导航两个客户端并发写入，各自的写入范围互不破坏
    serial_comm::SerialCommClient vis_writer(shm_name);
    serial_comm::SerialCommClient nav_writer(shm_name);
    const float expected_pitch_torque = static_cast<float>(49) * 0.1f;
    std::thread vis_thread([&] {
        for (int i = 0; i < 50; ++i) {
            serial_comm::VisWrite write{};
            write.serial.fire_control.fire = static_cast<uint8_t>(i % 2);
            write.serial.axes.pitch_torque = static_cast<float>(i) * 0.1f;
            write.vision_to_nav.auto_aim_state = static_cast<uint8_t>(i);
            write.vision_to_nav.target_id = static_cast<uint8_t>(i);
            S2C_CHECK(vis_writer.vis_write(write));
        }
    });
    std::thread nav_thread([&] {
        for (int i = 0; i < 50; ++i) {
            serial_comm::NavWrite write{};
            write.serial.deform_chassis.deform_control = static_cast<uint8_t>(i % 7);
            write.nav_to_vision.suggested_target_id = static_cast<uint8_t>(i);
            S2C_CHECK(nav_writer.nav_write(write));
        }
    });
    vis_thread.join();
    nav_thread.join();
    S2C_CHECK(waitFor([&] {
        return vis_client.vis_get_all_info(info)
            && info.send_data.axes.pitch_torque == expected_pitch_torque
            && info.send_data.deform_chassis.deform_control == (49 % 7)
            && info.nav_to_vision.suggested_target_id == 49;
    }, std::chrono::milliseconds(2000)));
    S2C_CHECK(info.send_data.fire_control.fire == static_cast<uint8_t>(49 % 2));
    S2C_CHECK(info.send_data.axes.yaw_big_target_position == 0.0);
    S2C_CHECK(info.vis_to_nav.auto_aim_state == 49);
    S2C_CHECK(info.vis_to_nav.target_id == 49);
    S2C_CHECK(info.nav_to_vision.suggested_target_id == 49);

    reader.stop();
    server.stop();
    s2c_test::closePtyPair(mcu_pty);
    s2c_test::closePtyPair(imu_pty);
    std::printf("所有串口通信测试通过\n");
    return 0;
}
