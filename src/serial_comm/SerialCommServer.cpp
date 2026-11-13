#include "serial_comm/SerialCommServer.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>

namespace s2c {
namespace serial_comm {

SerialCommServer::SerialCommServer(const Config& config)
    : send_rate_hz_(config.send_rate_hz)
    , ipc_server_(config.shm_name)
    , mcu_serial_([this](const com::mcu::ReceivePacket& packet) { onMcuReceive(packet); },
                  false, config.mcu_port)
    , imu_serial_([this](const com::imu::ReceivePacket& packet) { onImuReceive(packet); },
                  false, config.imu_port) {
    if (send_rate_hz_ <= 0.0) {
        throw std::invalid_argument("SerialCommServer: 发送频率必须为正值");
    }
    // 串口收发线程在构造末尾启动；此时全部成员都已构造完成
    mcu_serial_.startWorker();
    imu_serial_.startWorker();

    ipc_server_.setHandler<VisWrite>(kVisWriteChannel,
                                     [this](const VisWrite& write) { onVisWrite(write); });
    ipc_server_.setHandler<NavWrite>(kNavWriteChannel,
                                     [this](const NavWrite& write) { onNavWrite(write); });
}

SerialCommServer::~SerialCommServer() {
    stop();
    // 成员按声明逆序析构: 串口线程先于 state_ 停止，IPC 服务端最后释放共享内存
}

void SerialCommServer::start() {
    if (running_.load(std::memory_order_acquire)) {
        throw std::logic_error("SerialCommServer::start: 已在运行");
    }
    ipc_server_.start();
    running_.store(true, std::memory_order_release);
    send_thread_ = std::thread([this] { sendLoop(); });
}

void SerialCommServer::stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) return;
    send_thread_.join();
    ipc_server_.stop();
}

void SerialCommServer::onMcuReceive(const com::mcu::ReceivePacket& packet) {
    const com::mcu::ReceivePacket processed = preprocessor_.processReceive(packet);
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.mcu_packet = processed;
    state_.last_mcu_receive_time = steadySeconds(std::chrono::steady_clock::now());
}

void SerialCommServer::onImuReceive(const com::imu::ReceivePacket& packet) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.imu_packet = packet;
    state_.last_imu_receive_time = steadySeconds(std::chrono::steady_clock::now());
}

void SerialCommServer::onVisWrite(const VisWrite& write) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.send_data.fire_control = write.serial.fire_control;
    state_.send_data.axes = write.serial.axes;
    state_.vis_to_nav = write.vision_to_nav;
    state_.last_vision_write_time = steadySeconds(std::chrono::steady_clock::now());
}

void SerialCommServer::onNavWrite(const NavWrite& write) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.send_data.deform_chassis = write.serial.deform_chassis;
    state_.nav_to_vision = write.nav_to_vision;
    state_.last_nav_write_time = steadySeconds(std::chrono::steady_clock::now());
}

void SerialCommServer::sendLoop() {
    const std::chrono::duration<double> period(1.0 / send_rate_hz_);
    while (running_.load(std::memory_order_acquire)) {
        const auto start = std::chrono::steady_clock::now();

        com::mcu::SendPacket packet{};   // 帧头、协议版本、data_size 由默认值给出
        AllInfo info;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            packet.friction_wheel_enable = state_.send_data.fire_control.friction_wheel_enable;
            packet.fire = state_.send_data.fire_control.fire;

            packet.yaw_big_mode = state_.send_data.axes.yaw_big_mode;
            packet.yaw_big_target_position = state_.send_data.axes.yaw_big_target_position;
            packet.yaw_big_target_velocity = state_.send_data.axes.yaw_big_target_velocity;
            packet.yaw_big_torque = state_.send_data.axes.yaw_big_torque;

            packet.roll_mode = state_.send_data.axes.roll_mode;
            packet.roll_target_position = state_.send_data.axes.roll_target_position;
            packet.roll_target_velocity = state_.send_data.axes.roll_target_velocity;
            packet.roll_torque = state_.send_data.axes.roll_torque;

            packet.yaw_small_mode = state_.send_data.axes.yaw_small_mode;
            packet.yaw_small_target_position = state_.send_data.axes.yaw_small_target_position;
            packet.yaw_small_target_velocity = state_.send_data.axes.yaw_small_target_velocity;
            packet.yaw_small_torque = state_.send_data.axes.yaw_small_torque;

            packet.pitch_mode = state_.send_data.axes.pitch_mode;
            packet.pitch_target_position = state_.send_data.axes.pitch_target_position;
            packet.pitch_target_velocity = state_.send_data.axes.pitch_target_velocity;
            packet.pitch_torque = state_.send_data.axes.pitch_torque;

            packet.deform_control = state_.send_data.deform_chassis.deform_control;
            packet.displacement_target_velocity_x =
                state_.send_data.deform_chassis.displacement_target_velocity_x;
            packet.displacement_target_velocity_y =
                state_.send_data.deform_chassis.displacement_target_velocity_y;

            info.send_data = state_.send_data;
            info.mcu_data = state_.mcu_packet;
            info.imu_data = state_.imu_packet;
            info.vis_to_nav = state_.vis_to_nav;
            info.nav_to_vision = state_.nav_to_vision;
            info.server_info.timestamp = steadySeconds(std::chrono::steady_clock::now());
            info.server_info.last_mcu_receive_time = state_.last_mcu_receive_time;
            info.server_info.last_imu_receive_time = state_.last_imu_receive_time;
            info.server_info.last_vision_write_time = state_.last_vision_write_time;
            info.server_info.last_nav_write_time = state_.last_nav_write_time;
        }

        mcu_serial_.sendData(packet);
        ipc_server_.publish(kAllInfoChannel, info);

        std::this_thread::sleep_until(start + period);
    }
}

}  // namespace serial_comm
}  // namespace s2c
