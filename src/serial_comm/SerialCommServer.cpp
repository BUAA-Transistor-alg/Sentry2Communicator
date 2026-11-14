#include "serial_comm/SerialCommServer.hpp"

#include "common/Debug/WatchdogFeed.h"

#include <chrono>
#include <cmath>
#include <cstdio>
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
    if (!std::isfinite(send_rate_hz_) || send_rate_hz_ <= 0.0) {
        throw std::invalid_argument("SerialCommServer: 发送频率必须为有限正值");
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
    // send_thread_ 已在 stop() 中 join；成员按声明逆序析构，串口线程先于 IPC 服务端释放
}

void SerialCommServer::start() {
    if (running_.load(std::memory_order_acquire)) {
        throw std::logic_error("SerialCommServer::start: 已在运行");
    }
    // 发送线程可能因异常提前退出但尚未回收，先 join 再复用 thread 对象
    if (send_thread_.joinable()) send_thread_.join();
    ipc_server_.start();
    send_fps_counter_.reset();   // 停止期间的空档不计入重启后的帧率
    send_failed_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    try {
        send_thread_ = std::thread([this] {
            try {
                sendLoop();
            } catch (...) {
                send_failed_.store(true, std::memory_order_release);
                running_.store(false, std::memory_order_release);
                try {
                    ipc_server_.stop();   // IPC 请求线程不应在发送线程死亡后继续对外服务
                } catch (...) {
                }
                std::fprintf(stderr, "SerialCommServer: 发送线程异常退出\n");
            }
        });
    } catch (...) {
        // 线程创建失败：回滚运行状态并停掉 IPC 请求线程，避免析构时 join 抛异常
        running_.store(false, std::memory_order_release);
        ipc_server_.stop();
        throw;
    }
}

void SerialCommServer::stop() {
    running_.store(false, std::memory_order_release);
    // 线程可能因异常提前退出，只要 joinable 就必须 join，避免析构时 std::terminate
    if (send_thread_.joinable()) send_thread_.join();
    // 无论是否 start 过都置 IPC 状态为 kStopped，避免构造后直接 stop 仍对客户端显示可用
    ipc_server_.stop();
}

void SerialCommServer::onMcuReceive(const com::mcu::ReceivePacket& packet) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.mcu_packet = com::McuDataPreprocessor::processReceive(packet);
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
        if (ipc_server_.requestFailed()) {
            send_failed_.store(true, std::memory_order_release);
            running_.store(false, std::memory_order_release);
            std::fprintf(stderr, "SerialCommServer: IPC 请求线程异常退出\n");
            break;
        }
        const auto start = std::chrono::steady_clock::now();
        send_fps_counter_.tick();   // 先更新帧率再读取，快照中的 send_rate_hz 不滞后一帧

        com::mcu::SendPacket packet{};   // 帧头、协议版本、data_size 由默认值给出
        com::mcu::SendPacket processed{};
        AllInfo info{};   // 值初始化，避免结构体填充字节把不确定数据发布到共享内存
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
            info.server_info.send_rate_hz = send_fps_counter_.fps();
            info.server_info.last_mcu_receive_time = state_.last_mcu_receive_time;
            info.server_info.last_imu_receive_time = state_.last_imu_receive_time;
            info.server_info.last_vision_write_time = state_.last_vision_write_time;
            info.server_info.last_nav_write_time = state_.last_nav_write_time;
            processed = com::McuDataPreprocessor::processSend(packet);
        }

        mcu_serial_.sendData(processed);
        ipc_server_.publish(kAllInfoChannel, info);

        // 每轮 300Hz 循环喂一次狗：循环停摆（死锁等）时 tick 停止，看门狗随之重启
        WatchdogFeed::instance().tick();

        std::this_thread::sleep_until(start + period);
    }
}

}  // namespace serial_comm
}  // namespace s2c
