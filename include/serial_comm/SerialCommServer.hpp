// SerialCommServer.hpp — 串口通信服务端
//
// 职责:
//   · 维护"要发送给串口的数据"（火控 + 四个轴 + 变形与底盘位移）与"串口收到的最新数据"
//     （电控包、IMU 包），以及视觉/导航写入的互通信内容与服务端时间戳
//   · 以 kSendRateHz（300Hz）为周期组装 com::mcu::SendPacket 发给电控，
//     同时把一份完整快照（AllInfo）发布到共享内存供客户端读取
//   · 通过 IPC 服务端接收视觉写入与导航写入，写入范围由两种写入结构体区分
//
// 频率保持方式与参考工程 TorqueControllerForBigSmallYaw_v2 一致:
//   每轮重新取 now() 作为起点再 sleep_until(start + 周期)，误差不跨轮累积；
//   每轮固定的唤醒过冲使实际频率约 486Hz，参考工程采用同样方式，不额外补偿。
#pragma once

#include "Communications.hpp"
#include "common/FrameRateCounter.h"
#include "ipc/IpcServer.hpp"
#include "serial_comm/SerialCommTypes.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace s2c {
namespace serial_comm {

class SerialCommServer {
public:
    struct Config {
        std::string shm_name = kDefaultShmName;   // 共享内存名字（以 / 开头）
        std::string mcu_port;                     // 空串: 按 USB iProduct 筛选
        std::string imu_port;                     // 空串: 按 USB iProduct 筛选
        double send_rate_hz = kSendRateHz;        // 向串口发送与发布快照的频率
    };

    static Config defaultConfig() { return Config{}; }

    explicit SerialCommServer(const Config& config = defaultConfig());
    ~SerialCommServer();

    SerialCommServer(const SerialCommServer&) = delete;
    SerialCommServer& operator=(const SerialCommServer&) = delete;

    void start();   // 启动 IPC 请求线程与 300Hz 发送线程
    void stop();    // 停止发送线程与 IPC 请求线程并置 IPC 为停止态；串口接收线程随对象析构停止
    bool failed() const { return send_failed_.load(std::memory_order_acquire); }

private:
    // 服务端维护的全部状态，统一由 state_mutex_ 保护
    struct Maintained {
        SerialSendData          send_data{};
        com::mcu::ReceivePacket mcu_packet{};
        com::imu::ReceivePacket imu_packet{};
        VisToNav                vis_to_nav{};
        NavToVision             nav_to_vision{};
        double last_mcu_receive_time = 0.0;
        double last_imu_receive_time = 0.0;
        double last_vision_write_time = 0.0;
        double last_nav_write_time = 0.0;
    };

    void onMcuReceive(const com::mcu::ReceivePacket& packet);
    void onImuReceive(const com::imu::ReceivePacket& packet);
    void onVisWrite(const VisWrite& write);
    void onNavWrite(const NavWrite& write);
    void sendLoop();

    std::mutex state_mutex_;
    Maintained state_;
    double send_rate_hz_;
    FrameRateCounter send_fps_counter_{300};   // sendLoop 实际帧率（300 帧滑动窗口，300Hz 下约 1s）

    ipc::IpcServer ipc_server_;
    com::McuCommunication mcu_serial_;
    com::ImuCommunication imu_serial_;

    std::thread send_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> send_failed_{false};   // 发送线程异常退出标志
};

}  // namespace serial_comm
}  // namespace s2c
