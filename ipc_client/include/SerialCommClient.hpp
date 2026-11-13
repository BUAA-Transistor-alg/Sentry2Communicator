// SerialCommClient.hpp — 串口通信客户端（视觉部分 / 导航部分）
//
// 服务端在主工程里（s2c::serial_comm::SerialCommServer），维护串口收发数据并按 500Hz
// 发送；本客户端通过共享内存读取服务端发布的完整快照，或按各自范围提交写入内容。
//
// 方法前缀区分两个部分:
//   vis_ 视觉: 读取全部信息；写入火控、四个轴的控制部分、视觉向导航通信
//   nav_ 导航: 读取全部信息；写入变形与底盘位移、导航向视觉通信
//
// 两部分读取到的都是同一份 AllInfo（6 个结构体: 要发送的数据、最新电控数据、
// 最新 IMU 数据、视觉向导航通信、导航向视觉通信、服务端信息）。
//
// 本文件是 ipc_client 对外唯一接口；其余头文件属于内部实现，外部工程只需包含本文件。
#pragma once

#include "ipc_client/IpcClient.hpp"
#include "serial_comm/SerialCommTypes.hpp"

#include <string>

namespace s2c {
namespace serial_comm {

class SerialCommClient {
public:
    explicit SerialCommClient(const std::string& name = kDefaultShmName);
    ~SerialCommClient();

    SerialCommClient(const SerialCommClient&) = delete;
    SerialCommClient& operator=(const SerialCommClient&) = delete;

    // ── 视觉部分 ──
    bool vis_get_all_info(AllInfo& info);          // 读取当前全部信息
    bool vis_write(const VisWrite& write);         // 写入火控/四轴与视觉向导航通信

    // ── 导航部分 ──
    bool nav_get_all_info(AllInfo& info);          // 读取当前全部信息
    bool nav_write(const NavWrite& write);         // 写入变形与底盘位移与导航向视觉通信

private:
    ipc::IpcClient client_;
};

}  // namespace serial_comm
}  // namespace s2c
