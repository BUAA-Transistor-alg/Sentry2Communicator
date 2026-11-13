// src/serial_comm_server.cpp — 串口通信服务端可执行程序
//
// 运行 s2c::serial_comm::SerialCommServer: 打开电控与 IMU 串口，维护串口收发数据，
// 按 500Hz 向电控发送，并把完整快照发布到共享内存供客户端读取。
//
// 运行: ./build/s2c_serial_comm_server [共享内存名字]，Ctrl+C 退出
// 串口默认按 USB iProduct 筛选: iProduct 为 AutoAim_IMU_Com 的给 IMU，其余给电控。
#include "serial_comm/SerialCommServer.hpp"

#include "common/Debug/WatchdogFeed.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

namespace {

std::atomic<bool> keep_running{true};

void signalHandler(int /*signum*/) { keep_running = false; }

}  // namespace

int main(int argc, char** argv) {
    const std::string shm_name =
        argc > 1 ? argv[1] : s2c::serial_comm::kDefaultShmName;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    s2c::serial_comm::SerialCommServer::Config config;
    config.shm_name = shm_name;

    s2c::serial_comm::SerialCommServer server(config);
    server.start();
    // 由 auto_launch/watchdog.py 启动时环境变量里带有所需参数；手动运行时为空操作
    WatchdogFeed::instance().start();

    std::printf("[serial-comm-server] 共享内存: %s\n", shm_name.c_str());
    std::printf("[serial-comm-server] 串口发送频率: %.0f Hz，Ctrl+C 退出\n",
                s2c::serial_comm::kSendRateHz);

    while (keep_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    server.stop();
    WatchdogFeed::instance().stop();
    std::printf("[serial-comm-server] 已停止\n");
    return 0;
}
