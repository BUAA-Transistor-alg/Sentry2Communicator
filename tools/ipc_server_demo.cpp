// tools/ipc_server_demo.cpp — 服务端使用示例
//
// 发布一条获取通道的内容（每 100ms 更新一次），并在收到客户端发送的命令时打印。
// 运行: ./build/s2c_ipc_server_demo [共享内存名字]，Ctrl+C 退出
#include "ipc/IpcServer.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cinttypes>
#include <cstdio>
#include <string>
#include <thread>

namespace {

std::atomic<bool> keep_running{true};

void signalHandler(int /*signum*/) { keep_running = false; }

// 与客户端示例约定好的内容类型（只演示通信，不含实际业务逻辑）
struct ServerStatus {
    uint32_t counter;
    float value;
};

struct Command {
    uint32_t client_id;
    float aim_pitch;
};

constexpr uint32_t kStatusChannel = 0;
constexpr uint32_t kCommandChannel = 0;

}  // namespace

int main(int argc, char** argv) {
    const std::string name = argc > 1 ? argv[1] : s2c::ipc::IpcServer::kDefaultName;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    s2c::ipc::IpcServer server(name);
    server.setHandler<Command>(kCommandChannel, [](const Command& command) {
        std::printf("[server] 收到客户端 %u 的命令: aim_pitch=%.3f\n",
                    command.client_id, static_cast<double>(command.aim_pitch));
    });
    server.start();
    std::printf("[server] 共享内存: %s\n", name.c_str());

    uint32_t counter = 0;
    while (keep_running) {
        const ServerStatus status{counter++, static_cast<float>(counter) * 0.5f};
        server.publish(kStatusChannel, status);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const s2c::ipc::IpcServer::Stats stats = server.stats();
    std::printf("[server] 入队 %" PRIu64 "，处理 %" PRIu64 "，丢弃 %" PRIu64 "\n",
                stats.enqueued, stats.handled, stats.dropped);
    server.stop();
    return 0;
}
