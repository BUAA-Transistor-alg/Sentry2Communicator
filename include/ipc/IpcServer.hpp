// IpcServer.hpp — 本机进程间通信服务端（共享内存 + 信号量）
//
// 用法:
//   IpcServer server("/s2c_ipc");
//   server.publish(kStatusChannel, status);                 // 设定客户端可获取的内容
//   server.setHandler(kCommandChannel, [](const Command& c) { ... });  // 收到请求时回调
//   server.start();                                         // 启动请求处理线程
//   ...
//   server.stop();
//
// 约定:
//   · 获取通道编号与发送通道编号各自独立，取值 [0, kChannelCount)
//   · 回调在服务端的请求处理线程上串行调用，回调里不要做长时间阻塞操作
//   · setHandler 必须在 start() 之前调用；publish 可以在任意时刻调用
#pragma once

#include "ipc/IpcLayout.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <new>
#include <string>
#include <thread>
#include <type_traits>

namespace s2c {
namespace ipc {

class IpcServer {
public:
    static constexpr const char* kDefaultName = "/s2c_ipc";

    // 创建共享内存并初始化同步原语；同名共享内存已有服务端运行时抛异常
    explicit IpcServer(const std::string& name = kDefaultName);
    ~IpcServer();

    IpcServer(const IpcServer&) = delete;
    IpcServer& operator=(const IpcServer&) = delete;

    // ── 获取通道: 服务端设定内容，客户端 get 读取 ──
    template <typename T>
    void publish(uint32_t channel, const T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "获取内容必须是可逐字节复制的类型");
        static_assert(sizeof(T) <= kMaxPayload, "获取内容超过单通道载荷上限");
        publishBytes(channel, &value, sizeof(T));
    }

    // ── 发送通道: 客户端 send 后由请求处理线程调用回调 ──
    template <typename T>
    void setHandler(uint32_t channel, std::function<void(const T&)> handler) {
        static_assert(std::is_trivially_copyable<T>::value, "发送内容必须是可逐字节复制的类型");
        static_assert(sizeof(T) <= kMaxPayload, "发送内容超过单条请求的载荷上限");
        setHandlerBytes(channel,
                        [handler = std::move(handler)](const void* data, uint32_t size) {
                            if (size != sizeof(T)) return;   // 与注册类型不符的请求丢弃
                            alignas(T) unsigned char buffer[sizeof(T)];
                            std::memcpy(buffer, data, sizeof(T));
                            handler(*std::launder(reinterpret_cast<const T*>(buffer)));
                        });
    }

    void start();   // 启动请求处理线程
    void stop();    // 停止请求处理线程（未启动时为空操作）

    struct Stats {
        uint64_t enqueued = 0;   // 客户端累计入队请求数
        uint64_t dropped = 0;    // 队列满时被覆盖的请求数
        uint64_t handled = 0;    // 已处理并回调的请求数
    };
    Stats stats() const;

private:
    void publishBytes(uint32_t channel, const void* data, size_t size);
    void setHandlerBytes(uint32_t channel, std::function<void(const void*, uint32_t)> handler);
    void requestLoop();
    void removeStaleSegment();

    std::string name_;
    int shm_fd_ = -1;
    SharedLayout* layout_ = nullptr;

    std::array<std::function<void(const void*, uint32_t)>, kChannelCount> handlers_{};
    std::thread request_thread_;
    std::atomic<bool> stop_{false};
    std::atomic<uint64_t> handled_{0};
};

}  // namespace ipc
}  // namespace s2c
