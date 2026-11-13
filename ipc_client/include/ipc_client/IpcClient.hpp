// IpcClient.hpp — 本机进程间通信客户端（共享内存 + 信号量）
//
// 只有两个通信方法:
//   get   : 读取服务端提前设定好的内容
//   send  : 把内容发送给服务端，服务端在接收到时调用回调
//
// 并发与耗时保证:
//   · 多个客户端进程可以同时使用同一个服务端；获取通道与请求队列都使用进程共享
//     robust 互斥量保护，临界区内只做一次内存拷贝
//   · get 直接读共享内存，send 入队后立即返回；客户端不等待信号量、不设置超时
//   · 服务端进程死亡时，它持有的 robust 互斥量由内核标记，下一个加锁者拿到
//     EOWNERDEAD 后接管；客户端同时检查服务端状态与进程是否存活，失效时调用直接返回 false
//   · 请求队列满时覆盖最旧的一条请求，客户端不会因为服务端不消费而阻塞
//
// 通道:
//   · 获取通道与发送通道编号各自独立，取值 [0, kChannelCount)
//   · 同一条通道上客户端与服务端的内容类型必须一致（可逐字节复制，长度不超过 kMaxPayload）
#pragma once

#include "ipc/IpcLayout.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>

namespace s2c {
namespace ipc {

class IpcClient {
public:
    static constexpr const char* kDefaultName = "/s2c_ipc";

    // 记录共享内存名字；服务端未启动时不影响构造，调用时自动尝试附加
    explicit IpcClient(const std::string& name = kDefaultName);
    ~IpcClient();

    IpcClient(const IpcClient&) = delete;
    IpcClient& operator=(const IpcClient&) = delete;

    // 获取服务端发布的内容；服务端未启动、该通道从未发布或类型不一致时返回 false
    template <typename T>
    bool get(uint32_t channel, T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "获取内容必须是可逐字节复制的类型");
        static_assert(sizeof(T) <= kMaxPayload, "获取内容超过单通道载荷上限");
        return getBytes(channel, &value, sizeof(T));
    }

    // 把内容发送给服务端；成功入队返回 true，服务端未启动返回 false
    template <typename T>
    bool send(uint32_t channel, const T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "发送内容必须是可逐字节复制的类型");
        static_assert(sizeof(T) <= kMaxPayload, "发送内容超过单条请求的载荷上限");
        return sendBytes(channel, &value, sizeof(T));
    }

private:
    bool getBytes(uint32_t channel, void* out, size_t expected);
    bool sendBytes(uint32_t channel, const void* data, size_t size);
    bool ensureAttached();
    bool attach();
    void detach();
    bool serverRunning() const;

    std::string name_;
    SharedLayout* layout_ = nullptr;
};

}  // namespace ipc
}  // namespace s2c
