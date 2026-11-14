// IpcClient.hpp — 本机进程间通信客户端（共享内存 + 信号量）
//
// 只有两个通信方法:
//   get   : 读取服务端提前设定好的内容
//   send  : 把内容发送给服务端，服务端在接收到时调用回调
//
// 并发与耗时保证:
//   · 多个客户端进程可以同时使用同一个服务端；获取通道与请求队列都使用进程共享
//     robust 互斥量保护；send 在临界区内复查存活并拷贝一次，get 在进入临界区前检查存活
//   · get 直接读共享内存，send 入队后立即返回；客户端不等待信号量、不设置超时
//   · 服务端进程死亡或调用 stop 时，客户端检查共享内存状态与进程存活，调用直接返回 false；
//     get 与 stop 恰好并发、或存活检查通过后服务端立即被杀时，可能返回一次停止前的完整快照，
//     下一次调用返回 false；
//     服务端进程存活但持有互斥量不释放时，按互斥量语义阻塞等待，不设置超时
//   · 请求队列满时覆盖最旧的一条请求，客户端不会因为服务端不消费而阻塞
//   · send 是尽力入队：与服务端 stop 并发时，可能返回 true 但请求随后被丢弃
//   · 同一个客户端对象不要跨线程共用；每个线程使用独立的客户端对象
//   · 通道编号越界抛 std::out_of_range，载荷超过 kMaxPayload 抛 std::length_error
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
    uint64_t shm_dev_ = 0;   // 当前映射对应的共享内存 inode，用于判断是否需要重新附加
    uint64_t shm_ino_ = 0;
};

}  // namespace ipc
}  // namespace s2c
