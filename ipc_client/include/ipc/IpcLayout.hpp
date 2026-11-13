// IpcLayout.hpp — 本机进程间通信的共享内存布局与同步原语
//
// 通信方式: 共享内存 + 信号量 + 进程共享 robust 互斥量
//   · 共享内存: 服务端按名字创建并初始化，客户端按同一名字附加
//   · 信号量: 客户端把请求放入队列后 post，服务端的请求线程 wait 后取出并调用回调
//   · 进程共享 robust 互斥量: 保护获取通道与请求队列。持有者进程死亡时由内核标记，
//     下一个加锁者拿到 EOWNERDEAD 并接管（pthread_mutex_consistent），因此客户端
//     调用不会因为服务端已失效而永久阻塞
//
// 客户端不等待任何信号量、不设置任何超时: 获取通道直接读共享内存，发送通道入队后
// 立即返回，队列满时覆盖最旧的一条请求。客户端每次调用的耗时只包含若干次系统调用
// 与一次临界区内的内存拷贝。
#pragma once

#include <pthread.h>
#include <semaphore.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <system_error>

namespace s2c {
namespace ipc {

// ── 可调参数 ──
constexpr uint32_t kChannelCount  = 16;    // 获取通道与发送通道各自的最大数量
constexpr size_t   kMaxPayload    = 1024;  // 单个通道内容 / 单条请求的载荷上限（字节）
constexpr uint32_t kQueueCapacity = 128;   // 请求队列容量

constexpr uint64_t kMagic   = 0x5345433249504331ULL;  // "SEC2IPC1"
constexpr uint32_t kVersion = 1;

// 服务端状态（写在共享内存头部，客户端据此判断能否使用该段）
enum class ServerState : uint32_t {
    kStopped = 0,
    kRunning = 1,
};

// 获取通道: 服务端发布、客户端读取。
// 双缓冲: 服务端先写非活动缓冲区，再切换活动索引，发布过程中进程死亡也不会留下半份数据。
struct GetChannelSlot {
    pthread_mutex_t mutex;
    uint32_t active;              // 当前有效缓冲区索引（0 / 1）
    uint32_t size[2];             // 各缓冲区的数据长度
    uint8_t  payload[2][kMaxPayload];
};

// 发送请求: 客户端写入、服务端取出
struct SendRequest {
    uint32_t channel;
    uint32_t size;
    uint8_t  payload[kMaxPayload];
};

struct RequestQueue {
    pthread_mutex_t mutex;
    sem_t available;              // 客户端入队后 post；服务端请求线程 wait
    uint32_t head;                // 出队位置
    uint32_t count;               // 当前请求数量
    uint64_t enqueued;            // 累计入队数量
    uint64_t dropped;             // 队列满时被覆盖的旧请求数量
    SendRequest requests[kQueueCapacity];
};

struct SharedLayout {
    uint64_t magic;
    uint32_t version;
    uint32_t state;               // ServerState
    int32_t  server_pid;
    GetChannelSlot get[kChannelCount];
    RequestQueue queue;
};

namespace detail {

[[noreturn]] inline void throwSystemError(int code, const char* what) {
    throw std::system_error(code, std::generic_category(), what);
}

// 初始化进程共享 + robust 互斥量（放在共享内存里，由服务端初始化）
inline void initializeRobustMutex(pthread_mutex_t* mutex) {
    pthread_mutexattr_t attr;
    int rc = pthread_mutexattr_init(&attr);
    if (rc != 0) throwSystemError(rc, "pthread_mutexattr_init");
    rc = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    if (rc != 0) {
        pthread_mutexattr_destroy(&attr);
        throwSystemError(rc, "pthread_mutexattr_setpshared");
    }
    rc = pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    if (rc != 0) {
        pthread_mutexattr_destroy(&attr);
        throwSystemError(rc, "pthread_mutexattr_setrobust");
    }
    rc = pthread_mutex_init(mutex, &attr);
    pthread_mutexattr_destroy(&attr);
    if (rc != 0) throwSystemError(rc, "pthread_mutex_init");
}

// 加锁: 上一个持有者死亡（例如服务端崩溃）时内核返回 EOWNERDEAD，
// 这里把互斥量标记为一致后继续使用，从而避免无限等待。
inline void lockRobust(pthread_mutex_t* mutex) {
    const int rc = pthread_mutex_lock(mutex);
    if (rc == 0) return;
    if (rc == EOWNERDEAD) {
        const int consistent_rc = pthread_mutex_consistent(mutex);
        if (consistent_rc != 0) throwSystemError(consistent_rc, "pthread_mutex_consistent");
        return;
    }
    throwSystemError(rc, "pthread_mutex_lock");
}

inline void unlockRobust(pthread_mutex_t* mutex) {
    const int rc = pthread_mutex_unlock(mutex);
    if (rc != 0) throwSystemError(rc, "pthread_mutex_unlock");
}

}  // namespace detail

}  // namespace ipc
}  // namespace s2c
