#include "ipc/IpcServer.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace s2c {
namespace ipc {

namespace {

constexpr mode_t kSharedMemoryMode = 0666;

std::string checkName(const std::string& name) {
    if (name.empty() || name.front() != '/') {
        throw std::invalid_argument("IpcServer: 共享内存名字必须以 / 开头: " + name);
    }
    return name;
}

bool processAlive(int32_t pid) {
    if (pid <= 0) return false;
    if (::kill(pid, 0) != 0) return errno != ESRCH;
    // 已退出但尚未被父进程回收的僵尸进程不算存活
    char path[64];
    std::snprintf(path, sizeof(path), "/proc/%d/stat", static_cast<int>(pid));
    std::FILE* file = std::fopen(path, "r");
    if (file == nullptr) return true;
    char buffer[512];
    const bool read = std::fgets(buffer, sizeof(buffer), file) != nullptr;
    std::fclose(file);
    if (!read) return true;
    const char* end = std::strrchr(buffer, ')');
    if (end == nullptr || end[1] != ' ') return true;
    const size_t index = static_cast<size_t>(end - buffer);
    if (index + 2 >= sizeof(buffer)) return true;
    return buffer[index + 2] != 'Z';
}

}  // namespace

IpcServer::IpcServer(const std::string& name) : name_(checkName(name)) {
    removeStaleSegment();

    shm_fd_ = ::shm_open(name_.c_str(), O_CREAT | O_EXCL | O_RDWR, kSharedMemoryMode);
    if (shm_fd_ < 0) {
        if (errno == EEXIST) {
            throw std::runtime_error(
                "IpcServer: 同名共享内存已存在且无法清理: /dev/shm/" + name_.substr(1) +
                "（确认没有其他服务端在运行，并检查该文件的属主与权限）");
        }
        throw std::system_error(errno, std::generic_category(), "IpcServer: shm_open");
    }
    if (::ftruncate(shm_fd_, static_cast<off_t>(sizeof(SharedLayout))) != 0) {
        const int error = errno;
        ::close(shm_fd_);
        shm_fd_ = -1;
        ::shm_unlink(name_.c_str());
        throw std::system_error(error, std::generic_category(), "IpcServer: ftruncate");
    }
    // shm_open 的模式受 umask 影响；用 fchmod 明确所有同机用户都可读写
    if (::fchmod(shm_fd_, kSharedMemoryMode) != 0) {
        const int error = errno;
        ::close(shm_fd_);
        shm_fd_ = -1;
        ::shm_unlink(name_.c_str());
        throw std::system_error(error, std::generic_category(), "IpcServer: fchmod");
    }
    void* address = ::mmap(nullptr, sizeof(SharedLayout), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (address == MAP_FAILED) {
        const int error = errno;
        ::close(shm_fd_);
        shm_fd_ = -1;
        ::shm_unlink(name_.c_str());
        throw std::system_error(error, std::generic_category(), "IpcServer: mmap");
    }
    layout_ = static_cast<SharedLayout*>(address);

    // 新段的初始内容为 0；先初始化同步原语，再发布 magic/version，最后置为运行状态。
    // 客户端只有在看到 magic、version 与运行状态之后才会使用该段。
    try {
        std::memset(layout_, 0, sizeof(SharedLayout));
        for (uint32_t i = 0; i < kChannelCount; ++i) {
            detail::initializeRobustMutex(&layout_->get[i].mutex);
            layout_->get[i].active = 0;
        }
        detail::initializeRobustMutex(&layout_->queue.mutex);
        if (::sem_init(&layout_->queue.available, 1, 0) != 0) {
            const int error = errno;
            throw std::system_error(error, std::generic_category(), "IpcServer: sem_init");
        }
    } catch (...) {
        // 同步原语初始化失败：构造函数未完成，析构不会执行，需要在这里释放共享内存
        ::munmap(layout_, sizeof(SharedLayout));
        layout_ = nullptr;
        ::close(shm_fd_);
        shm_fd_ = -1;
        ::shm_unlink(name_.c_str());
        throw;
    }

    __atomic_store_n(&layout_->version, kVersion, __ATOMIC_RELEASE);
    __atomic_store_n(&layout_->server_pid, static_cast<int32_t>(::getpid()), __ATOMIC_RELEASE);
    __atomic_store_n(&layout_->magic, kMagic, __ATOMIC_RELEASE);
    // 构造后允许客户端在 start() 之前发布/入队；stop() 之后状态置为 kStopped
    __atomic_store_n(&layout_->state, static_cast<uint32_t>(ServerState::kRunning), __ATOMIC_RELEASE);
}

IpcServer::~IpcServer() {
    stop();
    if (layout_ != nullptr) {
        __atomic_store_n(&layout_->state, static_cast<uint32_t>(ServerState::kStopped), __ATOMIC_RELEASE);
        ::munmap(layout_, sizeof(SharedLayout));
        layout_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
    ::shm_unlink(name_.c_str());
}

void IpcServer::removeStaleSegment() {
    const int probe = ::shm_open(name_.c_str(), O_RDWR, 0);
    if (probe < 0) {
        if (errno == ENOENT) return;   // 没有同名共享内存，直接创建
        throw std::runtime_error(
            "IpcServer: 同名共享内存存在但无法打开: /dev/shm/" + name_.substr(1) +
            "（检查该文件的属主与读写权限）");
    }

    bool server_running = false;
    struct stat info;
    if (::fstat(probe, &info) == 0 && static_cast<size_t>(info.st_size) >= sizeof(SharedLayout)) {
        void* address = ::mmap(nullptr, sizeof(SharedLayout), PROT_READ, MAP_SHARED, probe, 0);
        if (address != MAP_FAILED) {
            const SharedLayout* existing = static_cast<const SharedLayout*>(address);
            const uint64_t magic = __atomic_load_n(&existing->magic, __ATOMIC_ACQUIRE);
            const int32_t pid = __atomic_load_n(&existing->server_pid, __ATOMIC_ACQUIRE);
            if (magic == kMagic && processAlive(pid)) {
                server_running = true;
            }
            ::munmap(address, sizeof(SharedLayout));
        }
    }
    ::close(probe);
    if (server_running) {
        throw std::runtime_error("IpcServer: 同名共享内存已有服务端在运行: " + name_);
    }
    if (::shm_unlink(name_.c_str()) != 0 && errno != ENOENT) {
        throw std::system_error(errno, std::generic_category(), "IpcServer: shm_unlink");
    }
}

void IpcServer::publishBytes(uint32_t channel, const void* data, size_t size) {
    if (layout_ == nullptr) throw std::logic_error("IpcServer::publish: 共享内存不可用");
    if (channel >= kChannelCount) throw std::out_of_range("IpcServer::publish: 通道编号越界");
    if (size > kMaxPayload) throw std::length_error("IpcServer::publish: 载荷超过上限");

    GetChannelSlot& slot = layout_->get[channel];
    detail::lockRobust(&slot.mutex);
    if (slot.active > 1u) slot.active = 0u;   // 共享内存被外部改写时恢复有效索引
    const uint32_t next = slot.active ^ 1u;
    std::memcpy(slot.payload[next], data, size);
    slot.size[next] = static_cast<uint32_t>(size);
    slot.active = next;
    detail::unlockRobust(&slot.mutex);
}

void IpcServer::setHandlerBytes(uint32_t channel,
                                std::function<bool(const void*, uint32_t)> handler) {
    if (request_thread_.joinable()) {
        throw std::logic_error("IpcServer::setHandler: 必须在 start() 之前注册回调");
    }
    if (channel >= kChannelCount) throw std::out_of_range("IpcServer::setHandler: 通道编号越界");
    handlers_[channel] = std::move(handler);
}

void IpcServer::start() {
    if (layout_ == nullptr) throw std::logic_error("IpcServer::start: 共享内存不可用");
    if (request_thread_.joinable()) {
        if (!request_failed_.load(std::memory_order_acquire)) {
            throw std::logic_error("IpcServer::start: 请求线程已在运行");
        }
        request_thread_.join();   // 线程已因故障退出，回收后允许重新启动
        request_failed_.store(false, std::memory_order_release);
    }
    stop_.store(false, std::memory_order_release);
    {
        RequestQueue& queue = layout_->queue;
        detail::lockRobust(&queue.mutex);
        if (stopped_once_) {   // 首次 start 保留 start 前排队的请求；停止后重启丢弃残留
            purgeQueueLocked();
            stopped_once_ = false;
        }
        // 在队列锁内切换为 kRunning：并发 send 在锁内复查状态，不会越过清空边界
        __atomic_store_n(&layout_->state, static_cast<uint32_t>(ServerState::kRunning), __ATOMIC_RELEASE);
        detail::unlockRobust(&queue.mutex);
    }
    try {
        request_failed_.store(false, std::memory_order_release);
        request_thread_ = std::thread([this] {
            try {
                requestLoop();
            } catch (...) {
                request_failed_.store(true, std::memory_order_release);
                __atomic_store_n(&layout_->state, static_cast<uint32_t>(ServerState::kStopped),
                                 __ATOMIC_RELEASE);
            }
        });
    } catch (...) {
        RequestQueue& queue = layout_->queue;
        detail::lockRobust(&queue.mutex);
        __atomic_store_n(&layout_->state, static_cast<uint32_t>(ServerState::kStopped), __ATOMIC_RELEASE);
        // 线程从未启动，启动前入队的请求仍由下一次首次 start() 按原语义处理
        detail::unlockRobust(&queue.mutex);
        throw;
    }
    started_once_ = true;   // 线程真正创建成功后才算运行过
}

void IpcServer::stop() {
    // 先在队列锁内置 kStopped，阻止新的 send 入队，再停请求线程
    if (layout_ != nullptr) {
        RequestQueue& queue = layout_->queue;
        detail::lockRobust(&queue.mutex);
        __atomic_store_n(&layout_->state, static_cast<uint32_t>(ServerState::kStopped), __ATOMIC_RELEASE);
        detail::unlockRobust(&queue.mutex);
    }
    if (request_thread_.joinable()) {
        stop_.store(true, std::memory_order_release);
        // 计数饱和时 sem_post 返回 EOVERFLOW；此时请求线程本来就不在阻塞等待，无需唤醒
        if (::sem_post(&layout_->queue.available) != 0 && errno != EOVERFLOW) {
            throw std::system_error(errno, std::generic_category(), "IpcServer::stop: sem_post");
        }
        request_thread_.join();
    }
    if (layout_ != nullptr) {
        RequestQueue& queue = layout_->queue;
        detail::lockRobust(&queue.mutex);
        // 仅在运行态到停止态的那一次清空获取通道；重复 stop() 不丢弃停止后 publish 的新内容
        const bool first_stop = started_once_ && !stopped_once_;
        if (started_once_) {
            purgeQueueLocked();
            stopped_once_ = true;
        }
        if (first_stop) for (uint32_t i = 0; i < kChannelCount; ++i) {
            GetChannelSlot& slot = layout_->get[i];
            detail::lockRobust(&slot.mutex);
            slot.active = 0;
            slot.size[0] = 0;
            slot.size[1] = 0;
            detail::unlockRobust(&slot.mutex);
        }
        detail::unlockRobust(&queue.mutex);
    }
}

void IpcServer::purgeQueueLocked() {
    RequestQueue& queue = layout_->queue;
    queue.purged += queue.count;
    queue.head = 0;
    queue.count = 0;
    for (;;) {
        errno = 0;
        if (::sem_trywait(&queue.available) == 0) continue;
        if (errno == EINTR) continue;
        break;   // EAGAIN 或信号量不可用：队列已清空，不能从析构路径抛出
    }
}

void IpcServer::requestLoop() {
    while (true) {
        while (::sem_wait(&layout_->queue.available) != 0) {
            if (errno == EINTR) continue;
            request_failed_.store(true, std::memory_order_release);
            __atomic_store_n(&layout_->state, static_cast<uint32_t>(ServerState::kStopped),
                             __ATOMIC_RELEASE);
            return;   // 信号量损坏时退出请求线程，不抛异常触发 std::terminate
        }
        if (stop_.load(std::memory_order_acquire)) break;

        // 一次唤醒后把队列里的请求全部处理完，而不是一次只处理一个
        bool first_request = true;
        while (!stop_.load(std::memory_order_acquire)) {
            SendRequest request;
            bool has_request = false;
            {
                RequestQueue& queue = layout_->queue;
                detail::lockRobust(&queue.mutex);
                if (queue.head >= kQueueCapacity || queue.count > kQueueCapacity) {
                    // robust 互斥量恢复后元数据可能不一致：丢弃队列，避免越界读取
                    purgeQueueLocked();
                }
                if (queue.count > 0) {
                    const uint32_t index = queue.head;
                    request = queue.requests[index];
                    queue.head = (queue.head + 1) % kQueueCapacity;
                    --queue.count;
                    has_request = true;
                }
                detail::unlockRobust(&queue.mutex);
            }
            if (!has_request) break;

            // 外层 sem_wait 消耗了第一条请求的通知；其余请求各消耗一个对应通知，
            // 避免批量清空时通知计数无限累积
            if (!first_request) {
                while (::sem_trywait(&layout_->queue.available) != 0 && errno == EINTR) {
                }
            }
            first_request = false;

            const uint32_t channel = request.channel;
            bool invoked = false;
            if (channel < kChannelCount && handlers_[channel]) {
                try {
                    invoked = handlers_[channel](request.payload, request.size);
                } catch (...) {
                    invoked = false;   // 回调异常计入 rejected，不让请求线程终止
                }
            }
            if (invoked) {
                handled_.fetch_add(1, std::memory_order_relaxed);
            } else {
                rejected_.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
}

IpcServer::Stats IpcServer::stats() const {
    Stats result;
    if (layout_ == nullptr) return result;
    RequestQueue& queue = layout_->queue;
    detail::lockRobust(&queue.mutex);
    result.enqueued = queue.enqueued;
    result.dropped = queue.dropped;
    result.purged = queue.purged;
    detail::unlockRobust(&queue.mutex);
    result.handled = handled_.load(std::memory_order_relaxed);
    result.rejected = rejected_.load(std::memory_order_relaxed);
    return result;
}

}  // namespace ipc
}  // namespace s2c
