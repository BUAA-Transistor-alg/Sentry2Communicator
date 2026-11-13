#include "ipc/IpcServer.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
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
    if (::kill(pid, 0) == 0) return true;
    return errno != ESRCH;
}

}  // namespace

IpcServer::IpcServer(const std::string& name) : name_(checkName(name)) {
    removeStaleSegment();

    shm_fd_ = ::shm_open(name_.c_str(), O_CREAT | O_EXCL | O_RDWR, kSharedMemoryMode);
    if (shm_fd_ < 0) {
        throw std::system_error(errno, std::generic_category(), "IpcServer: shm_open");
    }
    if (::ftruncate(shm_fd_, static_cast<off_t>(sizeof(SharedLayout))) != 0) {
        const int error = errno;
        ::close(shm_fd_);
        shm_fd_ = -1;
        ::shm_unlink(name_.c_str());
        throw std::system_error(error, std::generic_category(), "IpcServer: ftruncate");
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
    std::memset(layout_, 0, sizeof(SharedLayout));
    for (uint32_t i = 0; i < kChannelCount; ++i) {
        detail::initializeRobustMutex(&layout_->get[i].mutex);
        layout_->get[i].active = 0;
    }
    detail::initializeRobustMutex(&layout_->queue.mutex);
    if (::sem_init(&layout_->queue.available, 1, 0) != 0) {
        const int error = errno;
        ::munmap(layout_, sizeof(SharedLayout));
        layout_ = nullptr;
        ::close(shm_fd_);
        shm_fd_ = -1;
        ::shm_unlink(name_.c_str());
        throw std::system_error(error, std::generic_category(), "IpcServer: sem_init");
    }

    layout_->version = kVersion;
    layout_->server_pid = static_cast<int32_t>(::getpid());
    __atomic_store_n(&layout_->magic, kMagic, __ATOMIC_RELEASE);
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
    if (probe < 0) return;   // 没有同名共享内存，直接创建

    bool server_running = false;
    struct stat info;
    if (::fstat(probe, &info) == 0 && static_cast<size_t>(info.st_size) >= sizeof(SharedLayout)) {
        void* address = ::mmap(nullptr, sizeof(SharedLayout), PROT_READ, MAP_SHARED, probe, 0);
        if (address != MAP_FAILED) {
            const SharedLayout* existing = static_cast<const SharedLayout*>(address);
            if (existing->magic == kMagic && processAlive(existing->server_pid)) {
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
    const uint32_t next = slot.active ^ 1u;
    std::memcpy(slot.payload[next], data, size);
    slot.size[next] = static_cast<uint32_t>(size);
    slot.active = next;
    detail::unlockRobust(&slot.mutex);
}

void IpcServer::setHandlerBytes(uint32_t channel,
                                std::function<void(const void*, uint32_t)> handler) {
    if (request_thread_.joinable()) {
        throw std::logic_error("IpcServer::setHandler: 必须在 start() 之前注册回调");
    }
    if (channel >= kChannelCount) throw std::out_of_range("IpcServer::setHandler: 通道编号越界");
    handlers_[channel] = std::move(handler);
}

void IpcServer::start() {
    if (layout_ == nullptr) throw std::logic_error("IpcServer::start: 共享内存不可用");
    if (request_thread_.joinable()) throw std::logic_error("IpcServer::start: 请求线程已在运行");
    stop_.store(false, std::memory_order_release);
    request_thread_ = std::thread([this] { requestLoop(); });
}

void IpcServer::stop() {
    if (!request_thread_.joinable()) return;
    stop_.store(true, std::memory_order_release);
    if (::sem_post(&layout_->queue.available) != 0) {
        throw std::system_error(errno, std::generic_category(), "IpcServer::stop: sem_post");
    }
    request_thread_.join();
}

void IpcServer::requestLoop() {
    while (true) {
        while (::sem_wait(&layout_->queue.available) != 0) {
            if (errno == EINTR) continue;
            throw std::system_error(errno, std::generic_category(), "IpcServer: sem_wait");
        }
        if (stop_.load(std::memory_order_acquire)) break;

        SendRequest request;
        bool has_request = false;
        {
            RequestQueue& queue = layout_->queue;
            detail::lockRobust(&queue.mutex);
            if (queue.count > 0) {
                const uint32_t index = queue.head;
                request = queue.requests[index];
                queue.head = (queue.head + 1) % kQueueCapacity;
                --queue.count;
                has_request = true;
            }
            detail::unlockRobust(&queue.mutex);
        }
        if (!has_request) continue;

        const uint32_t channel = request.channel;
        if (channel < kChannelCount && handlers_[channel]) {
            handlers_[channel](request.payload, request.size);
        }
        handled_.fetch_add(1, std::memory_order_relaxed);
    }
}

IpcServer::Stats IpcServer::stats() const {
    Stats result;
    if (layout_ == nullptr) return result;
    RequestQueue& queue = layout_->queue;
    detail::lockRobust(&queue.mutex);
    result.enqueued = queue.enqueued;
    result.dropped = queue.dropped;
    detail::unlockRobust(&queue.mutex);
    result.handled = handled_.load(std::memory_order_relaxed);
    return result;
}

}  // namespace ipc
}  // namespace s2c
