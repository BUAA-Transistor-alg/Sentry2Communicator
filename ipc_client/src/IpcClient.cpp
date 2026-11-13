#include "ipc_client/IpcClient.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace s2c {
namespace ipc {

namespace {

void checkChannel(uint32_t channel, const char* method) {
    if (channel >= kChannelCount) {
        throw std::out_of_range(std::string("IpcClient::") + method + ": 通道编号越界");
    }
}

}  // namespace

IpcClient::IpcClient(const std::string& name) : name_(name) {
    if (name_.empty() || name_.front() != '/') {
        throw std::invalid_argument("IpcClient: 共享内存名字必须以 / 开头: " + name_);
    }
}

IpcClient::~IpcClient() { detach(); }

bool IpcClient::getBytes(uint32_t channel, void* out, size_t expected) {
    checkChannel(channel, "get");
    if (!ensureAttached()) return false;

    GetChannelSlot& slot = layout_->get[channel];
    detail::lockRobust(&slot.mutex);
    const uint32_t active = slot.active;
    const uint32_t size = slot.size[active];
    const bool matched = size == expected;
    if (matched) std::memcpy(out, slot.payload[active], size);
    detail::unlockRobust(&slot.mutex);
    return matched;
}

bool IpcClient::sendBytes(uint32_t channel, const void* data, size_t size) {
    checkChannel(channel, "send");
    if (size > kMaxPayload) throw std::length_error("IpcClient::send: 载荷超过上限");
    if (!ensureAttached()) return false;

    RequestQueue& queue = layout_->queue;
    detail::lockRobust(&queue.mutex);
    if (queue.count == kQueueCapacity) {
        // 队列满时覆盖最旧的一条请求，客户端不会因为服务端不消费而阻塞
        queue.head = (queue.head + 1) % kQueueCapacity;
        --queue.count;
        ++queue.dropped;
    }
    const uint32_t tail = (queue.head + queue.count) % kQueueCapacity;
    queue.requests[tail].channel = channel;
    queue.requests[tail].size = static_cast<uint32_t>(size);
    std::memcpy(queue.requests[tail].payload, data, size);
    ++queue.count;
    ++queue.enqueued;
    detail::unlockRobust(&queue.mutex);

    if (::sem_post(&queue.available) != 0 && errno != EOVERFLOW) {
        throw std::system_error(errno, std::generic_category(), "IpcClient::send: sem_post");
    }
    return true;
}

bool IpcClient::ensureAttached() {
    if (layout_ != nullptr && serverRunning()) return true;
    detach();
    return attach();
}

bool IpcClient::attach() {
    const int fd = ::shm_open(name_.c_str(), O_RDWR, 0);
    if (fd < 0) return false;

    struct stat info;
    if (::fstat(fd, &info) != 0 || static_cast<size_t>(info.st_size) < sizeof(SharedLayout)) {
        ::close(fd);
        return false;
    }
    void* address = ::mmap(nullptr, sizeof(SharedLayout), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (address == MAP_FAILED) return false;

    SharedLayout* layout = static_cast<SharedLayout*>(address);
    const uint64_t magic = __atomic_load_n(&layout->magic, __ATOMIC_ACQUIRE);
    if (magic != kMagic || layout->version != kVersion) {
        ::munmap(address, sizeof(SharedLayout));
        return false;
    }
    layout_ = layout;
    if (!serverRunning()) {
        detach();
        return false;
    }
    return true;
}

void IpcClient::detach() {
    if (layout_ != nullptr) {
        ::munmap(layout_, sizeof(SharedLayout));
        layout_ = nullptr;
    }
}

bool IpcClient::serverRunning() const {
    if (layout_ == nullptr) return false;
    const uint32_t state = __atomic_load_n(&layout_->state, __ATOMIC_ACQUIRE);
    if (state != static_cast<uint32_t>(ServerState::kRunning)) return false;
    const int32_t pid = layout_->server_pid;
    if (pid <= 0) return false;
    if (::kill(pid, 0) == 0) return true;
    return errno != ESRCH;   // 无权限访问的进程同样视为存活
}

}  // namespace ipc
}  // namespace s2c
