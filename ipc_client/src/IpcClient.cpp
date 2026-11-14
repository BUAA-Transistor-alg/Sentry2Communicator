#include "ipc_client/IpcClient.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace s2c {
namespace ipc {

namespace {

bool processIsZombie(pid_t pid) {
    char path[64];
    std::snprintf(path, sizeof(path), "/proc/%d/stat", static_cast<int>(pid));
    std::FILE* file = std::fopen(path, "r");
    if (file == nullptr) return false;
    char buffer[512];
    const bool read = std::fgets(buffer, sizeof(buffer), file) != nullptr;
    std::fclose(file);
    if (!read) return false;
    const char* end = std::strrchr(buffer, ')');
    if (end == nullptr || end[1] != ' ') return false;
    const size_t index = static_cast<size_t>(end - buffer);
    if (index + 2 >= sizeof(buffer)) return false;
    return buffer[index + 2] == 'Z';
}

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

    try {
        GetChannelSlot& slot = layout_->get[channel];
        detail::lockRobust(&slot.mutex);
        const uint32_t active = slot.active;
        if (active > 1u) {   // 共享内存被外部改写：拒绝使用损坏的索引
            detail::unlockRobust(&slot.mutex);
            return false;
        }
        const uint32_t size = slot.size[active];
        const bool matched = size == expected;
        if (matched) std::memcpy(out, slot.payload[active], size);
        detail::unlockRobust(&slot.mutex);
        return matched;
    } catch (const std::system_error&) {
        return false;   // robust 互斥量不可恢复等极端错误按调用失败处理
    }
}

bool IpcClient::sendBytes(uint32_t channel, const void* data, size_t size) {
    checkChannel(channel, "send");
    if (size > kMaxPayload) throw std::length_error("IpcClient::send: 载荷超过上限");
    if (!ensureAttached()) return false;

    try {
        RequestQueue& queue = layout_->queue;
        detail::lockRobust(&queue.mutex);
        // 锁内复查服务端状态与进程存活：缩小"附加成功后服务端死亡"的误报成功窗口
        if (!serverRunning()) {
            detail::unlockRobust(&queue.mutex);
            return false;
        }
        if (queue.head >= kQueueCapacity || queue.count > kQueueCapacity) {
            // 持有者死亡后元数据可能不一致：恢复为空队列，计入与服务端一致的 purged 统计
            queue.purged += static_cast<uint64_t>(
                queue.count <= kQueueCapacity ? queue.count : kQueueCapacity);
            queue.head = 0;
            queue.count = 0;
            // 队列已清空，残留通知一并消费，避免服务端后续空唤醒
            for (;;) {
                if (::sem_trywait(&queue.available) == 0) continue;
                if (errno == EINTR) continue;
                break;
            }
        }
        // 每个新增槽位对应一个通知；覆盖旧请求不新增槽位，不再额外 post
        const bool notify = (queue.count < kQueueCapacity);
        if (notify) {
            // 先通知再写入队列：客户端在入队中途死亡时只会多一次空唤醒，
            // 不会留下"队列非空但没有任何唤醒"的停滞状态
            if (::sem_post(&queue.available) != 0 && errno != EOVERFLOW) {
                const int error = errno;
                detail::unlockRobust(&queue.mutex);
                throw std::system_error(error, std::generic_category(), "IpcClient::send: sem_post");
            }
        }
        // 队列满时先推进 head 并减少 count，把最旧槽位从队列中摘除，再向该空闲槽写入新请求。
        // 客户端在写入中途死亡时服务端不会读到半写槽位（该槽位已不在有效队列范围内）
        if (queue.count == kQueueCapacity) {
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
        return true;
    } catch (const std::system_error&) {
        return false;   // robust 互斥量不可恢复等极端错误按调用失败处理
    }
}

bool IpcClient::ensureAttached() {
    if (layout_ != nullptr) {
        struct stat info;
        const std::string shm_path = "/dev/shm" + name_;
        if (::stat(shm_path.c_str(), &info) == 0) {
            if (static_cast<uint64_t>(info.st_dev) == shm_dev_ &&
                static_cast<uint64_t>(info.st_ino) == shm_ino_) {
                return serverRunning();   // 同一段共享内存：只复查状态，不重复 mmap
            }
        } else {
            return false;   // 共享内存已删除，等新文件出现后再重新附加
        }
        detach();
    }
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
    const uint32_t version = __atomic_load_n(&layout->version, __ATOMIC_ACQUIRE);
    if (magic != kMagic || version != kVersion) {
        ::munmap(address, sizeof(SharedLayout));
        return false;
    }
    layout_ = layout;
    shm_dev_ = static_cast<uint64_t>(info.st_dev);
    shm_ino_ = static_cast<uint64_t>(info.st_ino);
    // 服务端已停止但未析构时保留映射，后续调用只复查状态，避免每次重新 mmap
    return serverRunning();
}

void IpcClient::detach() {
    if (layout_ != nullptr) {
        ::munmap(layout_, sizeof(SharedLayout));
        layout_ = nullptr;
        shm_dev_ = 0;
        shm_ino_ = 0;
    }
}

bool IpcClient::serverRunning() const {
    if (layout_ == nullptr) return false;
    const uint32_t state = __atomic_load_n(&layout_->state, __ATOMIC_ACQUIRE);
    if (state != static_cast<uint32_t>(ServerState::kRunning)) return false;
    const int32_t pid = __atomic_load_n(&layout_->server_pid, __ATOMIC_ACQUIRE);
    if (pid <= 0) return false;
    if (::kill(pid, 0) == 0) return !processIsZombie(pid);
    return errno != ESRCH;   // 无权限访问的进程同样视为存活
}

}  // namespace ipc
}  // namespace s2c
