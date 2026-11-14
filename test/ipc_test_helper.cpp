// test/ipc_test_helper.cpp — IPC 测试用的子进程
//
// 模式:
//   client-loop <name> <client_id> <count>   并发客户端: 循环获取状态并发送命令
//   lock-and-die <name> <channel>            锁住获取通道后退出，制造持有者死亡
//   lock-queue-and-die <name>                锁住请求队列后退出，制造持有者死亡
//   enqueue-request <name> <channel>         直接向请求队列塞一条命令（绕过客户端状态检查）
//   corrupt-active <name> <channel>          把获取通道的 active 索引改成非法值
//   server-host <name>                       独立进程中的服务端，由测试进程杀死
#include "ipc/IpcServer.hpp"
#include "ipc_client/IpcClient.hpp"
#include "ipc_test_types.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>

namespace {

s2c::ipc::SharedLayout* attachLayout(const char* name) {
    const int fd = ::shm_open(name, O_RDWR, 0);
    if (fd < 0) return nullptr;
    struct stat info;
    if (::fstat(fd, &info) != 0 ||
        static_cast<size_t>(info.st_size) < sizeof(s2c::ipc::SharedLayout)) {
        ::close(fd);
        return nullptr;
    }
    void* address = ::mmap(nullptr, sizeof(s2c::ipc::SharedLayout),
                           PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (address == MAP_FAILED) return nullptr;
    return static_cast<s2c::ipc::SharedLayout*>(address);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    const std::string mode = argv[1];

    if (mode == "client-loop") {
        if (argc < 5) return 2;
        const std::string name = argv[2];
        const uint32_t client_id = static_cast<uint32_t>(std::stoul(argv[3]));
        const uint32_t count = static_cast<uint32_t>(std::stoul(argv[4]));
        s2c::ipc::IpcClient client(name);
        for (uint32_t i = 0; i < count; ++i) {
            ipc_test::StatusData status{};
            if (!client.get(ipc_test::kStatusChannel, status)) return 3;
            if (status.counter != 1) return 4;
            const ipc_test::CommandData command{client_id, static_cast<int32_t>(i)};
            if (!client.send(ipc_test::kCommandChannel, command)) return 5;
        }
        return 0;
    }

    if (mode == "lock-and-die") {
        if (argc < 4) return 2;
        s2c::ipc::SharedLayout* layout = attachLayout(argv[2]);
        if (layout == nullptr) return 6;
        const uint32_t channel = static_cast<uint32_t>(std::stoul(argv[3]));
        s2c::ipc::GetChannelSlot& slot = layout->get[channel];
        s2c::ipc::detail::lockRobust(&slot.mutex);
        // 破坏非活动缓冲区后退出：双缓冲应保证读取方仍拿到上一份完整数据
        const uint32_t inactive = slot.active ^ 1u;
        std::memset(slot.payload[inactive], 0xFF, sizeof(slot.payload[inactive]));
        slot.size[inactive] = static_cast<uint32_t>(sizeof(slot.payload[inactive]));
        ::_exit(0);   // 不释放锁: 内核把该互斥量标记为持有者死亡
    }

    if (mode == "lock-queue-and-die") {
        if (argc < 3) return 2;
        s2c::ipc::SharedLayout* layout = attachLayout(argv[2]);
        if (layout == nullptr) return 6;
        s2c::ipc::detail::lockRobust(&layout->queue.mutex);
        ::_exit(0);
    }

    if (mode == "enqueue-request") {
        if (argc < 4) return 2;
        s2c::ipc::SharedLayout* layout = attachLayout(argv[2]);
        if (layout == nullptr) return 6;
        const uint32_t channel = static_cast<uint32_t>(std::stoul(argv[3]));
        s2c::ipc::SendRequest request{};
        request.channel = channel;
        request.size = sizeof(ipc_test::CommandData);
        const ipc_test::CommandData command{99, 99};
        std::memcpy(request.payload, &command, sizeof(command));
        s2c::ipc::RequestQueue& queue = layout->queue;
        s2c::ipc::detail::lockRobust(&queue.mutex);
        if (queue.count < s2c::ipc::kQueueCapacity) {
            const uint32_t index = (queue.head + queue.count) % s2c::ipc::kQueueCapacity;
            queue.requests[index] = request;
            ++queue.count;
        }
        s2c::ipc::detail::unlockRobust(&queue.mutex);
        if (::sem_post(&queue.available) != 0) return 7;
        ::_exit(0);
    }

    if (mode == "corrupt-active") {
        if (argc < 4) return 2;
        s2c::ipc::SharedLayout* layout = attachLayout(argv[2]);
        if (layout == nullptr) return 6;
        const uint32_t channel = static_cast<uint32_t>(std::stoul(argv[3]));
        s2c::ipc::GetChannelSlot& slot = layout->get[channel];
        s2c::ipc::detail::lockRobust(&slot.mutex);
        slot.active = 3;   // 非法索引，验证读取方与服务端发布方的边界检查
        s2c::ipc::detail::unlockRobust(&slot.mutex);
        ::_exit(0);
    }

    if (mode == "server-host") {
        if (argc < 3) return 2;
        s2c::ipc::IpcServer server(argv[2]);
        server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{1, 2.5f});
        server.start();
        for (;;) ::pause();   // 由测试进程杀死
    }

    return 2;
}
