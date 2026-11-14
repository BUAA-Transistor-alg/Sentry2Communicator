// test_ipc.cpp — IPC 端到端测试
//
// 覆盖:
//   · 多组获取/发送通道（同一服务端，两组不同类型的内容）
//   · 多个客户端进程并发发送，服务端逐个回调
//   · 请求队列满时覆盖最旧请求，客户端不入队阻塞
//   · 持有者死亡（robust 互斥量）后客户端调用立即恢复
//   · 服务端停止后客户端调用在上限内返回 false，服务端重启后自动重新附加
//   · 服务端进程被杀死后客户端调用在上限内返回 false
//
// 客户端代码来自独立工程 ipc_client/，与服务端一起编译进本测试。
#include "ipc/IpcServer.hpp"
#include "ipc_client/IpcClient.hpp"
#include "ipc_test_types.hpp"
#include "test_support.hpp"

#include <signal.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using namespace s2c;
using s2c_test::waitFor;

// 客户端调用到返回的耗时上限（测试断言用；实现里没有任何超时参数）
constexpr long kMaxCallMicros = 200000;

std::string makeName(const char* tag) {
    return "/s2c_test_ipc_" + std::to_string(::getpid()) + "_" + tag;
}

// 先构造 argv 再 fork，子进程只调用 execv/_exit（不进行内存分配）
pid_t spawnHelper(std::vector<std::string> args) {
    std::vector<char*> argv;
    argv.reserve(args.size() + 2);
    argv.push_back(const_cast<char*>(S2C_IPC_HELPER_BIN));
    for (std::string& argument : args) argv.push_back(argument.data());
    argv.push_back(nullptr);

    const pid_t pid = ::fork();
    if (pid == 0) {
        ::execv(S2C_IPC_HELPER_BIN, argv.data());
        ::_exit(127);
    }
    S2C_CHECK(pid > 0);
    return pid;
}

int waitHelper(pid_t pid) {
    int status = 0;
    S2C_CHECK(::waitpid(pid, &status, 0) == pid);
    if (!WIFEXITED(status)) return -1;
    return WEXITSTATUS(status);
}

template <typename Callable>
long measureMicros(Callable&& callable) {
    const auto start = std::chrono::steady_clock::now();
    callable();
    return static_cast<long>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start).count());
}

void testGetAndSend() {
    std::printf("[test] 获取与发送\n");
    const std::string name = makeName("basic");
    ipc::IpcServer server(name);
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{7, 1.5f});
    server.publish(ipc_test::kAltStatusChannel, ipc_test::AltStatus{0x1122334455667788ULL});

    // 两组发送通道分别回调（必须在 start() 之前注册）
    std::mutex mutex;
    std::vector<ipc_test::CommandData> commands;
    std::vector<ipc_test::AltCommand> alt_commands;
    server.setHandler<ipc_test::CommandData>(
        ipc_test::kCommandChannel, [&](const ipc_test::CommandData& command) {
            std::lock_guard<std::mutex> lock(mutex);
            commands.push_back(command);
        });
    server.setHandler<ipc_test::AltCommand>(
        ipc_test::kAltCommandChannel, [&](const ipc_test::AltCommand& command) {
            std::lock_guard<std::mutex> lock(mutex);
            alt_commands.push_back(command);
        });
    server.start();

    ipc::IpcClient client(name);

    ipc_test::StatusData status{};
    S2C_CHECK(client.get(ipc_test::kStatusChannel, status));
    S2C_CHECK(status.counter == 7);
    S2C_CHECK(status.value == 1.5f);

    ipc_test::AltStatus alt{};
    S2C_CHECK(client.get(ipc_test::kAltStatusChannel, alt));
    S2C_CHECK(alt.code == 0x1122334455667788ULL);

    // 类型长度不一致时返回 false
    ipc_test::ShortStatus short_status{};
    S2C_CHECK(!client.get(ipc_test::kAltStatusChannel, short_status));

    // 服务端更新内容后客户端读到新值
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{8, 2.5f});
    S2C_CHECK(client.get(ipc_test::kStatusChannel, status));
    S2C_CHECK(status.counter == 8);

    S2C_CHECK(client.send(ipc_test::kCommandChannel, ipc_test::CommandData{3, -7}));
    S2C_CHECK(client.send(ipc_test::kAltCommandChannel, ipc_test::AltCommand{0.25}));
    S2C_CHECK(waitFor([&] {
        std::lock_guard<std::mutex> lock(mutex);
        return commands.size() == 1 && alt_commands.size() == 1;
    }, std::chrono::milliseconds(2000)));

    {
        std::lock_guard<std::mutex> lock(mutex);
        S2C_CHECK(commands[0].client_id == 3);
        S2C_CHECK(commands[0].index == -7);
        S2C_CHECK(alt_commands[0].gain == 0.25);
    }

    // 尺寸与注册类型不匹配的请求被拒绝，计入 rejected 而不是 handled
    S2C_CHECK(client.send(ipc_test::kCommandChannel, ipc_test::ShortStatus{1}));
    S2C_CHECK(waitFor([&] { return server.stats().rejected >= 1; },
                      std::chrono::milliseconds(2000)));
    server.stop();
}

void testConcurrentClients() {
    std::printf("[test] 多个客户端进程并发\n");
    constexpr uint32_t kClients = 8;
    constexpr uint32_t kMessages = 10;

    const std::string name = makeName("concurrent");
    ipc::IpcServer server(name);
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{1, 0.5f});

    std::mutex mutex;
    std::vector<ipc_test::CommandData> commands;
    server.setHandler<ipc_test::CommandData>(
        ipc_test::kCommandChannel, [&](const ipc_test::CommandData& command) {
            std::lock_guard<std::mutex> lock(mutex);
            commands.push_back(command);
        });
    server.start();

    std::vector<pid_t> children;
    for (uint32_t i = 0; i < kClients; ++i) {
        children.push_back(spawnHelper(
            {"client-loop", name, std::to_string(i), std::to_string(kMessages)}));
    }
    for (const pid_t pid : children) S2C_CHECK(waitHelper(pid) == 0);

    S2C_CHECK(waitFor([&] {
        std::lock_guard<std::mutex> lock(mutex);
        return commands.size() == kClients * kMessages;
    }, std::chrono::milliseconds(2000)));

    std::set<std::pair<uint32_t, int32_t>> seen;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (const ipc_test::CommandData& command : commands) {
            seen.insert({command.client_id, command.index});
        }
    }
    S2C_CHECK(seen.size() == kClients * kMessages);

    const ipc::IpcServer::Stats stats = server.stats();
    S2C_CHECK(stats.enqueued == kClients * kMessages);
    S2C_CHECK(stats.dropped == 0);
    S2C_CHECK(waitFor([&] { return server.stats().handled == kClients * kMessages; },
                      std::chrono::milliseconds(2000)));
    server.stop();
}

void testQueueOverwrite() {
    std::printf("[test] 队列满时覆盖最旧请求\n");
    const std::string name = makeName("overflow");
    ipc::IpcServer server(name);
    ipc::IpcClient client(name);

    constexpr uint32_t kExtra = 5;
    for (uint32_t i = 0; i < ipc::kQueueCapacity + kExtra; ++i) {
        const ipc_test::CommandData command{1, static_cast<int32_t>(i)};
        S2C_CHECK(client.send(ipc_test::kCommandChannel, command));
    }

    std::mutex mutex;
    std::vector<ipc_test::CommandData> commands;
    server.setHandler<ipc_test::CommandData>(
        ipc_test::kCommandChannel, [&](const ipc_test::CommandData& command) {
            std::lock_guard<std::mutex> lock(mutex);
            commands.push_back(command);
        });
    server.start();
    S2C_CHECK(waitFor([&] {
        std::lock_guard<std::mutex> lock(mutex);
        return commands.size() == ipc::kQueueCapacity;
    }, std::chrono::milliseconds(2000)));

    std::set<int32_t> indexes;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (const ipc_test::CommandData& command : commands) indexes.insert(command.index);
    }
    S2C_CHECK(indexes.size() == ipc::kQueueCapacity);
    S2C_CHECK(indexes.count(0) == 0);
    S2C_CHECK(indexes.count(kExtra - 1) == 0);
    S2C_CHECK(indexes.count(kExtra) == 1);
    S2C_CHECK(indexes.count(ipc::kQueueCapacity + kExtra - 1) == 1);
    S2C_CHECK(server.stats().dropped == kExtra);
    server.stop();
}

void testDeadOwnerRecovery() {
    std::printf("[test] 持有者死亡后恢复\n");
    const std::string name = makeName("recovery");
    ipc::IpcServer server(name);
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{5, 0.5f});
    ipc::IpcClient client(name);

    // 子进程锁住获取通道后退出，模拟持有者死亡
    S2C_CHECK(waitHelper(spawnHelper(
        {"lock-and-die", name, std::to_string(ipc_test::kStatusChannel)})) == 0);

    bool got = false;
    ipc_test::StatusData status{};
    const long get_micros = measureMicros([&] {
        got = client.get(ipc_test::kStatusChannel, status);
    });
    S2C_CHECK(got);
    S2C_CHECK(status.counter == 5);   // 双缓冲保证崩溃不会留下半份数据
    S2C_CHECK(get_micros < kMaxCallMicros);

    // 请求队列的持有者死亡后发送仍然成功
    std::mutex mutex;
    std::vector<ipc_test::CommandData> commands;
    server.setHandler<ipc_test::CommandData>(
        ipc_test::kCommandChannel, [&](const ipc_test::CommandData& command) {
            std::lock_guard<std::mutex> lock(mutex);
            commands.push_back(command);
        });
    server.start();
    S2C_CHECK(waitHelper(spawnHelper({"lock-queue-and-die", name})) == 0);

    bool sent = false;
    const long send_micros = measureMicros([&] {
        sent = client.send(ipc_test::kCommandChannel, ipc_test::CommandData{9, 9});
    });
    S2C_CHECK(sent);
    S2C_CHECK(send_micros < kMaxCallMicros);
    S2C_CHECK(waitFor([&] {
        std::lock_guard<std::mutex> lock(mutex);
        return commands.size() == 1 && commands[0].index == 9;
    }, std::chrono::milliseconds(2000)));
    server.stop();
}

void testServerStopAndRestart() {
    std::printf("[test] 服务端停止与重启\n");
    const std::string name = makeName("restart");
    std::unique_ptr<ipc::IpcServer> server = std::make_unique<ipc::IpcServer>(name);
    server->publish(ipc_test::kStatusChannel, ipc_test::StatusData{3, 0.25f});

    ipc::IpcClient client(name);
    ipc_test::StatusData status{};
    S2C_CHECK(client.get(ipc_test::kStatusChannel, status));
    S2C_CHECK(status.counter == 3);

    server.reset();   // 服务端失效
    bool got = true;
    const long get_micros = measureMicros([&] {
        got = client.get(ipc_test::kStatusChannel, status);
    });
    S2C_CHECK(!got);
    S2C_CHECK(get_micros < kMaxCallMicros);
    bool sent = true;
    const long send_micros = measureMicros([&] {
        sent = client.send(ipc_test::kCommandChannel, ipc_test::CommandData{2, 2});
    });
    S2C_CHECK(!sent);
    S2C_CHECK(send_micros < kMaxCallMicros);

    // 新服务端启动后客户端自动重新附加
    ipc::IpcServer restarted(name);
    restarted.publish(ipc_test::kStatusChannel, ipc_test::StatusData{4, 0.5f});
    S2C_CHECK(waitFor([&] {
        return client.get(ipc_test::kStatusChannel, status) && status.counter == 4;
    }, std::chrono::milliseconds(2000)));
}

void testKilledServerProcess() {
    std::printf("[test] 服务端进程被杀死\n");
    const std::string name = makeName("killed");
    const pid_t server_pid = spawnHelper({"server-host", name});

    ipc::IpcClient client(name);
    ipc_test::StatusData status{};
    S2C_CHECK(waitFor([&] { return client.get(ipc_test::kStatusChannel, status); },
                      std::chrono::milliseconds(2000)));
    S2C_CHECK(status.counter == 1);

    S2C_CHECK(::kill(server_pid, SIGKILL) == 0);
    int child_status = 0;
    S2C_CHECK(::waitpid(server_pid, &child_status, 0) == server_pid);

    bool got = true;
    const long get_micros = measureMicros([&] {
        got = client.get(ipc_test::kStatusChannel, status);
    });
    S2C_CHECK(!got);
    S2C_CHECK(get_micros < kMaxCallMicros);

    bool sent = true;
    const long send_micros = measureMicros([&] {
        sent = client.send(ipc_test::kCommandChannel, ipc_test::CommandData{4, 4});
    });
    S2C_CHECK(!sent);
    S2C_CHECK(send_micros < kMaxCallMicros);

    // 清理被杀死服务端遗留的共享内存
    S2C_CHECK(::shm_unlink(name.c_str()) == 0);
}

void testStopWithoutDestruction() {
    std::printf("[test] stop 后（不析构）客户端判定不可用\n");
    const std::string name = makeName("stop");
    ipc::IpcServer server(name);
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{6, 0.5f});
    server.start();

    ipc::IpcClient client(name);
    ipc_test::StatusData data{};
    S2C_CHECK(client.get(ipc_test::kStatusChannel, data));
    S2C_CHECK(data.counter == 6);

    server.stop();   // 不析构，客户端应立即判定服务端不可用
    S2C_CHECK(waitFor([&] { return !client.get(ipc_test::kStatusChannel, data); },
                      std::chrono::milliseconds(2000)));
    S2C_CHECK(!client.send(ipc_test::kCommandChannel, ipc_test::CommandData{1, 1}));

    // 重新 start 后客户端可以再次附加
    server.start();
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{7, 0.5f});
    S2C_CHECK(waitFor([&] {
        return client.get(ipc_test::kStatusChannel, data) && data.counter == 7;
    }, std::chrono::milliseconds(2000)));
    server.stop();
}

void testStopClearsStaleRequests() {
    std::printf("[test] stop 清空残留请求\n");
    const std::string name = makeName("stale");
    ipc::IpcServer server(name);
    std::atomic<int> calls{0};
    server.setHandler<ipc_test::CommandData>(
        ipc_test::kCommandChannel,
        [&](const ipc_test::CommandData&) { calls.fetch_add(1, std::memory_order_relaxed); });
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{1, 0.5f});
    server.start();
    server.stop();

    // 绕过客户端状态检查，直接向队列塞一条请求，模拟停止窗口内的残留
    const pid_t pid = spawnHelper({"enqueue-request", name,
                                   std::to_string(ipc_test::kCommandChannel)});
    S2C_CHECK(waitHelper(pid) == 0);

    server.start();   // start 应清掉残留请求
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    S2C_CHECK(calls.load(std::memory_order_relaxed) == 0);
    server.stop();
}

void testCorruptActiveIndex() {
    std::printf("[test] 共享内存 active 索引被改写\n");
    const std::string name = makeName("corrupt");
    ipc::IpcServer server(name);
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{42, 0.5f});
    server.start();
    ipc::IpcClient client(name);
    ipc_test::StatusData data{};
    S2C_CHECK(client.get(ipc_test::kStatusChannel, data));
    S2C_CHECK(data.counter == 42);

    const pid_t pid = spawnHelper({"corrupt-active", name,
                                   std::to_string(ipc_test::kStatusChannel)});
    S2C_CHECK(waitHelper(pid) == 0);
    S2C_CHECK(!client.get(ipc_test::kStatusChannel, data));

    // 服务端发布时恢复非法索引，客户端重新读到新值
    server.publish(ipc_test::kStatusChannel, ipc_test::StatusData{43, 0.5f});
    S2C_CHECK(waitFor([&] {
        return client.get(ipc_test::kStatusChannel, data) && data.counter == 43;
    }, std::chrono::milliseconds(2000)));
    server.stop();
}

void testNoServer() {
    std::printf("[test] 服务端不存在时立即返回\n");
    ipc::IpcClient client(makeName("absent"));
    ipc_test::StatusData status{};
    bool got = true;
    const long get_micros = measureMicros([&] { got = client.get(0, status); });
    S2C_CHECK(!got);
    S2C_CHECK(get_micros < kMaxCallMicros);

    bool sent = true;
    const long send_micros = measureMicros([&] {
        sent = client.send(0, ipc_test::CommandData{1, 1});
    });
    S2C_CHECK(!sent);
    S2C_CHECK(send_micros < kMaxCallMicros);
}

}  // namespace

int main() {
    testGetAndSend();
    testQueueOverwrite();
    testConcurrentClients();
    testDeadOwnerRecovery();
    testServerStopAndRestart();
    testStopWithoutDestruction();
    testStopClearsStaleRequests();
    testCorruptActiveIndex();
    testKilledServerProcess();
    testNoServer();
    std::printf("所有 IPC 测试通过\n");
    return 0;
}
