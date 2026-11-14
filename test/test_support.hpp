// test_support.hpp — 测试公共工具：断言、pty 对、超时轮询、帧构造
//
// 链路测试用 Linux 伪终端对（pty）建立真实的串口收发路径：测试持有主端，
// 从端设备节点交给 SerialProtocol 打开，双方都走内核 tty 层与 termios 配置。
#pragma once

#include "CRC.h"
#include "serial_comm/SerialCommProtocol.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace s2c_test {

[[noreturn]] inline void fail(const char* file, int line, const std::string& message) {
    std::fflush(stdout);
    std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", file, line, message.c_str());
    std::abort();
}

#define S2C_CHECK(cond)                                                        \
    do {                                                                       \
        if (!(cond)) s2c_test::fail(__FILE__, __LINE__, #cond);                \
    } while (0)

// ── 伪终端对 ──
struct PtyPair {
    int master_fd = -1;
    std::string slave_path;
};

// 主端也置为原始模式，避免行规程改动测试字节
inline PtyPair makePtyPair() {
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    S2C_CHECK(master >= 0);
    S2C_CHECK(grantpt(master) == 0);
    S2C_CHECK(unlockpt(master) == 0);
    const char* name = ptsname(master);
    S2C_CHECK(name != nullptr);
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    S2C_CHECK(tcgetattr(master, &tty) == 0);
    cfmakeraw(&tty);
    S2C_CHECK(tcsetattr(master, TCSANOW, &tty) == 0);
    PtyPair pair;
    pair.master_fd = master;
    pair.slave_path = std::string(name);
    return pair;
}

inline void closePtyPair(PtyPair& pair) {
    if (pair.master_fd >= 0) {
        close(pair.master_fd);
        pair.master_fd = -1;
    }
}

inline void writeAll(int fd, const void* data, size_t size) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    size_t written = 0;
    while (written < size) {
        ssize_t n = write(fd, bytes + written, size - written);
        S2C_CHECK(n > 0);
        written += static_cast<size_t>(n);
    }
}

// ── 超时轮询 ──
template <typename Predicate>
inline bool waitFor(Predicate predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return predicate();
}

// 带超时读取主端收到的字节
inline std::vector<uint8_t> readWithTimeout(int fd, size_t expected,
                                            std::chrono::milliseconds timeout) {
    std::vector<uint8_t> received;
    received.reserve(expected);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (received.size() < expected && std::chrono::steady_clock::now() < deadline) {
        uint8_t buffer[256];
        ssize_t n = read(fd, buffer, sizeof(buffer));
        if (n > 0) {
            received.insert(received.end(), buffer, buffer + n);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    return received;
}

// ── 帧构造（与协议一致: 前导 + data_size + payload + 帧尾 CRC）──
inline std::vector<uint8_t> makeMcuReceiveFrame(s2c::com::mcu::ReceivePacket packet) {
    packet.data_size = sizeof(s2c::com::mcu::ReceivePacket) - 3 - 1 - 1;
    uint8_t* raw = reinterpret_cast<uint8_t*>(&packet);
    const uint8_t crc = s2c::com::CRC8_Check_Sum(raw, sizeof(packet) - 1);
    std::memcpy(raw + sizeof(packet) - 1, &crc, 1);
    return std::vector<uint8_t>(raw, raw + sizeof(packet));
}

inline std::vector<uint8_t> makeImuReceiveFrame(s2c::com::imu::ReceivePacket packet) {
    packet.data_size = sizeof(s2c::com::imu::ReceivePacket) - 3 - 1 - 4;
    uint8_t* raw = reinterpret_cast<uint8_t*>(&packet);
    const uint32_t crc = s2c::com::CRC32_Calculate(raw, sizeof(packet) - 4);
    std::memcpy(raw + sizeof(packet) - 4, &crc, 4);
    return std::vector<uint8_t>(raw, raw + sizeof(packet));
}

// ── 回调收集 ──
template <typename PacketT>
struct PacketSink {
    mutable std::mutex mutex;
    std::vector<PacketT> packets;

    void push(const PacketT& packet) {
        std::lock_guard<std::mutex> lock(mutex);
        packets.push_back(packet);
    }
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex);
        return packets.size();
    }
    PacketT at(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex);
        return packets.at(index);
    }
};

inline bool nearlyEqual(double a, double b, double tolerance = 1e-6) {
    return (a > b ? a - b : b - a) <= tolerance;
}

}  // namespace s2c_test
