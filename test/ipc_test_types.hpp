// test/ipc_test_types.hpp — IPC 测试与测试助手共用的内容类型与通道编号
#pragma once

#include <cstdint>

namespace ipc_test {

// 获取通道发布的内容
struct StatusData {
    uint32_t counter;
    float value;
};

struct AltStatus {
    uint64_t code;
};

// 用于验证类型长度不一致时 get 返回 false
struct ShortStatus {
    uint16_t value;
};

// 发送通道携带的内容
struct CommandData {
    uint32_t client_id;
    int32_t index;
};

struct AltCommand {
    double gain;
};

constexpr uint32_t kStatusChannel = 0;
constexpr uint32_t kAltStatusChannel = 1;
constexpr uint32_t kCommandChannel = 0;
constexpr uint32_t kAltCommandChannel = 1;

}  // namespace ipc_test
