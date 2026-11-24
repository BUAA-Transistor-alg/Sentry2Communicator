// tools/test_serial_mcu_torque.cpp — 单轴力矩阶跃测试
//
// 启动时指定一个轴，随后交互输入力矩值（-1 到 1 之间的小数）。每个力矩值的流程:
//   用 1s 从 0 升到输入值 → 打印提醒 → 保持 3s（统计电控返回的实际力矩极值）
//   → 打印提醒 → 用 1s 降到 0 → 打印提醒 → 打印四个电机温度与保持段实际力矩最小值/最大值
//   → 等待下一次输入。
// 任一电机温度超过 55℃ 立刻输出 0 力矩，并每隔 1s 打印一次提醒与全部温度，
// 直至所有电机温度低于 50℃ 后回到等待输入。
// 收到 SIGINT/SIGTERM 后以 100Hz 发送 100 帧 0 力矩再退出。
// 除被指定轴的力矩外，其余发送值与本目录 test_serial_mcu.cpp 一致（摩擦轮关、火控关、
// 四轴仅力矩模式、零目标位置与速度、变形位与底盘位移为零）。
//
// 运行: ./build/s2c_test_serial_mcu_torque <yaw_big|roll|yaw_small|pitch> [--port <设备节点>]
//       --port 跳过 USB iProduct 筛选直接打开该设备节点（自测时配合 pty 使用）。

#include "Communications.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace s2c {
namespace {

using Clock = std::chrono::steady_clock;

constexpr auto FRAME_PERIOD = std::chrono::milliseconds(10);   // 100Hz
constexpr int RAMP_FRAMES = 100;                                // 1s
constexpr int HOLD_FRAMES = 300;                                // 3s
constexpr int ZERO_FRAMES_ON_EXIT = 100;                        // Ctrl+C 退出时发送的 0 力矩帧数
constexpr int TEMPERATURE_LIMIT_HIGH = 55;                      // 超过该温度立刻输出 0 力矩
constexpr int TEMPERATURE_LIMIT_RESUME = 50;                     // 全部低于该温度后恢复

std::atomic<bool> keep_running{true};

void signalHandler(int /*signum*/) { keep_running = false; }

// ── 轴选择：发送力矩与反馈的实际力矩、温度字段 ──
struct AxisInfo {
    const char* name;
    float com::mcu::SendPacket::*send_torque;
    float com::mcu::ReceivePacket::*actual_torque;
    uint8_t com::mcu::ReceivePacket::*temperature;
};

const AxisInfo AXES[] = {
    {"yaw_big", &com::mcu::SendPacket::yaw_big_torque,
     &com::mcu::ReceivePacket::yaw_big_torque, &com::mcu::ReceivePacket::yaw_big_temperature},
    {"roll", &com::mcu::SendPacket::roll_torque,
     &com::mcu::ReceivePacket::roll_torque, &com::mcu::ReceivePacket::roll_temperature},
    {"yaw_small", &com::mcu::SendPacket::yaw_small_torque,
     &com::mcu::ReceivePacket::yaw_small_torque, &com::mcu::ReceivePacket::yaw_small_temperature},
    {"pitch", &com::mcu::SendPacket::pitch_torque,
     &com::mcu::ReceivePacket::pitch_torque, &com::mcu::ReceivePacket::pitch_temperature},
};

// ── 反馈缓存：串口线程写入，控制循环读取 ──
std::mutex feedback_mutex;
com::mcu::ReceivePacket latest_feedback{};
uint64_t feedback_count = 0;

struct FeedbackSnapshot {
    bool valid = false;
    com::mcu::ReceivePacket packet{};
    uint64_t count = 0;
};

void onReceive(const com::mcu::ReceivePacket& packet) {
    std::lock_guard<std::mutex> lock(feedback_mutex);
    latest_feedback = packet;
    ++feedback_count;
}

FeedbackSnapshot snapshotFeedback() {
    std::lock_guard<std::mutex> lock(feedback_mutex);
    FeedbackSnapshot snapshot;
    snapshot.valid = feedback_count > 0;
    snapshot.packet = latest_feedback;
    snapshot.count = feedback_count;
    return snapshot;
}

// ── 标准输入：后台线程按行读取，控制循环每帧取走已到达的整行 ──
std::mutex input_mutex;
std::deque<std::string> input_lines;
std::atomic<bool> input_finished{false};
std::atomic<bool> input_stop{false};

void pushInputLine(std::string line) {
    std::lock_guard<std::mutex> lock(input_mutex);
    input_lines.push_back(std::move(line));
}

void inputThread() {
    std::string buffer;
    while (!input_stop.load()) {
        struct pollfd pfd;
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;
        pfd.revents = 0;
        const int ready = ::poll(&pfd, 1, 100);
        if (ready < 0) {
            if (errno == EINTR) continue;
            input_finished.store(true);
            return;
        }
        if (ready == 0) continue;
        char chunk[256];
        const ssize_t n = ::read(STDIN_FILENO, chunk, sizeof(chunk));
        if (n > 0) {
            buffer.append(chunk, static_cast<size_t>(n));
            for (;;) {
                const size_t pos = buffer.find('\n');
                if (pos == std::string::npos) break;
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                pushInputLine(std::move(line));
            }
            continue;
        }
        if (n == 0) {
            if (!buffer.empty()) pushInputLine(std::move(buffer));
            input_finished.store(true);
            return;
        }
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
        input_finished.store(true);
        return;
    }
}

std::vector<std::string> drainInput() {
    std::vector<std::string> lines;
    std::lock_guard<std::mutex> lock(input_mutex);
    while (!input_lines.empty()) {
        lines.push_back(std::move(input_lines.front()));
        input_lines.pop_front();
    }
    return lines;
}

bool inputQueueEmpty() {
    std::lock_guard<std::mutex> lock(input_mutex);
    return input_lines.empty();
}

// ── 文本处理 ──
std::string trim(const std::string& text) {
    size_t begin = 0;
    while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])) != 0) ++begin;
    size_t end = text.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) --end;
    return text.substr(begin, end - begin);
}

std::string formatTorque(float value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << value;
    return stream.str();
}

bool parseTorque(const std::string& line, float& torque) {
    const std::string text = trim(line);
    if (text.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const float value = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0') return false;
    if (errno == ERANGE || !std::isfinite(value)) return false;
    if (value < -1.0f || value > 1.0f) return false;
    torque = value;
    return true;
}

// ── 发送包构造：与 test_serial_mcu.cpp 的发送值一致，仅被指定轴的力矩另行赋值 ──
com::mcu::SendPacket makeBasePacket() {
    com::mcu::SendPacket packet;
    packet.friction_wheel_enable = 0;
    packet.fire = 0;
    packet.deform_control = 0;
    packet.displacement_target_velocity_x = 0.0f;
    packet.displacement_target_velocity_y = 0.0f;
    packet.displacement_target_velocity_yaw = 0.0f;
    // ── 大 yaw ──
    packet.yaw_big_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    packet.yaw_big_target_position = 0.0;
    packet.yaw_big_target_velocity = 0.0f;
    packet.yaw_big_torque = 0.0f;
    // ── roll ──
    packet.roll_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    packet.roll_target_position = 0.0f;
    packet.roll_target_velocity = 0.0f;
    packet.roll_torque = 0.0f;
    // ── 小 yaw ──
    packet.yaw_small_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    packet.yaw_small_target_position = 0.0f;
    packet.yaw_small_target_velocity = 0.0f;
    packet.yaw_small_torque = 0.0f;
    // ── pitch ──
    packet.pitch_mode = com::mcu::AXIS_MODE_TORQUE_ONLY;
    packet.pitch_target_position = 0.0f;
    packet.pitch_target_velocity = 0.0f;
    packet.pitch_torque = 0.0f;
    return packet;
}

// ── 温度 ──
void printTemperatures(const com::mcu::ReceivePacket& packet) {
    std::cout << "  yaw_big=" << static_cast<int>(packet.yaw_big_temperature) << "C"
              << " roll=" << static_cast<int>(packet.roll_temperature) << "C"
              << " yaw_small=" << static_cast<int>(packet.yaw_small_temperature) << "C"
              << " pitch=" << static_cast<int>(packet.pitch_temperature) << "C"
              << std::endl;
}

bool anyMotorAboveHigh(const com::mcu::ReceivePacket& packet) {
    return packet.yaw_big_temperature > TEMPERATURE_LIMIT_HIGH
        || packet.roll_temperature > TEMPERATURE_LIMIT_HIGH
        || packet.yaw_small_temperature > TEMPERATURE_LIMIT_HIGH
        || packet.pitch_temperature > TEMPERATURE_LIMIT_HIGH;
}

bool allMotorsBelowResume(const com::mcu::ReceivePacket& packet) {
    return packet.yaw_big_temperature < TEMPERATURE_LIMIT_RESUME
        && packet.roll_temperature < TEMPERATURE_LIMIT_RESUME
        && packet.yaw_small_temperature < TEMPERATURE_LIMIT_RESUME
        && packet.pitch_temperature < TEMPERATURE_LIMIT_RESUME;
}

void printUsage(const char* program) {
    std::cout << "用法: " << program << " <yaw_big|roll|yaw_small|pitch> [--port <设备节点>]"
              << std::endl;
    std::cout << "  --port 跳过 USB iProduct 筛选直接打开该设备节点（自测时使用）" << std::endl;
}

void printPrompt() {
    std::cout << "请输入力矩值（-1 到 1 之间的小数，Ctrl+C 退出）: " << std::flush;
}

const AxisInfo* findAxis(const std::string& name) {
    for (const AxisInfo& info : AXES) {
        if (name == info.name) return &info;
    }
    return nullptr;
}

enum class State {
    Waiting,   // 等待输入
    RampUp,    // 1s 上升
    Hold,      // 保持 3s
    RampDown,  // 1s 下降
    Thermal,   // 过热保护
};

void printCycleResult(const AxisInfo& axis, bool has_sample,
                      float minimum, float maximum, int sample_count) {
    const FeedbackSnapshot feedback = snapshotFeedback();
    std::cout << "[结果] 当前所有电机温度:" << std::endl;
    if (feedback.valid) {
        printTemperatures(feedback.packet);
    } else {
        std::cout << "  尚未收到电控反馈，温度未知" << std::endl;
    }
    if (has_sample) {
        std::cout << "[结果] 保持 3s 内 " << axis.name << " 实际力矩: min=" << formatTorque(minimum)
                  << " max=" << formatTorque(maximum)
                  << "（反馈样本 " << sample_count << " 帧）" << std::endl;
    } else {
        std::cout << "[结果] 保持 3s 内未收到新的电控反馈，实际力矩最小值/最大值不可用"
                  << std::endl;
    }
}

}  // namespace
}  // namespace s2c

// main() 必须留在全局命名空间（否则不是程序入口）；
// 随后把 namespace s2c 内的名字引入作用域，便于 main() 直接使用。
using namespace s2c;

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    const AxisInfo* axis = findAxis(argv[1]);
    if (axis == nullptr) {
        std::cerr << "未知的轴: " << argv[1] << std::endl;
        printUsage(argv[0]);
        return 1;
    }
    std::string forced_port;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            forced_port = argv[++i];
        } else {
            std::cerr << "无法识别的参数: " << argv[i] << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "单轴力矩阶跃测试: axis=" << axis->name << std::endl;
    std::cout << "SendPacket size: " << sizeof(com::mcu::SendPacket) << " bytes" << std::endl;

    com::McuCommunication serial(onReceive, true, forced_port);
    std::cout << "Port open: " << (serial.isOpen() ? "yes" : "no")
              << ", port: " << (serial.portName().empty() ? "<none>" : serial.portName())
              << std::endl;
    std::cout << "发送频率 100Hz；Ctrl+C 退出时发送 " << ZERO_FRAMES_ON_EXIT << " 帧 0 力矩"
              << std::endl;

    std::thread input_thread(inputThread);

    State state = State::Waiting;
    float target_torque = 0.0f;
    int ramp_frame = 0;
    int hold_frame = 0;
    int down_frame = 0;
    uint64_t hold_start_feedback_count = 0;
    bool hold_has_sample = false;
    float hold_min = 0.0f;
    float hold_max = 0.0f;
    int hold_sample_count = 0;
    Clock::time_point next_temperature_report{};
    Clock::time_point last_send_warning{};
    Clock::time_point next_frame = Clock::now();
    uint64_t failed_send_count = 0;

    printPrompt();

    while (keep_running.load()) {
        const FeedbackSnapshot feedback = snapshotFeedback();
        const Clock::time_point now = Clock::now();

        // ── 处理已经到达的输入行 ──
        for (const std::string& line : drainInput()) {
            const std::string text = trim(line);
            if (text.empty()) continue;
            if (state == State::Thermal) {
                std::cout << "[提示] 过热保护中，忽略输入: " << text << std::endl;
                continue;
            }
            if (state != State::Waiting) {
                std::cout << "[提示] 当前力矩测试尚未结束，忽略输入: " << text << std::endl;
                continue;
            }
            float value = 0.0f;
            if (!parseTorque(text, value)) {
                std::cout << "[错误] \"" << text << "\" 不是 -1 到 1 之间的小数，请重新输入"
                          << std::endl;
                printPrompt();
                continue;
            }
            target_torque = value;
            ramp_frame = 0;
            state = State::RampUp;
            std::cout << "[开始] 目标力矩 " << formatTorque(target_torque)
                      << "，用 1s 从 0 升到目标值" << std::endl;
        }

        // ── 过热保护：任一电机超过上限立刻输出 0 力矩，全部低于恢复线后回到等待输入 ──
        if (feedback.valid && anyMotorAboveHigh(feedback.packet)) {
            if (state != State::Thermal) {
                std::cout << "[警告] 有电机温度超过 " << TEMPERATURE_LIMIT_HIGH
                          << "C，立即输出 0 力矩并进入过热保护" << std::endl;
                printTemperatures(feedback.packet);
                state = State::Thermal;
                target_torque = 0.0f;
                ramp_frame = 0;
                hold_frame = 0;
                down_frame = 0;
                hold_has_sample = false;
                hold_sample_count = 0;
                next_temperature_report = now + std::chrono::seconds(1);
            }
        }

        if (state == State::Thermal) {
            if (feedback.valid && allMotorsBelowResume(feedback.packet)) {
                state = State::Waiting;
                std::cout << "[恢复] 所有电机温度已低于 " << TEMPERATURE_LIMIT_RESUME << "C"
                          << std::endl;
                printTemperatures(feedback.packet);
                printPrompt();
            } else if (now >= next_temperature_report) {
                std::cout << "[警告] 过热保护中，持续输出 0 力矩，等待所有电机温度低于 "
                          << TEMPERATURE_LIMIT_RESUME << "C" << std::endl;
                if (feedback.valid) {
                    printTemperatures(feedback.packet);
                } else {
                    std::cout << "  尚未收到电控反馈，温度未知" << std::endl;
                }
                next_temperature_report = now + std::chrono::seconds(1);
            }
        }

        // ── 本帧被指定轴的力矩 ──
        float torque = 0.0f;
        switch (state) {
            case State::RampUp:
                torque = target_torque * static_cast<float>(ramp_frame + 1)
                       / static_cast<float>(RAMP_FRAMES);
                break;
            case State::Hold:
                torque = target_torque;
                break;
            case State::RampDown:
                torque = target_torque * static_cast<float>(RAMP_FRAMES - (down_frame + 1))
                       / static_cast<float>(RAMP_FRAMES);
                break;
            case State::Waiting:
            case State::Thermal:
                torque = 0.0f;
                break;
        }

        // ── 以 100Hz 发送一帧 ──
        com::mcu::SendPacket packet = makeBasePacket();
        packet.*(axis->send_torque) = torque;
        if (serial.sendData(packet)) {
            failed_send_count = 0;
        } else {
            ++failed_send_count;
            if (now - last_send_warning > std::chrono::seconds(1)) {
                last_send_warning = now;
                std::cout << "[警告] 串口发送失败（端口未打开或写入背压），已连续失败 "
                          << failed_send_count << " 帧" << std::endl;
            }
        }

        // ── 状态推进与到点提醒 ──
        switch (state) {
            case State::RampUp:
                ++ramp_frame;
                if (ramp_frame >= RAMP_FRAMES) {
                    state = State::Hold;
                    hold_frame = 0;
                    hold_start_feedback_count = feedback.count;
                    hold_has_sample = false;
                    hold_min = 0.0f;
                    hold_max = 0.0f;
                    hold_sample_count = 0;
                    std::cout << "[提醒] 已用 1s 将 " << axis->name << " 力矩升到 "
                              << formatTorque(target_torque) << "，开始保持 3s" << std::endl;
                }
                break;
            case State::Hold:
                if (feedback.valid && feedback.count > hold_start_feedback_count) {
                    const float actual = feedback.packet.*(axis->actual_torque);
                    if (!hold_has_sample) {
                        hold_min = actual;
                        hold_max = actual;
                        hold_has_sample = true;
                    } else {
                        hold_min = std::min(hold_min, actual);
                        hold_max = std::max(hold_max, actual);
                    }
                    ++hold_sample_count;
                }
                ++hold_frame;
                if (hold_frame >= HOLD_FRAMES) {
                    state = State::RampDown;
                    down_frame = 0;
                    std::cout << "[提醒] 保持 3s 完成，开始用 1s 将 " << axis->name
                              << " 力矩降到 0" << std::endl;
                }
                break;
            case State::RampDown:
                ++down_frame;
                if (down_frame >= RAMP_FRAMES) {
                    state = State::Waiting;
                    std::cout << "[提醒] " << axis->name << " 力矩已降到 0" << std::endl;
                    printCycleResult(*axis, hold_has_sample, hold_min, hold_max,
                                     hold_sample_count);
                    printPrompt();
                }
                break;
            case State::Waiting:
            case State::Thermal:
                break;
        }

        // ── 输入已结束且回到等待输入：退出（随后发送 0 力矩帧） ──
        if (input_finished.load() && state == State::Waiting && inputQueueEmpty()) {
            break;
        }

        next_frame += FRAME_PERIOD;
        std::this_thread::sleep_until(next_frame);
    }

    // 先停止输入线程的读取意图，再立刻发送 0 力矩帧，避免退出时出现发送空档
    input_stop.store(true);

    com::mcu::SendPacket zero_packet = makeBasePacket();
    Clock::time_point next = Clock::now();
    int zero_sent = 0;
    std::cout << "[退出] 以 100Hz 发送 " << ZERO_FRAMES_ON_EXIT << " 帧 0 力矩" << std::endl;
    for (int i = 0; i < ZERO_FRAMES_ON_EXIT; ++i) {
        if (serial.sendData(zero_packet)) ++zero_sent;
        next += FRAME_PERIOD;
        std::this_thread::sleep_until(next);
    }
    std::cout << "[退出] 已发送 " << zero_sent << "/" << ZERO_FRAMES_ON_EXIT
              << " 帧 0 力矩，退出" << std::endl;

    if (input_thread.joinable()) input_thread.join();
    return 0;
}
