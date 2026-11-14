// SerialProtocol.hpp — 模板化串口协议通信类
// 模板参数:
//   SendPacketT     : 发送包类型（完整 wire-format 结构体，含前导/长度/CRC）
//   ReceivePacketT  : 接收包类型（完整 wire-format 结构体，含前导/长度/CRC）
//   CRC_Func        : CRC 计算函数，签名 CRCType(const uint8_t*, size_t)
//   PortSelector    : 端口筛选函数，签名 bool(const std::string& product_info)
//   PreambleLen     : 帧同步前导字节数（前导字节值取自 ReceivePacketT 前 N 字节）
//
// CRC 返回值类型和字节数从 CRC_Func 自动推导。
// 帧格式: [前导(PreambleLen 字节)] [data_size(1B)] [payload] [CRC(CRC_SIZE)]
//
// 端口: 默认枚举 /dev/ttyACM*，由 udev 读 USB 设备的 iProduct 交给 PortSelector 筛选；
//       构造时也可传入显式设备节点（跳过枚举），供现场固定端口与无 USB 设备信息
//       的环境（自测用的 pty 对）使用。
//
// 波特率: 可用编译期宏 SERIAL_BAUD_RATE 覆盖（默认 B921600）。
// 注意: 本工程发送帧 72B、接收帧 242B。串口通信服务端按 300Hz 发送
//       （72B×300 ≈ 21.6 kB/s），电控按 100Hz 上报时接收 ≈ 24.2 kB/s，
//       双向合计 ≈ 45.8 kB/s，460800(≈46.1 kB/s) 余量很小，默认使用 921600(≈92.2 kB/s)。
//       电控上报频率更高时需重新评估，只需在本文件与电控侧各改一处。
//
#ifndef S2C_COM_SERIALPROTOCOL_HPP
#define S2C_COM_SERIALPROTOCOL_HPP

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <cstring>
#include <stdint.h>
#include <errno.h>
#include <vector>
#include <array>
#include <mutex>
#include <atomic>
#include <chrono>
#include <dirent.h>
#include <sys/types.h>
#include <functional>
#include <string>
#include <libudev.h>
#include <thread>
#include <algorithm>
#include <type_traits>

#ifndef SERIAL_BAUD_RATE
#define SERIAL_BAUD_RATE B921600
#endif

namespace s2c {
namespace com {

// 从函数指针类型提取返回值类型
template <typename T>
struct FunctionTraits;

template <typename R, typename... Args>
struct FunctionTraits<R(*)(Args...)> {
    using return_type = R;
};

template <
    typename SendPacketT,
    typename ReceivePacketT,
    auto CRC_Func,
    auto PortSelector,
    size_t PreambleLen
>
class SerialProtocol {
public:
    // ── 从 CRC 函数自动推导 ──
    using CRCType = typename FunctionTraits<decltype(CRC_Func)>::return_type;
    static constexpr size_t CRC_SIZE       = sizeof(CRCType);
    static constexpr size_t PREAMBLE_SIZE  = PreambleLen;

    // data_size 字段在前导字节之后
    static constexpr size_t DATA_SIZE_OFFSET = PREAMBLE_SIZE;

    // 帧最小长度 = 前导 + data_size 字段 + CRC
    static constexpr size_t FRAME_MIN_SIZE  = PREAMBLE_SIZE + 1 + CRC_SIZE;

    static_assert(std::is_trivially_copyable<SendPacketT>::value,
                  "SendPacketT 必须可逐字节复制");
    static_assert(std::is_standard_layout<SendPacketT>::value,
                  "SendPacketT 必须是标准布局");
    static_assert(std::is_trivially_copyable<ReceivePacketT>::value,
                  "ReceivePacketT 必须可逐字节复制");
    static_assert(std::is_standard_layout<ReceivePacketT>::value,
                  "ReceivePacketT 必须是标准布局");

    // forced_port 为空串时按 udev 筛选端口；非空时直接打开该设备节点（不枚举、不筛选）
    SerialProtocol(std::function<void(const ReceivePacketT&)> callback,
                   bool auto_start = true,
                   std::string forced_port = std::string());
    ~SerialProtocol();
    void startWorker();
    void stopWorker();
    bool sendData(SendPacketT& packet);

    // ── 链路自检用的只读状态（排查"到底选中/打开了哪个口"）──
    //   isOpen()  : 端口已打开或最近一次完整读/写成功；发送背压期间可能保持 true
    //   portName(): 实际选中的设备节点（未选中则为空串）
    bool        isOpen()   const { return port_open_.load(); }
    std::string portName() const {
        std::lock_guard<std::mutex> lock(fd_mutex_);
        return port_;
    }

    // ── 端口枚举（只读工具 / 现场排查"为什么找不到串口"用）──
    //   getSerialProductInfo : 由 udev 读 USB 设备的 iProduct（与 PortSelector 的入参同一来源）
    //   findAvailableSerialPorts : 当前可打开的 /dev/ttyACM* 列表
    //   两者都不依赖实例状态，故为 static，可脱离实例单独调用。
    static std::string              getSerialProductInfo(const std::string& port);
    static std::vector<std::string> findAvailableSerialPorts();

private:
    static constexpr size_t BUFFER_SIZE = 1024;

    // 前导序列：构造时从默认 ReceivePacketT 的前 PREAMBLE_SIZE 字节提取并保存
    std::array<uint8_t, PREAMBLE_SIZE> preamble_bytes_;

    int fd_;
    mutable std::mutex fd_mutex_;
    std::array<uint8_t, BUFFER_SIZE> buffer_;
    size_t buffer_index_ = 0;
    std::array<uint8_t, sizeof(SendPacketT)> tx_pending_{};   // 上一帧未写完的尾部
    size_t tx_pending_size_ = 0;

    std::function<void(const ReceivePacketT&)> serialDataCallback;
    std::atomic<bool> running{true};
    std::thread recv_thread_;

    std::chrono::steady_clock::time_point last_reconnect_time;
    std::chrono::steady_clock::time_point last_received_time;
    std::chrono::steady_clock::time_point last_tx_error_time_{};   // 发送失败日志限速
    std::atomic<uint64_t> rx_crc_errors_{0};
    std::atomic<uint64_t> rx_length_errors_{0};
    std::chrono::steady_clock::time_point last_rx_error_log_{};
    std::chrono::steady_clock::time_point last_link_log_time_{};

    // ── 只读状态（声明在最后：构造函数初始化列表顺序必须与声明顺序一致，见构造函数里的 -Wreorder 注释）──
    std::string             port_;              // initializeSerial() 选中的端口，未选中为空
    std::string             forced_port_;       // 非空时直接打开该节点
    std::atomic<bool>       port_open_{false};  // 是否已成功 open

    void initializeSerial();
    void logLinkProblem(const char* message);
    void logRxErrors();
    void processFrame(const uint8_t* data, size_t frame_length);
    void processBuffer();
    void tryReconnect();
    void timerCallback();
    void timerThread();
};

// ============================================================================
// 模板成员函数实现
// ============================================================================

template <typename S, typename R, auto C, auto P, size_t L>
SerialProtocol<S, R, C, P, L>::SerialProtocol(std::function<void(const R&)> callback,
                                              bool auto_start,
                                              std::string forced_port)
    : fd_(-1), serialDataCallback(callback),
      forced_port_(std::move(forced_port))   // 初始化顺序必须与成员声明顺序一致（否则 -Wreorder）
{
    // 从默认 ReceivePacketT 提取前导序列保存到成员变量
    const R dflt{};
    const uint8_t* src = reinterpret_cast<const uint8_t*>(&dflt);
    std::copy_n(src, PREAMBLE_SIZE, preamble_bytes_.begin());

    initializeSerial();
    last_reconnect_time = std::chrono::steady_clock::now();
    last_received_time = std::chrono::steady_clock::now();
    if (auto_start) {
        startWorker();
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
SerialProtocol<S, R, C, P, L>::~SerialProtocol() {
    stopWorker();
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::startWorker() {
    if (recv_thread_.joinable()) {
        running = false;
        recv_thread_.join();
    }
    // 重新启动接收线程时丢弃上次停止前的缓冲与时间基准，避免立即触发重连
    last_received_time = std::chrono::steady_clock::now();
    last_reconnect_time = last_received_time;
    {
        // tx_pending_size_ 由发送线程在 fd_mutex_ 内访问，重启接收线程时同样持锁重置
        std::lock_guard<std::mutex> lock(fd_mutex_);
        buffer_index_ = 0;
        tx_pending_size_ = 0;
    }
    running = true;
    try {
        recv_thread_ = std::thread(&SerialProtocol::timerThread, this);
    } catch (...) {
        running = false;
        port_open_.store(false);
        std::lock_guard<std::mutex> lock(fd_mutex_);
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        throw;   // 构造函数失败时避免泄漏已打开的串口
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::stopWorker() {
    running = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
bool SerialProtocol<S, R, C, P, L>::sendData(S& packet) {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ < 0) {
        return false;
    }

    // 先把上一帧未写完的尾部补发完，避免电控收到截断帧
    // 残尾补发与新帧写入共用 1ms 等待预算：低于 2ms 发送周期，背压时不让发布与喂狗降频
    const auto tx_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
    while (tx_pending_size_ > 0) {
        const ssize_t written = ::write(fd_, tx_pending_.data(), tx_pending_size_);
        if (written > 0) {
            tx_pending_size_ -= static_cast<size_t>(written);
            if (tx_pending_size_ > 0) {
                std::memmove(tx_pending_.data(), tx_pending_.data() + written, tx_pending_size_);
                if (std::chrono::steady_clock::now() >= tx_deadline) {
                    return false;   // 部分写持续推进也受 1ms 总预算约束
                }
            }
            continue;
        }
        if (written == 0) errno = EAGAIN;   // 无写空间按背压处理
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            const auto now = std::chrono::steady_clock::now();
            if (now < tx_deadline) {
                struct pollfd pfd;
                pfd.fd = fd_;
                pfd.events = POLLOUT;
                pfd.revents = 0;
                const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                    tx_deadline - now).count();
                const int wait_ms = remaining < 1 ? 1 : static_cast<int>(remaining);
                const int ready = ::poll(&pfd, 1, wait_ms);
                if (ready > 0 && (pfd.revents & POLLOUT) != 0) continue;
                if (ready < 0 && errno == EINTR) continue;
                if (ready > 0) {
                    port_open_.store(false);
                    return false;   // 保留残尾，等设备恢复或接收线程关闭 fd
                }
                if (ready < 0) {
                    port_open_.store(false);
                    return false;   // poll 自身错误同样保留残尾
                }
                errno = ETIMEDOUT;
            } else {
                errno = ETIMEDOUT;
            }
            return false;   // 背压：残尾保留到下一轮继续补发
        }
        // 写入错误同样保留残尾；重连时 initializeSerial 会清空，避免新帧接到残帧后面
        if (errno == ENODEV || errno == EIO || errno == EBADF) port_open_.store(false);
        return false;
    }

    CRCType crc = C(reinterpret_cast<const uint8_t*>(&packet), sizeof(S) - CRC_SIZE);
    memcpy(reinterpret_cast<uint8_t*>(&packet) + sizeof(S) - CRC_SIZE, &crc, CRC_SIZE);

    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&packet);
    for (;;) {
        const ssize_t written = ::write(fd_, bytes, sizeof(S));
        if (written > 0) {
            if (static_cast<size_t>(written) == sizeof(S)) {
                port_open_.store(true);   // 完整写出后恢复 isOpen()
                return true;
            }
            // 部分写入：保存余量，下一次 sendData 先补发，链路上不会出现半帧夹新帧
            const size_t remaining = sizeof(S) - static_cast<size_t>(written);
            std::memcpy(tx_pending_.data(), bytes + written, remaining);
            tx_pending_size_ = remaining;
            return false;
        }
        if (written == 0) errno = EAGAIN;   // 无写空间按背压处理
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            const auto now = std::chrono::steady_clock::now();
            if (now < tx_deadline) {
                struct pollfd pfd;
                pfd.fd = fd_;
                pfd.events = POLLOUT;
                pfd.revents = 0;
                const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                    tx_deadline - now).count();
                const int wait_ms = remaining < 1 ? 1 : static_cast<int>(remaining);
                const int ready = ::poll(&pfd, 1, wait_ms);
                if (ready > 0 && (pfd.revents & POLLOUT) != 0) continue;
                if (ready < 0 && errno == EINTR) continue;
                if (ready < 0) {
                    port_open_.store(false);
                    return false;   // poll 自身错误按链路失效处理
                }
                if (ready > 0) errno = EIO;
                else errno = ETIMEDOUT;
            } else {
                errno = ETIMEDOUT;
            }
        }
        if (errno == ENODEV || errno == EIO || errno == EBADF) port_open_.store(false);
        const auto now = std::chrono::steady_clock::now();
        if (now - last_tx_error_time_ > std::chrono::seconds(1)) {
            last_tx_error_time_ = now;
            printf("TX write failed: errno=%d\n", errno);
        }
        return false;
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
std::string SerialProtocol<S, R, C, P, L>::getSerialProductInfo(const std::string& port) {  // static
    struct udev *udev;
    struct udev_device *dev;
    std::string result = "";
    udev = udev_new();
    if (!udev) return "";   // 读不到产品信息时返回空串，由端口筛选函数决定是否可用
    dev = udev_device_new_from_subsystem_sysname(udev, "tty", port.c_str());
    if (!dev) { udev_unref(udev); return ""; }
    struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
    if (parent) {
        const char *product = udev_device_get_sysattr_value(parent, "product");
        if (product) result += std::string(product);
    }
    udev_device_unref(dev);
    udev_unref(udev);
    return result;
}

template <typename S, typename R, auto C, auto P, size_t L>
std::vector<std::string> SerialProtocol<S, R, C, P, L>::findAvailableSerialPorts() {  // static
    struct dirent *entry;
    DIR *dp = opendir("/dev/");
    if (dp == nullptr) { printf("Failed to open /dev/ directory\n"); return {}; }
    std::vector<std::string> ports;
    while ((entry = readdir(dp)) != nullptr) {
        if (strncmp(entry->d_name, "ttyACM", 6) == 0) {
            std::string candidate_port = "/dev/" + std::string(entry->d_name);
            // 用 access 探测，避免 open/close 触发 USB CDC 的 DTR 复位脉冲
            if (::access(candidate_port.c_str(), R_OK | W_OK) == 0) {
                ports.push_back(candidate_port);
            }
        }
    }
    closedir(dp);
    return ports;
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::initializeSerial() {
    fd_ = -1;                    // 进入时保证"未打开"，所有提前 return 都保持关闭状态
    tx_pending_size_ = 0;
    port_open_ = false;          // 每次（含重连）都从"未打开"重新判定
    port_.clear();               // 端口状态跟随本次选口结果，避免重连失败后显示旧端口
    std::string port = forced_port_;
    if (port.empty()) {
        auto ports = findAvailableSerialPorts();
        if (ports.empty()) { logLinkProblem("No available serial port found!"); return; }
        for (const auto& test_port : ports) {
            if (P(getSerialProductInfo(test_port.substr(5)))) { port = test_port; break; }
        }
        if (port.empty()) { logLinkProblem("Target serial port not found!"); return; }
    }
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        char message[192];
        snprintf(message, sizeof(message), "Failed to open port %s: %s", port.c_str(), strerror(errno));
        logLinkProblem(message);
        return;
    }
    struct termios tty; memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) { logLinkProblem("tcgetattr failed"); close(fd_); fd_ = -1; return; }
    if (cfsetospeed(&tty, SERIAL_BAUD_RATE) != 0 || cfsetispeed(&tty, SERIAL_BAUD_RATE) != 0) {
        char message[192];
        snprintf(message, sizeof(message), "Failed to set baud rate %d on %s",
                 SERIAL_BAUD_RATE, port.c_str());
        logLinkProblem(message);
        close(fd_);
        fd_ = -1;
        return;
    }
    tty.c_cflag |= (CLOCAL | CREAD); tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB; tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~ICANON; tty.c_lflag &= ~ECHO; tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { logLinkProblem("tcsetattr failed"); close(fd_); fd_ = -1; return; }
    tcflush(fd_, TCIOFLUSH);
    port_ = port;                // termios 配置成功后才记录，失败路径保持空串
    port_open_ = true;
    printf("Serial initialized: %s\n", port.c_str());
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::logLinkProblem(const char* message) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_link_log_time_ < std::chrono::seconds(10)) return;
    last_link_log_time_ = now;
    printf("%s\n", message);
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::logRxErrors() {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_rx_error_log_ <= std::chrono::seconds(1)) return;
    last_rx_error_log_ = now;
    printf("RX errors: crc=%llu length=%llu\n",
           static_cast<unsigned long long>(rx_crc_errors_.load(std::memory_order_relaxed)),
           static_cast<unsigned long long>(rx_length_errors_.load(std::memory_order_relaxed)));
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::processFrame(const uint8_t* data, size_t frame_length) {
    (void)frame_length;   // 调用方已保证帧长与 ReceivePacketT 一致
    R packet{};
    memcpy(&packet, data, sizeof(R));
    serialDataCallback(packet);
    last_received_time = std::chrono::steady_clock::now();
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::processBuffer() {
    // data_size 必须与 ReceivePacketT 的结构体长度一致；不一致的帧丢弃前导后重新同步
    constexpr size_t EXPECTED_PAYLOAD = sizeof(R) - PREAMBLE_SIZE - 1 - CRC_SIZE;
    static_assert(EXPECTED_PAYLOAD <= 255, "接收包 payload 超过 data_size 字段上限");
    while (buffer_index_ >= FRAME_MIN_SIZE) {
        size_t header_pos = 0;
        bool found_header = false;
        // 在整个缓冲区里找前导，不限制搜索窗口，避免 CRC 错误后紧接着的合法帧被丢掉
        while (header_pos + PREAMBLE_SIZE <= buffer_index_) {
            bool match = true;
            for (size_t k = 0; k < PREAMBLE_SIZE; ++k)
                if (buffer_[header_pos + k] != preamble_bytes_[k]) { match = false; break; }
            if (match) { found_header = true; break; }
            header_pos++;
        }
        if (!found_header) {
            if (buffer_index_ > PREAMBLE_SIZE - 1) {
                size_t keep = PREAMBLE_SIZE - 1;
                for (size_t i = 0; i < keep; ++i) buffer_[i] = buffer_[buffer_index_ - keep + i];
                buffer_index_ = keep;
            }
            return;
        }
        if (header_pos > 0) {
            memmove(buffer_.data(), buffer_.data() + header_pos, buffer_index_ - header_pos);
            buffer_index_ -= header_pos;
        }
        if (buffer_index_ < FRAME_MIN_SIZE) return;
        const uint8_t data_length = buffer_[DATA_SIZE_OFFSET];
        if (data_length != EXPECTED_PAYLOAD) {
            rx_length_errors_.fetch_add(1, std::memory_order_relaxed);
            logRxErrors();
            // 长度字段与 ReceivePacketT 不匹配（协议不一致或字段损坏）：丢前导后继续找下一帧
            memmove(buffer_.data(), buffer_.data() + PREAMBLE_SIZE, buffer_index_ - PREAMBLE_SIZE);
            buffer_index_ -= PREAMBLE_SIZE;
            continue;
        }
        const size_t frame_length = sizeof(R);
        if (buffer_index_ < frame_length) return;
        CRCType computed_crc = C(buffer_.data(), frame_length - CRC_SIZE);
        CRCType received_crc;
        memcpy(&received_crc, buffer_.data() + frame_length - CRC_SIZE, CRC_SIZE);
        if (computed_crc != received_crc) {
            rx_crc_errors_.fetch_add(1, std::memory_order_relaxed);
            logRxErrors();
            // 只丢弃前导后继续重新同步，保留紧随其后的合法帧
            memmove(buffer_.data(), buffer_.data() + PREAMBLE_SIZE, buffer_index_ - PREAMBLE_SIZE);
            buffer_index_ -= PREAMBLE_SIZE;
            continue;
        }
        processFrame(buffer_.data(), frame_length);
        if (frame_length < buffer_index_) {
            memmove(buffer_.data(), buffer_.data() + frame_length, buffer_index_ - frame_length);
            buffer_index_ -= frame_length;
        } else { buffer_index_ = 0; }
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::tryReconnect() {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) close(fd_);
    fd_ = -1;                    // 关闭后必须清空，避免重连选口失败时继续使用旧文件描述符
    buffer_index_ = 0;
    tx_pending_size_ = 0;        // 重连后旧帧的残尾不再续发
    initializeSerial();
    last_reconnect_time = std::chrono::steady_clock::now();
    last_received_time = std::chrono::steady_clock::now();
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::timerCallback() {
    // fd_ 的写入（重连/错误关闭）都发生在本接收线程内，因此这里的无锁读取与 write 侧
    // 的持锁访问不会构成竞争；不要把 timerCallback 移到其他线程调用
    auto now = std::chrono::steady_clock::now();
    if (fd_ < 0) {
        if (now - last_reconnect_time > std::chrono::seconds(3)) {
            logLinkProblem("Serial port not available, trying reconnect"); tryReconnect();
        }
        return;
    }
    if (now - last_received_time > std::chrono::seconds(3)) {
        if (now - last_reconnect_time > std::chrono::seconds(3)) {
            logLinkProblem("No data received, trying reconnect"); tryReconnect();
        }
        return;
    }
    if (buffer_index_ < BUFFER_SIZE - 256) {
        uint8_t temp_buffer[256];
        ssize_t bytes_read = read(fd_, temp_buffer, sizeof(temp_buffer));
        if (bytes_read > 0) {
            port_open_.store(true);   // 读成功说明设备可用，恢复 isOpen()
            memcpy(buffer_.data() + buffer_index_, temp_buffer, bytes_read);
            buffer_index_ += bytes_read;
            processBuffer();
        } else if (bytes_read < 0 && (errno == ENODEV || errno == EIO || errno == EBADF)) {
            port_open_.store(false);   // 设备已拔出或失效，isOpen() 立即反映
            // 在锁内关闭描述符：发送路径持同一把锁，关闭后不会再对陈旧 fd 写入
            std::lock_guard<std::mutex> lock(fd_mutex_);
            if (fd_ >= 0) {
                ::close(fd_);
                fd_ = -1;
            }
            port_.clear();   // 端口已失效，portName() 不再返回旧节点
        }
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::timerThread() {
    while (running) {
        auto start = std::chrono::steady_clock::now();
        timerCallback();
        std::this_thread::sleep_until(start + std::chrono::microseconds(100));
    }
}

}  // namespace com
}  // namespace s2c

#endif // S2C_COM_SERIALPROTOCOL_HPP
