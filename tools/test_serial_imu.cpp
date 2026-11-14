// tools/test_serial_imu.cpp — IMU 串口链路自检
//
// 打印每帧 IMU 接收包的角速度、加速度、欧拉角、帧间隔与 CRC，以及实际选中的串口
// （isOpen() / portName() 用于现场排查"到底选中/打开了哪个口"）。
// 10ms 周期向 IMU 发送一帧心跳（前导 + data_size=0 + CRC32）。
//
// 运行: ./build/s2c_test_serial_imu，Ctrl+C 退出

#include "Communications.hpp"

#include <atomic>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

namespace s2c {
namespace {

std::atomic<bool> keep_running{true};

void signalHandler(int /*signum*/) { keep_running = false; }

void onReceive(const com::imu::ReceivePacket& packet) {
    std::cout << "\n[Received IMU Packet]" << std::endl;
    std::cout << "  frame_header1:   0x" << std::hex << std::uppercase
              << static_cast<int>(packet.frame_header1) << std::dec << std::endl;
    std::cout << "  frame_header2:   0x" << std::hex << std::uppercase
              << static_cast<int>(packet.frame_header2) << std::dec << std::endl;
    std::cout << "  frame_header3:   0x" << std::hex << std::uppercase
              << static_cast<int>(packet.frame_header3) << std::dec << std::endl;
    std::cout << "  data_size:       " << static_cast<int>(packet.data_size) << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  gyro (rad/s):    gx=" << packet.gx
              << " gy=" << packet.gy
              << " gz=" << packet.gz << std::endl;
    std::cout << "  accel (m/s^2):   ax=" << packet.ax
              << " ay=" << packet.ay
              << " az=" << packet.az << std::endl;
    std::cout << "  euler (rad):     yaw=" << packet.euler_yaw
              << " pitch=" << packet.euler_pitch
              << " roll=" << packet.euler_roll << std::endl;
    std::cout << "  dt (0.1ms):      " << packet.dt_one_tenth_ms << std::endl;
    std::cout << "  crc32:           0x" << std::hex << std::uppercase
              << packet.crc32 << std::dec << std::endl;
    std::cout << std::endl;
}

}  // namespace
}  // namespace s2c

using namespace s2c;

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Starting IMU Serial Communication Test..." << std::endl;
    std::cout << "SendPacket size: " << sizeof(com::imu::SendPacket) << " bytes" << std::endl;
    std::cout << "ReceivePacket size: " << sizeof(com::imu::ReceivePacket) << " bytes" << std::endl;

    com::ImuCommunication serial(onReceive);

    std::cout << "Port open: " << (serial.isOpen() ? "yes" : "no")
              << ", port: " << (serial.portName().empty() ? "<none>" : serial.portName())
              << std::endl;

    std::cout << "Sending heartbeat every 10ms..." << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;

    com::imu::SendPacket packet;   // 默认前导、data_size=0，CRC32 由发送函数填写
    int send_count = 0;
    while (keep_running) {
        if (serial.sendData(packet)) {
            send_count++;
            if (send_count % 100 == 0) {
                std::cout << "Sent " << send_count << " heartbeats..." << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nExiting... Total heartbeats sent: " << send_count << std::endl;
    return 0;
}
