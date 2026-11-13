// Communications.hpp — 基于 SerialProtocol 模板的具体通信类型定义
//
// McuCommunication : 与电控（MCU）通信，CRC8，前导 0x42 0x52 0x04
// ImuCommunication : 与大 yaw 上的 IMU 通信，CRC32，前导 0xA7 0xB6 0xC5
//
// RobotCommunication 组合: 两个串口 + McuDataPreprocessor
//   - MCU 回调里经预处理器后缓存"处理前 / 处理后"两份包（预处理器当前原样返回）
//   - IMU 回调里缓存最新包（IMU 固定在大 yaw 转子，可信、高频）
//
// 端口筛选: USB 设备的 iProduct 为 AutoAim_IMU_Com 的给 IMU，其余给 MCU；
//           构造时可显式指定两个设备节点（跳过筛选），供现场固定端口与自测使用。
#ifndef S2C_COM_COMMUNICATIONS_HPP
#define S2C_COM_COMMUNICATIONS_HPP

#include "SerialProtocol.hpp"
#include "serial_comm/SerialProtocol.hpp"
#include "CRC.h"
#include "McuDataPreprocessor.h"
#include <string>
#include <mutex>

namespace s2c {
namespace com {

// ── 端口筛选函数 ──
inline bool mcuPortSelector(const std::string& product_info) {
    return product_info != "AutoAim_IMU_Com";
}
inline bool imuPortSelector(const std::string& product_info) {
    return product_info == "AutoAim_IMU_Com";
}

// ── 具体通信类型别名 ──
using McuCommunication = SerialProtocol<
    mcu::SendPacket,
    mcu::ReceivePacket,
    CRC8_Check_Sum,
    mcuPortSelector,
    mcu::PREAMBLE_SIZE
>;

using ImuCommunication = SerialProtocol<
    imu::SendPacket,
    imu::ReceivePacket,
    CRC32_Calculate,
    imuPortSelector,
    imu::PREAMBLE_SIZE
>;

// ============================================================================
// RobotCommunication — 组合 MCU 与 IMU 通信与数据预处理
// ============================================================================
class RobotCommunication {
public:
    // 显式端口: 字段为空串表示该串口仍按 USB iProduct 筛选
    struct PortNames {
        std::string mcu;
        std::string imu;
    };

    struct LatestData {
        bool               imu_valid = false;
        imu::ReceivePacket imu_packet{};
        bool               mcu_valid = false;
        mcu::ReceivePacket mcu_packet{};   // 预处理后（当前与原始包相同）
        // ★ **处理前**的原始串口包（追加在末尾，已有字段顺序保持不变）。
        //   mcu_packet 是 McuDataPreprocessor 的输出；raw_mcu_packet 是它的输入
        //   （调用方可据此同时取得处理前与处理后两份数据）。
        //   IMU 不经预处理器，imu_packet 本身即原始包；raw_imu_packet 为对称命名的副本。
        mcu::ReceivePacket raw_mcu_packet{};
        imu::ReceivePacket raw_imu_packet{};
    };

    explicit RobotCommunication(const PortNames& ports = PortNames{})
        : mcu_serial_([this](const mcu::ReceivePacket& pkt) { onMcuReceive(pkt); }, false, ports.mcu)
        , imu_serial_([this](const imu::ReceivePacket& pkt) { onImuReceive(pkt); }, false, ports.imu)
    {
        mcu_serial_.startWorker();
        imu_serial_.startWorker();
    }

    ~RobotCommunication() {
        mcu_serial_.stopWorker();
        imu_serial_.stopWorker();
    }

    // 获取最新原始数据（MCU 数据经预处理器，当前为原样返回）
    LatestData getLatestData() {
        LatestData data;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (has_imu_data_) {
                data.imu_packet = latest_imu_packet_;
                data.raw_imu_packet = latest_imu_packet_;   // IMU 无预处理 → 与 imu_packet 相同
                data.imu_valid = true;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            if (has_mcu_data_) {
                data.mcu_packet = latest_mcu_packet_;
                data.raw_mcu_packet = latest_mcu_raw_;      // ★ 处理前原始包
                data.mcu_valid = true;
            }
        }
        return data;
    }

    // ── 原始（处理前）串口包的只读访问器 ──
    // 返回**副本**: onXxxReceive 由串口后台线程写入，返回引用会与写入竞争
    // （调用方拿到的引用可能在读取过程中被后台线程改写）。需要"同一次加锁的一致快照"
    // 时请用 getLatestData()。
    mcu::ReceivePacket lastRawMcuPacket() {
        std::lock_guard<std::mutex> lock(mcu_mutex_);
        return latest_mcu_raw_;
    }
    imu::ReceivePacket lastImuPacket() {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        return latest_imu_packet_;
    }

    // 发送 MCU 数据（发送前经预处理器，当前为原样返回）
    bool sendToMcu(mcu::SendPacket packet) {
        mcu::SendPacket processed = preprocessor_.processSend(packet);
        return mcu_serial_.sendData(processed);
    }

    // 发送 IMU 数据（心跳等，无预处理）
    bool sendToImu(imu::SendPacket packet) {
        return imu_serial_.sendData(packet);
    }

    void stop() {
        mcu_serial_.stopWorker();
        imu_serial_.stopWorker();
    }

private:
    void onImuReceive(const imu::ReceivePacket& packet) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_packet_ = packet;
        has_imu_data_ = true;
    }

    void onMcuReceive(const mcu::ReceivePacket& packet) {
        // ★ 在 processReceive **之前**保留处理前的原始串口包
        //   （latest_mcu_packet_ 是它的输出；两份数据分别供调用方使用）
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            latest_mcu_raw_ = packet;
        }
        const mcu::ReceivePacket processed = preprocessor_.processReceive(packet);
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            latest_mcu_packet_ = processed;
            has_mcu_data_ = true;
        }
    }

    McuDataPreprocessor  preprocessor_;
    McuCommunication     mcu_serial_;
    ImuCommunication     imu_serial_;

    std::mutex         imu_mutex_;
    imu::ReceivePacket latest_imu_packet_{};
    bool               has_imu_data_ = false;

    std::mutex         mcu_mutex_;
    mcu::ReceivePacket latest_mcu_packet_{};   // McuDataPreprocessor 输出（当前原样返回）
    mcu::ReceivePacket latest_mcu_raw_{};      // ★ 处理前原始串口包（processReceive 输入）
    bool               has_mcu_data_ = false;
};

}  // namespace com
}  // namespace s2c

#endif // S2C_COM_COMMUNICATIONS_HPP
