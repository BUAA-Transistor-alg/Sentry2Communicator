// SerialCommTypes.hpp — 串口通信服务器与客户端之间的共享内容定义
//
// 服务端维护要发给串口的数据与串口收到的最新数据，并按固定频率发布一份完整快照；
// 客户端读取这份快照，或按写入范围提交要更新的内容。
//
// 全部结构与通道编号在本文件定义，服务端与客户端两个工程都只依赖这一份声明。
#pragma once

#include "serial_comm/SerialProtocol.hpp"
#include "ipc/IpcLayout.hpp"

#include <chrono>
#include <cstdint>

namespace s2c {
namespace serial_comm {

// ── 通信参数 ──
constexpr const char* kDefaultShmName = "/s2c_serial_comm";
constexpr double kSendRateHz = 500.0;            // 服务端向串口发送与发布快照的频率
constexpr uint32_t kTargetCount = 6;             // 视觉向导航通信的目标列表长度
constexpr uint32_t kPredictionPointCount = 10;   // 导航向视觉通信的预测序列长度
constexpr double kPredictionIntervalSeconds = 0.1;

// ── 共享内存通道编号 ──
// 获取通道: 服务端发布的完整快照（AllInfo）
// 发送通道: 视觉写入、导航写入
constexpr uint32_t kAllInfoChannel = 0;
constexpr uint32_t kVisWriteChannel = 0;
constexpr uint32_t kNavWriteChannel = 1;

// ── 火控 ──
struct FireControlData {
    uint8_t friction_wheel_enable;   // 摩擦轮开关
    uint8_t fire;                    // 火控
};

// ── 四个轴的控制量 ──
struct AxisControlData {
    // 大 yaw
    uint8_t yaw_big_mode;               // com::mcu::AxisMode
    double  yaw_big_target_position;    // rad，多圈连续
    float   yaw_big_target_velocity;    // rad/s
    float   yaw_big_torque;             // N·m
    // roll
    uint8_t roll_mode;                  // com::mcu::AxisMode
    float   roll_target_position;       // rad
    float   roll_target_velocity;       // rad/s
    float   roll_torque;                // N·m
    // 小 yaw
    uint8_t yaw_small_mode;             // com::mcu::AxisMode
    float   yaw_small_target_position;  // rad
    float   yaw_small_target_velocity;  // rad/s
    float   yaw_small_torque;           // N·m
    // pitch
    uint8_t pitch_mode;                 // com::mcu::AxisMode
    float   pitch_target_position;      // rad
    float   pitch_target_velocity;      // rad/s
    float   pitch_torque;               // N·m
};

// ── 变形与底盘位移 ──
struct DeformChassisData {
    uint8_t deform_control;                   // 变形控制位
    float   displacement_target_velocity_x;   // 底盘位移目标速度 x（m/s）
    float   displacement_target_velocity_y;   // 底盘位移目标速度 y（m/s）
};

// ── 服务端维护的、要发送给串口的全部数据 ──
struct SerialSendData {
    FireControlData  fire_control;
    AxisControlData  axes;
    DeformChassisData deform_chassis;
};

// ── 视觉向导航通信 ──
struct TargetInfo {
    uint8_t recognized;      // 该类目标是否被识别到（0 / 1）
    float   enemy_x;         // 敌方相对自身 x（m，导航坐标系）
    float   enemy_y;         // 敌方相对自身 y（m，导航坐标系）
    float   enemy_z;         // 敌方相对自身 z（m，导航坐标系）
};

struct VisToNav {
    uint8_t auto_aim_state;            // 自瞄当前状态编号（整份通信共用）
    uint8_t target_id;                 // 瞄准目标编号（整份通信共用）
    TargetInfo targets[kTargetCount];  // 固定长度 6 的目标列表
};

// ── 导航向视觉通信 ──
enum class AutoAimMode : uint8_t {
    kArmor = 0,       // 装甲板
    kEnergyRune = 1,  // 能量机关
};

struct PredictedState {
    float position[3];   // 世界坐标系位置（m）
    float velocity[3];   // 世界坐标系速度（m/s）
};

struct NavToVision {
    AutoAimMode auto_aim_mode;          // 自瞄模式选项（装甲板 / 能量机关）
    uint8_t suggested_target_id;        // 建议的优先目标编号
    float current_velocity[3];          // 世界坐标系当前速度（m/s）
    float current_position[3];          // 世界坐标系当前位置（m）
    float orientation_quaternion[4];    // 世界坐标系朝向四元数 (w, x, y, z)
    PredictedState predicted[kPredictionPointCount];  // 未来 10 个点，相邻间隔 0.1s
    float suggested_big_yaw;            // 建议的大 yaw 方向（rad）
    uint8_t request_big_yaw_change;     // 是否强烈要求转换大 yaw 方向（0 / 1）
};

// ── 服务端信息 ──
// 时间戳均为 steady_clock 时间点换算成的秒数（double）
struct ServerInfo {
    double timestamp;                // 服务端当前时间戳
    double send_rate_hz;             // sendLoop 实际帧率（Hz，滑动窗口统计）
    double last_mcu_receive_time;    // 上一次接收到电控消息的时间戳
    double last_imu_receive_time;    // 上一次接收到 IMU 消息的时间戳
    double last_vision_write_time;   // 上一次视觉写入的时间戳
    double last_nav_write_time;      // 上一次导航写入的时间戳
};

// ── 视觉写入: 串口发送内容（火控 + 四个轴）与视觉向导航通信 ──
struct VisSerialWrite {
    FireControlData fire_control;
    AxisControlData axes;
};

struct VisWrite {
    VisSerialWrite serial;          // 写入串口发送内容部分
    VisToNav vision_to_nav;         // 视觉向导航通信部分
};

// ── 导航写入: 串口发送内容（变形与底盘位移）与导航向视觉通信 ──
struct NavSerialWrite {
    DeformChassisData deform_chassis;
};

struct NavWrite {
    NavSerialWrite serial;          // 写入串口发送内容部分
    NavToVision nav_to_vision;      // 导航向视觉通信部分
};

// ── 客户端读取到的当前全部信息（6 个结构体）──
struct AllInfo {
    SerialSendData          send_data;      // 1. 服务端要发送给串口的全部数据
    com::mcu::ReceivePacket mcu_data;       // 2. 串口收到的最新电控数据
    com::imu::ReceivePacket imu_data;       // 3. 串口收到的最新 IMU 数据
    VisToNav                vis_to_nav;     // 4. 视觉向导航通信内容
    NavToVision             nav_to_vision;  // 5. 导航向视觉通信内容
    ServerInfo              server_info;    // 6. 服务端信息
};

// 快照与写入内容都要能放进一条 IPC 通道
static_assert(sizeof(AllInfo) <= ipc::kMaxPayload, "AllInfo 超过 IPC 单通道载荷上限");
static_assert(sizeof(VisWrite) <= ipc::kMaxPayload, "VisWrite 超过 IPC 单条请求上限");
static_assert(sizeof(NavWrite) <= ipc::kMaxPayload, "NavWrite 超过 IPC 单条请求上限");
// kChannelCount 必须覆盖本文件使用的通道编号
static_assert(ipc::kChannelCount > kAllInfoChannel, "kChannelCount 未覆盖 AllInfo 获取通道");
static_assert(ipc::kChannelCount > kVisWriteChannel, "kChannelCount 未覆盖视觉写入通道");
static_assert(ipc::kChannelCount > kNavWriteChannel, "kChannelCount 未覆盖导航写入通道");

// steady_clock 时间点换算为秒数（double）
inline double steadySeconds(std::chrono::steady_clock::time_point time) {
    return std::chrono::duration<double>(time.time_since_epoch()).count();
}

}  // namespace serial_comm
}  // namespace s2c
