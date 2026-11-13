// SerialProtocol.hpp — 串口协议包定义（MCU 电控包、IMU 包与裁判系统数据段）
//
// 内容由串口通信服务端（主工程）与串口通信客户端（本工程）共用；
// 数据段与字段偏移的编译期检查在本文件末尾。
#pragma once
#include <cstdint>
#include <cstddef>

namespace s2c {
namespace com {

// ============================================================================
// 裁判系统串口协议数据段（RoboMaster 2027 高校系列赛-通信协议 V1.0.0）
//
// 本命名空间只定义各命令码的 data 段（不含 frame_header / cmd_id / frame_tail），
// 由 mcu::ReceivePacket 原样携带。多字节字段为小端，位域按 GCC/Clang 与官方 SDK
// 的定义（低位在前）。
// ============================================================================
namespace referee {

#pragma pack(push, 1)

// 0x0001 比赛状态数据（11 字节，1Hz）
struct GameStatus {
    uint8_t  game_type : 4;         // 比赛类型：1 超级对抗赛 / 2 高校单项赛 / 3 ICRA / 4 高校联盟赛 3V3 / 5 高校联盟赛步兵对抗
    uint8_t  game_progress : 4;     // 当前比赛阶段：0 未开始 / 1 准备 / 2 十五秒自检 / 3 五秒倒计时 / 4 比赛中 / 5 结算中
    uint16_t stage_remain_time;     // 当前阶段剩余时间（秒）
    uint64_t SyncTimeStamp;         // UNIX 时间（NTP 同步后生效）
};

// 0x0003 机器人血量数据（24 字节，3Hz）
struct GameRobotHp {
    uint16_t ally_1_robot_HP;       // 己方 1 号重装机器人血量
    uint16_t reserved_0;            // 保留位
    uint16_t ally_3_robot_HP;       // 己方 3 号步兵机器人血量
    uint16_t ally_4_robot_HP;       // 己方 4 号步兵机器人血量
    int16_t  damage_difference;     // 己方全队总伤害与对方全队总伤害之差
    uint16_t ally_7_robot_HP;       // 己方 7 号哨兵机器人血量
    uint16_t outpost_R_HP;          // R 前哨血量
    uint16_t ally_base_HP;          // 己方基地血量
    uint16_t outpost_B_HP;          // B 前哨血量
    uint16_t enemy_base_HP;         // 对方基地血量
    uint16_t ally_control_degree;   // 己方累计控制度
    uint16_t enemy_control_degree;  // 对方累计控制度
};

// 0x0101 场地事件数据（8 字节，1Hz）
struct EventData {
    uint32_t event_data;            // 己方补给区/能量机关/堡垒/基地/制高点等激活状态与阵地控制状态
    uint32_t event_data_2;          // R/B 基地阵地控制、飞镖命中记录、RMUL 高地控制区等
};

// 0x0104 裁判警告数据（3 字节，己方判罚/判负时触发，其余时间 1Hz）
struct RefereeWarning {
    uint8_t level;                  // 最后一次判罚等级：1 双方黄牌 / 2 黄牌 / 3 红牌 / 4 判负
    uint8_t offending_robot_id;     // 违规机器人 ID（判负与双方黄牌时为 0）
    uint8_t count;                  // 该判罚等级的违规次数
};

// 0x0105 飞镖发射相关数据（3 字节，1Hz）
struct DartInfo {
    uint8_t  dart_remaining_time;   // 己方飞镖发射剩余时间（秒）
    uint16_t dart_info;             // bit 0-2 最近命中目标 / bit 3-5 对方被命中累计次数 / bit 6-8 当前选定击打目标 / bit 9-15 保留
};

// 0x0201 机器人性能体系数据（17 字节，10Hz）
struct RobotStatus {
    uint8_t robot_id;                       // 本机器人 ID
    uint8_t robot_level;                    // 机器人等级
    uint16_t current_HP;                    // 当前血量
    uint16_t maximum_HP;                    // 血量上限
    uint16_t shooter_barrel_cooling_value;  // 射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;     // 射击热量上限
    uint16_t chassis_power_limit;           // 底盘功率上限
    float    bullet_speed_limit;            // 射击初速度上限（m/s）
    uint8_t  power_management_gimbal_output : 1;   // gimbal 口输出（0 无输出 / 1 24V）
    uint8_t  power_management_chassis_output : 1;  // chassis 口输出
    uint8_t  power_management_shooter_output : 1;  // shooter 口输出
};

// 0x0202 实时底盘缓冲能量和射击热量数据（14 字节，10Hz）
struct PowerHeatData {
    uint16_t reserved_0;                // 保留位
    uint16_t reserved_1;                // 保留位
    float    reserved_2;                // 保留位
    uint16_t buffer_energy;             // 缓冲能量（J）
    uint16_t shooter_17mm_barrel_heat;  // 17mm 发射机构射击热量
    uint16_t shooter_42mm_barrel_heat;  // 42mm 发射机构射击热量
};

// 0x0203 机器人位置数据（12 字节，1Hz）
struct RobotPosition {
    float x;                        // 位置 x 坐标（m）
    float y;                        // 位置 y 坐标（m）
    float angle;                    // 测速模块朝向（度，正北为 0 度）
};

// 0x0204 机器人增益数据（7 字节，3Hz）
struct Buff {
    uint8_t  recovery_buff;         // 回血增益（百分比）
    uint16_t cooling_buff;          // 射击热量冷却增益（直接值，x/s）
    uint8_t  defence_buff;          // 防御增益（百分比）
    uint8_t  vulnerability_buff;    // 负防御增益（百分比）
    uint16_t attack_buff;           // 攻击增益（百分比）
};

// 0x0206 伤害状态数据（7 字节，伤害发生后发送）
struct HurtData {
    uint8_t  armor_id : 4;            // 装甲模块或测速模块 ID（其他扣血原因时为 0）
    uint8_t  HP_deduction_reason : 4; // 血量变化类型：0 弹丸攻击 / 1 装甲离线 / 2 超级电容离线 / 5 装甲受到撞击
    uint16_t hit_sequence;            // 命中事件序号，每发命中递增 1
    int16_t  hit_position_x;          // 命中位置 x（mm，装甲检测面左下角为原点）
    int16_t  hit_position_y;          // 命中位置 y（mm）
};

// 0x0209 机器人 RFID 模块状态（6 字节，3Hz）
struct RfidStatus {
    uint32_t rfid_status;           // 各增益点 RFID 卡检测状态（bit 0-31）
    uint16_t rfid_status_2;         // 其余增益点 RFID 卡检测状态（bit 0-8）
};

// 0x020B 地面机器人位置数据（40 字节，1Hz，仅哨兵）
struct GroundRobotPosition {
    float heavyengineer_x;          // 己方重装机器人位置 x（m）
    float heavyengineer_y;          // 己方重装机器人位置 y（m）
    float reserved_0;               // 保留位
    float reserved_1;               // 保留位
    float standard_3_x;             // 己方 3 号步兵机器人位置 x（m）
    float standard_3_y;             // 己方 3 号步兵机器人位置 y（m）
    float standard_4_x;             // 己方 4 号步兵机器人位置 x（m）
    float standard_4_y;             // 己方 4 号步兵机器人位置 y（m）
    float reserved_2;               // 保留位
    float reserved_3;               // 保留位
};

// 0x020D 哨兵自主决策信息同步（14 字节，1Hz，仅哨兵）
struct SentryInfo {
    uint32_t sentry_info;           // bit 0-10 已兑换发弹量 / bit 11-14 远程兑换发弹量次数 / bit 15-18 远程兑换血量次数
                                    // bit 19 免费复活可确认 / bit 20 可兑换立即复活 / bit 21-30 立即复活所需金币 / bit 31 保留
    uint16_t sentry_info_2;         // bit 0 脱战状态 / bit 1-11 17mm 剩余可兑换量 / bit 12-13 当前姿态 / bit 14 能量机关可激活 / bit 15 强化姿态
    uint64_t sentry_info_3;         // bit 0-7 进攻姿态弱化剩余秒数 / bit 8-15 防御姿态 / bit 16-23 移动姿态
                                    // bit 24-31 保留 / bit 32-39 强化进攻姿态 / bit 40-47 强化防御姿态 / bit 48-55 强化移动姿态 / bit 56-63 保留
};

#pragma pack(pop)

static_assert(sizeof(GameStatus) == 11, "0x0001 数据段长度应为 11");
static_assert(sizeof(GameRobotHp) == 24, "0x0003 数据段长度应为 24");
static_assert(sizeof(EventData) == 8, "0x0101 数据段长度应为 8");
static_assert(sizeof(RefereeWarning) == 3, "0x0104 数据段长度应为 3");
static_assert(sizeof(DartInfo) == 3, "0x0105 数据段长度应为 3");
static_assert(sizeof(RobotStatus) == 17, "0x0201 数据段长度应为 17");
static_assert(sizeof(PowerHeatData) == 14, "0x0202 数据段长度应为 14");
static_assert(sizeof(RobotPosition) == 12, "0x0203 数据段长度应为 12");
static_assert(sizeof(Buff) == 7, "0x0204 数据段长度应为 7");
static_assert(sizeof(HurtData) == 7, "0x0206 数据段长度应为 7");
static_assert(sizeof(RfidStatus) == 6, "0x0209 数据段长度应为 6");
static_assert(sizeof(GroundRobotPosition) == 40, "0x020B 数据段长度应为 40");
static_assert(sizeof(SentryInfo) == 14, "0x020D 数据段长度应为 14");

// 各数据段内部字段偏移（对应协议表格的字节偏移量，位域成员不能取 offsetof，
// 其起始偏移由所在数据段长度与前序字段偏移共同确定）
static_assert(offsetof(GameStatus, stage_remain_time) == 1, "0x0001 偏移变化");
static_assert(offsetof(GameStatus, SyncTimeStamp) == 3, "0x0001 偏移变化");
static_assert(offsetof(GameRobotHp, ally_3_robot_HP) == 4, "0x0003 偏移变化");
static_assert(offsetof(GameRobotHp, damage_difference) == 8, "0x0003 偏移变化");
static_assert(offsetof(GameRobotHp, enemy_control_degree) == 22, "0x0003 偏移变化");
static_assert(offsetof(EventData, event_data_2) == 4, "0x0101 偏移变化");
static_assert(offsetof(RefereeWarning, offending_robot_id) == 1, "0x0104 偏移变化");
static_assert(offsetof(DartInfo, dart_info) == 1, "0x0105 偏移变化");
static_assert(offsetof(RobotStatus, current_HP) == 2, "0x0201 偏移变化");
static_assert(offsetof(RobotStatus, chassis_power_limit) == 10, "0x0201 偏移变化");
static_assert(offsetof(RobotStatus, bullet_speed_limit) == 12, "0x0201 偏移变化");
static_assert(offsetof(PowerHeatData, buffer_energy) == 8, "0x0202 偏移变化");
static_assert(offsetof(RobotPosition, y) == 4, "0x0203 偏移变化");
static_assert(offsetof(RobotPosition, angle) == 8, "0x0203 偏移变化");
static_assert(offsetof(Buff, cooling_buff) == 1, "0x0204 偏移变化");
static_assert(offsetof(Buff, attack_buff) == 5, "0x0204 偏移变化");
static_assert(offsetof(HurtData, hit_sequence) == 1, "0x0206 偏移变化");
static_assert(offsetof(HurtData, hit_position_y) == 5, "0x0206 偏移变化");
static_assert(offsetof(RfidStatus, rfid_status_2) == 4, "0x0209 偏移变化");
static_assert(offsetof(GroundRobotPosition, standard_3_x) == 16, "0x020B 偏移变化");
static_assert(offsetof(GroundRobotPosition, standard_4_x) == 24, "0x020B 偏移变化");
static_assert(offsetof(SentryInfo, sentry_info_2) == 4, "0x020D 偏移变化");
static_assert(offsetof(SentryInfo, sentry_info_3) == 6, "0x020D 偏移变化");

} // namespace referee


// ============================================================================
// MCU（电控）通信协议 —— 帧格式: [前导 3B][data_size 1B][payload data_size B][CRC8 1B]
// 前导: 0x42 0x52 0x04
//
// 设计要点:
//   - 四个关节（大 yaw、roll、小 yaw、pitch）各自独立的模式位、目标位置、目标速度、力矩
//   - 位置: 大 yaw 为 double（多圈连续语义，不限位），其余为 float（关节系弧度）
//   - 模式位: 1 = 仅力矩、2 = 力矩 + 内环；其余任何取值（含 0）一律视为**非法模式**
//     ⇒ 力矩按 0 处理（保守，见电控端实现）
//   - 变形控制位（uint8_t）与底盘位移目标速度（x、y）是哨兵专有通道
//   - pitch/roll 的"编码器/指令 → 角度"线性映射由上位机 McuDataPreprocessor 完成；
//     yaw 轴的角度换算（编码器计数 → 弧度、多圈累计）由电控完成
//   - 接收包携带四个关节的位置、速度、实际使用的控制力矩与电机温度，
//     并携带裁判系统 13 个命令码的完整数据段（见 referee 命名空间）
//   - **反馈通道的更新率并不一致**:
//       * 小 yaw / roll / pitch 编码器（MCU1 直控）: 每帧都是新值（实时可信）
//       * 大 yaw 反馈、底盘 IMU（MCU2）: **更新率低且间隔不规则**；两次更新之间
//         MCU1 会**重复发送上一次的值**（值被保持），上层需自行识别新样本。
// ============================================================================
namespace mcu {

// 帧同步前导字节数（frame_header1 + frame_header2 + protocol_version）
constexpr size_t PREAMBLE_SIZE = 3;

// 协议版本
constexpr uint8_t PROTOCOL_VERSION = 0x04;

// ── 关节控制模式 ──
enum AxisMode : uint8_t {
    AXIS_MODE_TORQUE_ONLY     = 1, // 仅力矩: 电控直接施加 *_torque
    AXIS_MODE_TORQUE_PLUS_PID = 2, // 力矩 + 位置/速度内环: τ = kp(θ*−θ) + kd(ω*−ω) + *_torque
};

#pragma pack(push, 1)
struct SendPacket
{
    uint8_t frame_header1 = 0x42;
    uint8_t frame_header2 = 0x52;
    uint8_t protocol_version = PROTOCOL_VERSION;
    uint8_t data_size = 67;             // 1+1 + 1+4+4 + (1+8+4+4) + 3×(1+4+4+4)
    // ── 火控 ──
    uint8_t friction_wheel_enable;      // 摩擦轮开关
    uint8_t fire;                       // 火控
    // ── 变形与底盘位移 ──
    uint8_t deform_control;                     // 变形控制位
    float   displacement_target_velocity_x;     // 底盘位移目标速度 x（m/s）
    float   displacement_target_velocity_y;     // 底盘位移目标速度 y（m/s）
    // ── 大 yaw ──
    uint8_t yaw_big_mode;               // AxisMode
    double  yaw_big_target_position;    // rad，多圈连续
    float   yaw_big_target_velocity;    // rad/s
    float   yaw_big_torque;             // N·m（前馈/纯力矩）
    // ── roll ──
    uint8_t roll_mode;                  // AxisMode
    float   roll_target_position;       // rad
    float   roll_target_velocity;       // rad/s
    float   roll_torque;                // N·m
    // ── 小 yaw ──
    uint8_t yaw_small_mode;             // AxisMode
    float   yaw_small_target_position;  // rad
    float   yaw_small_target_velocity;  // rad/s
    float   yaw_small_torque;           // N·m
    // ── pitch ──
    uint8_t pitch_mode;                 // AxisMode
    float   pitch_target_position;      // rad
    float   pitch_target_velocity;      // rad/s
    float   pitch_torque;               // N·m
    uint8_t crc8;
};

struct ReceivePacket
{
    uint8_t frame_header1 = 0x42;
    uint8_t frame_header2 = 0x52;
    uint8_t protocol_version = PROTOCOL_VERSION;
    uint8_t data_size = 237;
    // ── 弹速/火控相关信息 ──
    float   bullet_velocity;            // m/s
    // ── 四个关节：位置 / 速度 / 实际使用的控制力矩 / 电机温度 ──
    double  yaw_big_position;           // rad，多圈连续
    float   yaw_big_velocity;           // rad/s
    float   yaw_big_torque;             // N·m（实际使用的控制力矩）
    uint8_t yaw_big_temperature;        // ℃
    float   roll_position;              // rad
    float   roll_velocity;              // rad/s
    float   roll_torque;                // N·m
    uint8_t roll_temperature;           // ℃
    float   yaw_small_position;         // rad
    float   yaw_small_velocity;         // rad/s
    float   yaw_small_torque;           // N·m
    uint8_t yaw_small_temperature;      // ℃
    float   pitch_position;             // rad
    float   pitch_velocity;             // rad/s
    float   pitch_torque;               // N·m
    uint8_t pitch_temperature;          // ℃
    // ── 底盘 IMU（底盘自身姿态，用于底盘旋转补偿）──
    float   chassis_imu_yaw;            // rad
    float   chassis_imu_omega;          // rad/s
    // ── 状态 ──
    uint8_t mark;                       // 递增标志位
    uint8_t color;                      // 颜色
    uint8_t auto_aim_switch;            // 电控自瞄开关
    // ── 裁判系统数据段（完整数据段，见 referee 命名空间）──
    referee::GameStatus            game_status;             // 0x0001
    referee::GameRobotHp           game_robot_hp;           // 0x0003
    referee::EventData             event_data;              // 0x0101
    referee::RefereeWarning        referee_warning;         // 0x0104
    referee::DartInfo              dart_info;               // 0x0105
    referee::RobotStatus           robot_status;            // 0x0201
    referee::PowerHeatData         power_heat_data;         // 0x0202
    referee::RobotPosition         robot_position;          // 0x0203
    referee::Buff                  buff;                    // 0x0204
    referee::HurtData              hurt_data;               // 0x0206
    referee::RfidStatus            rfid_status;             // 0x0209
    referee::GroundRobotPosition   ground_robot_position;   // 0x020B
    referee::SentryInfo            sentry_info;             // 0x020D
    uint8_t crc8;
};
#pragma pack(pop)

// 编译期一致性检查（payload 长度必须与 data_size 一致）
static_assert(sizeof(SendPacket) == 3 + 1 + 67 + 1,
              "SendPacket 布局与 data_size 不一致");
static_assert(sizeof(ReceivePacket) == 3 + 1 + 237 + 1,
              "ReceivePacket 布局与 data_size 不一致");

// 发送包关键字段偏移（电控侧应逐字段核对，避免静默错位）
static_assert(offsetof(SendPacket, friction_wheel_enable) == 4, "偏移变化");
static_assert(offsetof(SendPacket, deform_control) == 6, "偏移变化");
static_assert(offsetof(SendPacket, displacement_target_velocity_x) == 7, "偏移变化");
static_assert(offsetof(SendPacket, displacement_target_velocity_y) == 11, "偏移变化");
static_assert(offsetof(SendPacket, yaw_big_mode) == 15, "偏移变化");
static_assert(offsetof(SendPacket, yaw_big_target_position) == 16, "偏移变化");
static_assert(offsetof(SendPacket, roll_mode) == 32, "偏移变化");
static_assert(offsetof(SendPacket, yaw_small_mode) == 45, "偏移变化");
static_assert(offsetof(SendPacket, pitch_mode) == 58, "偏移变化");
static_assert(offsetof(SendPacket, crc8) == 71, "偏移变化");

// 接收包关键字段偏移（电控侧应逐字段核对，避免静默错位）
static_assert(offsetof(ReceivePacket, bullet_velocity) == 4, "偏移变化");
static_assert(offsetof(ReceivePacket, yaw_big_position) == 8, "偏移变化");
static_assert(offsetof(ReceivePacket, yaw_big_temperature) == 24, "偏移变化");
static_assert(offsetof(ReceivePacket, roll_position) == 25, "偏移变化");
static_assert(offsetof(ReceivePacket, yaw_small_position) == 38, "偏移变化");
static_assert(offsetof(ReceivePacket, pitch_position) == 51, "偏移变化");
static_assert(offsetof(ReceivePacket, chassis_imu_yaw) == 64, "偏移变化");
static_assert(offsetof(ReceivePacket, mark) == 72, "偏移变化");
static_assert(offsetof(ReceivePacket, game_status) == 75, "偏移变化");
static_assert(offsetof(ReceivePacket, game_robot_hp) == 86, "偏移变化");
static_assert(offsetof(ReceivePacket, event_data) == 110, "偏移变化");
static_assert(offsetof(ReceivePacket, referee_warning) == 118, "偏移变化");
static_assert(offsetof(ReceivePacket, dart_info) == 121, "偏移变化");
static_assert(offsetof(ReceivePacket, robot_status) == 124, "偏移变化");
static_assert(offsetof(ReceivePacket, power_heat_data) == 141, "偏移变化");
static_assert(offsetof(ReceivePacket, robot_position) == 155, "偏移变化");
static_assert(offsetof(ReceivePacket, buff) == 167, "偏移变化");
static_assert(offsetof(ReceivePacket, hurt_data) == 174, "偏移变化");
static_assert(offsetof(ReceivePacket, rfid_status) == 181, "偏移变化");
static_assert(offsetof(ReceivePacket, ground_robot_position) == 187, "偏移变化");
static_assert(offsetof(ReceivePacket, sentry_info) == 227, "偏移变化");
static_assert(offsetof(ReceivePacket, crc8) == 241, "偏移变化");

} // namespace mcu


// ============================================================================
// IMU 通信协议（与大 yaw 转子固连的 IMU）
// IMU 安装位置: 固定在大 yaw 转子上，其欧拉角与陀螺给出该平台的姿态与角速度。
// ============================================================================
namespace imu {

constexpr size_t PREAMBLE_SIZE = 3;

#pragma pack(push, 1)
struct SendPacket
{
    uint8_t frame_header1 = 0xA7;
    uint8_t frame_header2 = 0xB6;
    uint8_t frame_header3 = 0xC5;
    uint8_t data_size = 0;              // 无数据载荷，仅心跳
    uint32_t crc32;
};

struct ReceivePacket
{
    uint8_t frame_header1 = 0xA7;
    uint8_t frame_header2 = 0xB6;
    uint8_t frame_header3 = 0xC5;
    uint8_t data_size;
    float gx;                           // IMU 本体系角速度 (rad/s)
    float gy;
    float gz;
    float ax;                           // 加速度 (m/s²)
    float ay;
    float az;
    double euler_yaw;                   // 世界系欧拉角 (rad)，ZXY 约定
    double euler_pitch;
    double euler_roll;
    uint32_t dt_one_tenth_ms;           // 本帧间隔（0.1ms 单位）
    uint32_t crc32;
};
#pragma pack(pop)

} // namespace imu

}  // namespace com
}  // namespace s2c
