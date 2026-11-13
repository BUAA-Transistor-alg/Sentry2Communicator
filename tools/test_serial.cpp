// tools/test_serial.cpp — MCU 串口链路自检
//
// 打印每帧 MCU 反馈包的全部字段（四个关节的位置/速度/力矩/温度、底盘 IMU、状态位
// 与裁判系统 13 个命令码数据段）以及实际选中的串口（include/com/SerialProtocol.hpp
// 的 isOpen() / portName() 就是给本程序排查"到底选中/打开了哪个口"用的），
// 10ms 周期向电控发送一帧（自瞄开、四轴仅力矩、零目标、零力矩）。
//
// 运行: ./build/s2c_test_serial，Ctrl+C 退出

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

void printAxes(const com::mcu::ReceivePacket& packet) {
    std::cout << "  yaw_big:     pos=" << packet.yaw_big_position
              << " vel=" << packet.yaw_big_velocity
              << " torque=" << packet.yaw_big_torque
              << " temp=" << static_cast<int>(packet.yaw_big_temperature) << " C" << std::endl;
    std::cout << "  roll:        pos=" << packet.roll_position
              << " vel=" << packet.roll_velocity
              << " torque=" << packet.roll_torque
              << " temp=" << static_cast<int>(packet.roll_temperature) << " C" << std::endl;
    std::cout << "  yaw_small:   pos=" << packet.yaw_small_position
              << " vel=" << packet.yaw_small_velocity
              << " torque=" << packet.yaw_small_torque
              << " temp=" << static_cast<int>(packet.yaw_small_temperature) << " C" << std::endl;
    std::cout << "  pitch:       pos=" << packet.pitch_position
              << " vel=" << packet.pitch_velocity
              << " torque=" << packet.pitch_torque
              << " temp=" << static_cast<int>(packet.pitch_temperature) << " C" << std::endl;
}

void printReferee(const com::mcu::ReceivePacket& packet) {
    const com::referee::GameStatus& gs = packet.game_status;
    std::cout << "  [0x0001] game_type=" << static_cast<int>(gs.game_type)
              << " game_progress=" << static_cast<int>(gs.game_progress)
              << " stage_remain_time=" << gs.stage_remain_time
              << " SyncTimeStamp=" << gs.SyncTimeStamp << std::endl;
    const com::referee::GameRobotHp& hp = packet.game_robot_hp;
    std::cout << "  [0x0003] ally1=" << hp.ally_1_robot_HP
              << " ally3=" << hp.ally_3_robot_HP
              << " ally4=" << hp.ally_4_robot_HP
              << " damage_diff=" << hp.damage_difference
              << " ally7=" << hp.ally_7_robot_HP
              << " outpostR=" << hp.outpost_R_HP
              << " ally_base=" << hp.ally_base_HP
              << " outpostB=" << hp.outpost_B_HP
              << " enemy_base=" << hp.enemy_base_HP
              << " ally_control=" << hp.ally_control_degree
              << " enemy_control=" << hp.enemy_control_degree << std::endl;
    std::cout << "  [0x0101] event_data=0x" << std::hex << packet.event_data.event_data
              << " event_data_2=0x" << packet.event_data.event_data_2 << std::dec << std::endl;
    const com::referee::RefereeWarning& warning = packet.referee_warning;
    std::cout << "  [0x0104] level=" << static_cast<int>(warning.level)
              << " offending_robot_id=" << static_cast<int>(warning.offending_robot_id)
              << " count=" << static_cast<int>(warning.count) << std::endl;
    std::cout << "  [0x0105] dart_remaining_time="
              << static_cast<int>(packet.dart_info.dart_remaining_time)
              << " dart_info=" << packet.dart_info.dart_info << std::endl;
    const com::referee::RobotStatus& rs = packet.robot_status;
    std::cout << "  [0x0201] robot_id=" << static_cast<int>(rs.robot_id)
              << " level=" << static_cast<int>(rs.robot_level)
              << " HP=" << rs.current_HP << "/" << rs.maximum_HP
              << " cooling=" << rs.shooter_barrel_cooling_value
              << " heat_limit=" << rs.shooter_barrel_heat_limit
              << " power_limit=" << rs.chassis_power_limit
              << " bullet_speed_limit=" << rs.bullet_speed_limit
              << " power_out(gimbal,chassis,shooter)="
              << rs.power_management_gimbal_output
              << rs.power_management_chassis_output
              << rs.power_management_shooter_output << std::endl;
    const com::referee::PowerHeatData& ph = packet.power_heat_data;
    std::cout << "  [0x0202] buffer_energy=" << ph.buffer_energy
              << " heat_17mm=" << ph.shooter_17mm_barrel_heat
              << " heat_42mm=" << ph.shooter_42mm_barrel_heat << std::endl;
    std::cout << "  [0x0203] x=" << packet.robot_position.x
              << " y=" << packet.robot_position.y
              << " angle=" << packet.robot_position.angle << std::endl;
    const com::referee::Buff& buff = packet.buff;
    std::cout << "  [0x0204] recovery=" << static_cast<int>(buff.recovery_buff)
              << " cooling=" << buff.cooling_buff
              << " defence=" << static_cast<int>(buff.defence_buff)
              << " vulnerability=" << static_cast<int>(buff.vulnerability_buff)
              << " attack=" << buff.attack_buff << std::endl;
    const com::referee::HurtData& hurt = packet.hurt_data;
    std::cout << "  [0x0206] armor_id=" << static_cast<int>(hurt.armor_id)
              << " reason=" << static_cast<int>(hurt.HP_deduction_reason)
              << " hit_sequence=" << hurt.hit_sequence
              << " hit=(" << hurt.hit_position_x << "," << hurt.hit_position_y << ")" << std::endl;
    std::cout << "  [0x0209] rfid_status=0x" << std::hex << packet.rfid_status.rfid_status
              << " rfid_status_2=0x" << packet.rfid_status.rfid_status_2 << std::dec << std::endl;
    const com::referee::GroundRobotPosition& ground = packet.ground_robot_position;
    std::cout << "  [0x020B] heavyengineer=(" << ground.heavyengineer_x << ","
              << ground.heavyengineer_y << ")"
              << " standard_3=(" << ground.standard_3_x << "," << ground.standard_3_y << ")"
              << " standard_4=(" << ground.standard_4_x << "," << ground.standard_4_y << ")"
              << std::endl;
    const com::referee::SentryInfo& sentry = packet.sentry_info;
    std::cout << "  [0x020D] sentry_info=0x" << std::hex << sentry.sentry_info
              << " sentry_info_2=0x" << sentry.sentry_info_2
              << " sentry_info_3=0x" << sentry.sentry_info_3 << std::dec << std::endl;
}

void onReceive(const com::mcu::ReceivePacket& packet) {
    std::cout << "\n[Received Packet]" << std::endl;
    std::cout << "  frame_header1:      0x" << std::hex << std::uppercase
              << static_cast<int>(packet.frame_header1) << std::dec << std::endl;
    std::cout << "  frame_header2:      0x" << std::hex << std::uppercase
              << static_cast<int>(packet.frame_header2) << std::dec << std::endl;
    std::cout << "  protocol_version:   " << static_cast<int>(packet.protocol_version) << std::endl;
    std::cout << "  data_size:          " << static_cast<int>(packet.data_size) << std::endl;
    std::cout << "  bullet_velocity:    " << std::fixed << std::setprecision(2)
              << packet.bullet_velocity << " m/s" << std::endl;
    printAxes(packet);
    std::cout << "  chassis_imu_yaw:    " << packet.chassis_imu_yaw << std::endl;
    std::cout << "  chassis_imu_omega:  " << packet.chassis_imu_omega << " rad/s" << std::endl;
    std::cout << "  mark:               " << static_cast<int>(packet.mark) << std::endl;
    std::cout << "  color:              " << static_cast<int>(packet.color) << std::endl;
    std::cout << "  auto_aim_switch:    " << static_cast<int>(packet.auto_aim_switch) << std::endl;
    printReferee(packet);
    std::cout << "  crc8:               0x" << std::hex << std::uppercase
              << static_cast<int>(packet.crc8) << std::dec << std::endl;
    std::cout << std::endl;
}

}  // namespace
}  // namespace s2c

// main() 必须留在全局命名空间（否则不是程序入口）；
// 随后把 namespace s2c 内的名字引入作用域，便于 main() 直接使用。
using namespace s2c;

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Starting Serial Communication Test..." << std::endl;
    std::cout << "SendPacket size: " << sizeof(com::mcu::SendPacket) << " bytes" << std::endl;
    std::cout << "ReceivePacket size: " << sizeof(com::mcu::ReceivePacket) << " bytes" << std::endl;

    com::McuCommunication serial(onReceive);

    // 链路自检：构造即尝试选口/打开，这里把结果打出来（现场排查"为什么收不到"）
    std::cout << "Port open: " << (serial.isOpen() ? "yes" : "no")
              << ", port: " << (serial.portName().empty() ? "<none>" : serial.portName())
              << std::endl;

    std::cout << "Sending packets... (friction_wheel=1, fire=0, 四轴仅力矩、零目标、零力矩)" << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;

    int send_count = 0;
    while (keep_running) {
        com::mcu::SendPacket packet;
        // 默认初始化的 frame_header1/2、protocol_version、data_size 已自动设置
        packet.friction_wheel_enable = 1;
        packet.fire = 0;
        // ── 大 yaw：仅力矩模式、目标位置/速度不参与 ──
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
        // ── 变形与底盘位移 ──
        packet.deform_control = 0;
        packet.displacement_target_velocity_x = 0.0f;
        packet.displacement_target_velocity_y = 0.0f;

        if (serial.sendData(packet)) {
            send_count++;
            if (send_count % 100 == 0) {
                std::cout << "Sent " << send_count << " packets..." << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nExiting... Total packets sent: " << send_count << std::endl;

    return 0;
}
