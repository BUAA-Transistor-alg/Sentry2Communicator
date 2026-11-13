// ipc_client/examples/example.cpp — SerialCommClient 使用示例
//
// 串口通信服务端在主工程里（s2c::serial_comm::SerialCommServer），按 500Hz 收发串口并
// 发布完整快照。本示例演示客户端视觉部分与导航部分的读取与写入；服务端未启动时
// 所有调用返回 false，程序照常结束。
#include "SerialCommClient.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

namespace {

using s2c::serial_comm::AllInfo;
using s2c::serial_comm::AutoAimMode;
using s2c::serial_comm::NavWrite;
using s2c::serial_comm::SerialCommClient;
using s2c::serial_comm::VisWrite;

// 视觉写入: 火控、四个轴的控制部分、视觉向导航通信
VisWrite makeVisWrite(int iteration) {
    VisWrite write{};
    write.serial.fire_control.friction_wheel_enable = 1;
    write.serial.fire_control.fire = (iteration % 10 == 0) ? 1 : 0;

    write.serial.axes.yaw_big_mode = s2c::com::mcu::AXIS_MODE_TORQUE_ONLY;
    write.serial.axes.yaw_big_target_position = 0.01 * static_cast<double>(iteration);
    write.serial.axes.yaw_big_target_velocity = 0.0f;
    write.serial.axes.yaw_big_torque = 0.0f;
    write.serial.axes.roll_mode = s2c::com::mcu::AXIS_MODE_TORQUE_ONLY;
    write.serial.axes.roll_target_position = 0.0f;
    write.serial.axes.roll_target_velocity = 0.0f;
    write.serial.axes.roll_torque = 0.0f;
    write.serial.axes.yaw_small_mode = s2c::com::mcu::AXIS_MODE_TORQUE_ONLY;
    write.serial.axes.yaw_small_target_position = 0.0f;
    write.serial.axes.yaw_small_target_velocity = 0.0f;
    write.serial.axes.yaw_small_torque = 0.0f;
    write.serial.axes.pitch_mode = s2c::com::mcu::AXIS_MODE_TORQUE_ONLY;
    write.serial.axes.pitch_target_position = 0.0f;
    write.serial.axes.pitch_target_velocity = 0.0f;
    write.serial.axes.pitch_torque = 0.0f;

    for (uint32_t i = 0; i < s2c::serial_comm::kTargetCount; ++i) {
        s2c::serial_comm::TargetInfo& target = write.vision_to_nav.targets[i];
        target.recognized = (i == 0) ? 1 : 0;
        target.enemy_x = static_cast<float>(i) + 0.5f;
        target.enemy_y = -1.0f - static_cast<float>(i);
        target.enemy_z = 0.25f;
    }
    write.vision_to_nav.auto_aim_state = static_cast<uint8_t>(iteration % 4);
    write.vision_to_nav.target_id = 0;
    return write;
}

// 导航写入: 变形与底盘位移、导航向视觉通信
NavWrite makeNavWrite(int iteration) {
    NavWrite write{};
    write.serial.deform_chassis.deform_control = 0;
    write.serial.deform_chassis.displacement_target_velocity_x =
        0.1f * static_cast<float>(iteration % 5);
    write.serial.deform_chassis.displacement_target_velocity_y = -0.05f;

    write.nav_to_vision.auto_aim_mode = AutoAimMode::kArmor;
    write.nav_to_vision.suggested_target_id = 1;
    write.nav_to_vision.current_velocity[0] = 0.1f;
    write.nav_to_vision.current_velocity[1] = -0.2f;
    write.nav_to_vision.current_velocity[2] = 0.0f;
    write.nav_to_vision.current_position[0] = 1.0f;
    write.nav_to_vision.current_position[1] = 2.0f;
    write.nav_to_vision.current_position[2] = 0.0f;
    write.nav_to_vision.orientation_quaternion[0] = 1.0f;
    write.nav_to_vision.orientation_quaternion[1] = 0.0f;
    write.nav_to_vision.orientation_quaternion[2] = 0.0f;
    write.nav_to_vision.orientation_quaternion[3] = 0.0f;
    for (uint32_t i = 0; i < s2c::serial_comm::kPredictionPointCount; ++i) {
        s2c::serial_comm::PredictedState& point = write.nav_to_vision.predicted[i];
        point.position[0] = 1.0f + 0.1f * static_cast<float>(i);
        point.position[1] = 2.0f;
        point.position[2] = 0.0f;
        point.velocity[0] = 0.1f;
        point.velocity[1] = 0.0f;
        point.velocity[2] = 0.0f;
    }
    write.nav_to_vision.suggested_big_yaw = 0.5f;
    write.nav_to_vision.request_big_yaw_change = 0;
    return write;
}

void printInfo(const AllInfo& info) {
    const AllInfo& data = info;
    std::printf("  send: friction=%u fire=%u deform=0x%02X yaw_big=%.3f pitch=%.3f\n",
                data.send_data.fire_control.friction_wheel_enable,
                data.send_data.fire_control.fire,
                data.send_data.deform_chassis.deform_control,
                data.send_data.axes.yaw_big_target_position,
                static_cast<double>(data.send_data.axes.pitch_target_position));
    std::printf("  mcu: mark=%u pitch=%.3f | imu: dt=%u\n",
                data.mcu_data.mark,
                static_cast<double>(data.mcu_data.pitch_position),
                data.imu_data.dt_one_tenth_ms);
    std::printf("  vis_to_nav[0]: recognized=%u enemy=(%.2f, %.2f, %.2f) state=%u target=%u\n",
                data.vis_to_nav.targets[0].recognized,
                static_cast<double>(data.vis_to_nav.targets[0].enemy_x),
                static_cast<double>(data.vis_to_nav.targets[0].enemy_y),
                static_cast<double>(data.vis_to_nav.targets[0].enemy_z),
                data.vis_to_nav.auto_aim_state,
                data.vis_to_nav.target_id);
    std::printf("  nav_to_vision: mode=%u suggested=%u big_yaw=%.3f strong=%u\n",
                static_cast<unsigned>(data.nav_to_vision.auto_aim_mode),
                data.nav_to_vision.suggested_target_id,
                static_cast<double>(data.nav_to_vision.suggested_big_yaw),
                data.nav_to_vision.request_big_yaw_change);
    std::printf("  server: now=%.3f rate=%.1fHz mcu=%.3f imu=%.3f vis=%.3f nav=%.3f\n",
                data.server_info.timestamp,
                data.server_info.send_rate_hz,
                data.server_info.last_mcu_receive_time,
                data.server_info.last_imu_receive_time,
                data.server_info.last_vision_write_time,
                data.server_info.last_nav_write_time);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string name = argc > 1 ? argv[1] : s2c::serial_comm::kDefaultShmName;
    SerialCommClient client(name);

    for (int i = 0; i < 20; ++i) {
        // 视觉部分: 写火控、四个轴与视觉向导航通信
        if (!client.vis_write(makeVisWrite(i))) {
            std::printf("[example] 视觉写入失败（串口通信服务端未就绪）\n");
        }
        // 导航部分: 写变形与底盘位移与导航向视觉通信
        if (!client.nav_write(makeNavWrite(i))) {
            std::printf("[example] 导航写入失败（串口通信服务端未就绪）\n");
        }

        // 两部分读取同一份完整快照
        AllInfo info{};
        if (client.vis_get_all_info(info)) {
            std::printf("[example] 视觉读取（第 %d 次）\n", i);
            printInfo(info);
        }
        if (client.nav_get_all_info(info)) {
            std::printf("[example] 导航读取（第 %d 次）\n", i);
            printInfo(info);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return 0;
}
