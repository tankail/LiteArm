/**
 * @file test_motor_enable.cpp
 * @brief 测试电机是否能正常驱动 - 7关节版本
 */

#include "litearm_robot/LiteArm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    try {
        std::string config_path;
        if (argc > 1) {
            config_path = argv[1];
        } else {
            config_path = ament_index_cpp::get_package_share_directory("litearm_config")
                         + "/robot_param/litearm_right_arm.yaml";
        }

        std::cout << "使用配置文件: " << config_path << std::endl;
        litearm_robot::LiteArm robot(config_path);

        int motor_count = robot.getMotorCount();
        std::cout << "电机数量: " << motor_count << std::endl;

        // 读取当前电机位置
        robot.send_get_motor_state_cmd();
        robot.motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto current_pos = robot.getCurrentPos();
        std::cout << "当前电机位置: [";
        for (int i = 0; i < motor_count; i++) {
            std::cout << current_pos[i];
            if (i < motor_count - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // 打印电机模式和错误状态
        std::cout << "电机状态信息:" << std::endl;
        for (int i = 0; i < motor_count && i < 4; i++) {  // 只打印前4个有效的
            auto state = robot.Motors[i]->get_current_motor_state();
            std::cout << "  Motor " << (i+1) << ": mode=" << (int)state->mode
                      << ", fault=" << (int)state->fault
                      << ", pos=" << state->position << std::endl;
        }

        // 使用当前位置作为起始点（过滤掉无效位置）
        std::vector<double> start_pos;
        for (int i = 0; i < motor_count; i++) {
            if (current_pos[i] < 100.0) {
                start_pos.push_back(current_pos[i]);
            } else {
                std::cout << "警告: Joint " << (i+1) << " 位置无效 (" << current_pos[i]
                          << ")，使用0代替" << std::endl;
                start_pos.push_back(0.0);
            }
        }

        // 定义力矩参数（在后续代码之前）
        std::vector<double> vel(motor_count, 0.5);
        std::vector<double> max_torque = {21.0, 36.0, 36.0, 21.0, 10.0, 10.0, 10.0};

        // !!! 关键修改：为所有8个电机（包括夹爪）设置命令，而不仅是7个
        std::cout << "注意：将控制所有8个电机（包括夹爪）以确保CAN数据格式正确" << std::endl;
        int total_motors = robot.Motors.size();
        std::vector<double> full_start_pos(total_motors);
        std::vector<double> full_vel(total_motors);
        std::vector<double> full_max_torque(total_motors);

        // 复制前7个关节的位置
        for (int i = 0; i < motor_count && i < total_motors; i++) {
            full_start_pos[i] = start_pos[i];
            full_vel[i] = 0.5;
            full_max_torque[i] = max_torque[i];
        }
        // 为夹爪设置合理的默认值
        if (total_motors > motor_count) {
            full_start_pos[motor_count] = 0.0;  // 夹爪位置
            full_vel[motor_count] = 0.5;
            full_max_torque[motor_count] = 0.5;  // 夹爪力矩较小
            std::cout << "夹爪电机 (ID=" << robot.Motors[motor_count]->get_motor_id()
                      << ") 设置为位置0, 速度0.5, 力矩0.5" << std::endl;
        }

        // 测试1: 直接发送当前位置命令（不调用set_stop）
        std::cout << "\n=== 测试1: 发送当前位置命令（使能电机） ===" << std::endl;
        // 使用完整参数控制所有电机
        for (int i = 0; i < total_motors; i++) {
            robot.Motors[i]->pos_vel_MAXtqe(full_start_pos[i], full_vel[i], full_max_torque[i]);
        }
        robot.motor_send_cmd();
        std::cout << "命令已发送，等待2秒..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 读取状态
        robot.send_get_motor_state_cmd();
        robot.motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        current_pos = robot.getCurrentPos();
        std::cout << "当前位置: [";
        for (int i = 0; i < motor_count; i++) {
            std::cout << current_pos[i];
            if (i < motor_count - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // 测试2: 移动 joint1 一个小角度
        std::cout << "\n=== 测试2: 移动 joint1 +0.2 rad ===" << std::endl;
        std::vector<double> full_target_pos = full_start_pos;
        full_target_pos[0] += 0.2;  // 只移动 joint1

        // 持续发送命令 3 秒
        for (int i = 0; i < 300; i++) {
            for (int j = 0; j < total_motors; j++) {
                robot.Motors[j]->pos_vel_MAXtqe(full_target_pos[j], full_vel[j], full_max_torque[j]);
            }
            robot.motor_send_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // 读取最终位置
        robot.send_get_motor_state_cmd();
        robot.motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        current_pos = robot.getCurrentPos();
        std::cout << "最终位置: [";
        for (int i = 0; i < motor_count; i++) {
            std::cout << current_pos[i];
            if (i < motor_count - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // 显示最终电机状态
        std::cout << "测试2后的电机状态:" << std::endl;
        for (int i = 0; i < motor_count && i < 4; i++) {
            auto state = robot.Motors[i]->get_current_motor_state();
            std::cout << "  Motor " << (i+1) << ": mode=" << (int)state->mode
                      << ", fault=" << (int)state->fault
                      << ", pos=" << state->position << std::endl;
        }

        if (std::abs(current_pos[0] - full_target_pos[0]) < 0.1) {
            std::cout << "\n成功! 电机已经移动到目标位置!" << std::endl;
        } else {
            std::cout << "\n失败! 电机没有移动到目标位置。" << std::endl;
            std::cout << "目标: " << full_target_pos[0] << ", 实际: " << current_pos[0] << std::endl;
        }

        // 回到起始位置
        std::cout << "\n=== 返回起始位置 ===" << std::endl;
        for (int i = 0; i < 300; i++) {
            for (int j = 0; j < total_motors; j++) {
                robot.Motors[j]->pos_vel_MAXtqe(full_start_pos[j], full_vel[j], full_max_torque[j]);
            }
            robot.motor_send_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "测试完成，电机将自动掉电。" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
