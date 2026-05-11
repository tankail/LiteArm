#include "litearm_robot/LiteArm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <cmath>
#include <algorithm>
#include <atomic>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

std::atomic<bool> keep_running(true);

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        keep_running = false;
        std::cout << "\n\n程序被中断，电机即将掉电，请注意安全！" << std::endl;
    }
}

std::vector<double> computeGravityTorque(
    pinocchio::Model& model,
    pinocchio::Data& data,
    const std::vector<double>& q)
{
    Eigen::VectorXd q_eigen(model.nq);
    for (int i = 0; i < model.nq; ++i) {
        q_eigen[i] = (i < (int)q.size()) ? q[i] : 0.0;
    }

    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(model.nv);

    Eigen::VectorXd tau = pinocchio::rnea(model, data, q_eigen, v_zero, a_zero);

    std::vector<double> gravity_torque(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
        gravity_torque[i] = tau[i];
    }
    return gravity_torque;
}

std::vector<double> clipTorque(const std::vector<double>& torque,
                                const std::vector<double>& max_torque)
{
    std::vector<double> clipped(torque.size());
    for (size_t i = 0; i < torque.size(); ++i) {
        clipped[i] = std::max(-max_torque[i], std::min(max_torque[i], torque[i]));
    }
    return clipped;
}

struct ArmState {
    std::string name;
    std::string config_path;
    std::unique_ptr<litearm_robot::LiteArm> robot;
    pinocchio::Model model;
    pinocchio::Data data;
    int n;
};

struct ArmData {
    std::vector<double> q;
    std::vector<double> G;
    std::vector<double> tau_cmd;
};

int main(int argc, char** argv)
{
    try {
        signal(SIGINT, signal_handler);

        std::string config_dir = ament_index_cpp::get_package_share_directory("litearm_config")
            + "/robot_param/";

        std::string urdf_dir = ament_index_cpp::get_package_share_directory("litearm_robot")
            + "/urdf/LiteArm_A10_251224_left_arm.urdf";

        // 支持命令行参数：./dual_left_arm_gravity_compensation <arm0_config> <arm7_config> <urdf>
        std::string arm0_config = config_dir + "litearm_left_arm_0.yaml";
        std::string arm7_config = config_dir + "litearm_left_arm_7.yaml";
        std::string urdf_path = urdf_dir;

        if (argc > 1) arm0_config = argv[1];
        if (argc > 2) arm7_config = argv[2];
        if (argc > 3) urdf_path = argv[3];

        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << "双臂重力补偿程序（左臂 x 2）" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << "手臂0配置文件: " << arm0_config << std::endl;
        std::cout << "手臂7配置文件: " << arm7_config << std::endl;
        std::cout << "URDF文件:      " << urdf_path << std::endl;

        // 初始化手臂0 (/dev/ttyACM0)
        std::cout << "\n>>> 初始化手臂0 (/dev/ttyACM0) ..." << std::endl;
        ArmState arm0;
        arm0.name = "手臂0(ACM0)";
        arm0.config_path = arm0_config;
        arm0.robot = std::make_unique<litearm_robot::LiteArm>(arm0_config);
        arm0.n = arm0.robot->getMotorCount();
        std::cout << "手臂0 电机数量: " << arm0.n << std::endl;

        // 初始化手臂7 (/dev/ttyACM7)
        std::cout << "\n>>> 初始化手臂7 (/dev/ttyACM7) ..." << std::endl;
        ArmState arm7;
        arm7.name = "手臂7(ACM7)";
        arm7.config_path = arm7_config;
        arm7.robot = std::make_unique<litearm_robot::LiteArm>(arm7_config);
        arm7.n = arm7.robot->getMotorCount();
        std::cout << "手臂7 电机数量: " << arm7.n << std::endl;

        // 加载 URDF 模型（两个手臂共用相同的运动学模型）
        std::cout << "\n>>> 加载 URDF 模型 ..." << std::endl;
        pinocchio::urdf::buildModel(urdf_path, arm0.model);
        arm0.data = pinocchio::Data(arm0.model);

        pinocchio::urdf::buildModel(urdf_path, arm7.model);
        arm7.data = pinocchio::Data(arm7.model);

        std::cout << "手臂0 Pinocchio 模型: nq=" << arm0.model.nq << ", nv=" << arm0.model.nv << std::endl;
        std::cout << "手臂7 Pinocchio 模型: nq=" << arm7.model.nq << ", nv=" << arm7.model.nv << std::endl;

        if (arm0.model.nq != arm0.n) {
            std::cerr << "警告: 手臂0 Pinocchio nq=" << arm0.model.nq
                      << " != 电机数量 " << arm0.n << std::endl;
        }
        if (arm7.model.nq != arm7.n) {
            std::cerr << "警告: 手臂7 Pinocchio nq=" << arm7.model.nq
                      << " != 电机数量 " << arm7.n << std::endl;
        }

        // 共用参数
        const int n = arm0.n;
        std::vector<double> tau_limit = {15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0};
        std::vector<double> gravity_gain = {0.85, 1.0, 1.0, 0.8, 1.0, 1.0, 1.0};
        std::vector<double> zero_pos(n, 0.0);
        std::vector<double> zero_vel(n, 0.0);
        std::vector<double> zero_kp(n, 0.0);
        std::vector<double> zero_kd(n, 0.0);

        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << "双臂纯重力补偿模式" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << "力矩限幅: [";
        for (int i = 0; i < n; ++i) {
            std::cout << tau_limit[i] << (i < n-1 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        std::cout << "重力增益: [";
        for (int i = 0; i < n; ++i) {
            std::cout << gravity_gain[i] << (i < n-1 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        std::cout << "\n提示：" << std::endl;
        std::cout << "- 两只手臂将补偿自身重力，可以轻松拖动" << std::endl;
        std::cout << "- 按 Ctrl+C 退出" << std::endl;
        std::cout << "- 退出后电机会掉电，请注意安全！" << std::endl;
        std::cout << std::string(70, '=') << "\n" << std::endl;

        // 先读取手臂0当前位置
        arm0.robot->send_get_motor_state_cmd();
        arm0.robot->motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto init_pos_0 = arm0.robot->getCurrentPos();
        std::cout << "手臂0 初始位置(rad): [";
        for (int i = 0; i < arm0.n; ++i) {
            std::cout << std::fixed << std::setprecision(3) << init_pos_0[i]
                      << (i < arm0.n-1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        // 读取手臂7当前位置
        arm7.robot->send_get_motor_state_cmd();
        arm7.robot->motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto init_pos_7 = arm7.robot->getCurrentPos();
        std::cout << "手臂7 初始位置(rad): [";
        for (int i = 0; i < arm7.n; ++i) {
            std::cout << std::fixed << std::setprecision(3) << init_pos_7[i]
                      << (i < arm7.n-1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        std::cout << "\n开始重力补偿...\n" << std::endl;

        auto last_print_time = std::chrono::steady_clock::now();
        const double print_interval = 0.5;
        int loop_count = 0;

        ArmData data0, data7;

        while (keep_running) {
            // ====== 手臂0 ======
            arm0.robot->send_get_motor_state_cmd();
            arm0.robot->motor_send_cmd();
            data0.q = arm0.robot->getCurrentPos();

            data0.G = computeGravityTorque(arm0.model, arm0.data, data0.q);
            data0.G.resize(n);
            for (int i = 0; i < n; ++i) {
                data0.G[i] *= gravity_gain[i];
            }
            data0.tau_cmd = clipTorque(data0.G, tau_limit);

            arm0.robot->posVelTorqueKpKd(zero_pos, zero_vel, data0.tau_cmd, zero_kp, zero_kd);

            // ====== 手臂7 ======
            arm7.robot->send_get_motor_state_cmd();
            arm7.robot->motor_send_cmd();
            data7.q = arm7.robot->getCurrentPos();

            data7.G = computeGravityTorque(arm7.model, arm7.data, data7.q);
            data7.G.resize(n);
            for (int i = 0; i < n; ++i) {
                data7.G[i] *= gravity_gain[i];
            }
            data7.tau_cmd = clipTorque(data7.G, tau_limit);

            arm7.robot->posVelTorqueKpKd(zero_pos, zero_vel, data7.tau_cmd, zero_kp, zero_kd);

            // 定期打印状态（双臂对比）
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_print_time).count();
            if (elapsed >= print_interval) {
                std::cout << std::fixed << std::setprecision(3);

                std::cout << "\n========== 循环 #" << loop_count << " ==========" << std::endl;

                // 关节角度对比（度）
                std::cout << "关节角度(°):" << std::endl;
                std::cout << "  关节  |  手臂0(ACM0)  |  手臂7(ACM7)" << std::endl;
                std::cout << "---------+---------------+---------------" << std::endl;
                for (int i = 0; i < n; ++i) {
                    std::cout << "  joint" << (i+1) << "  |"
                              << std::setw(9) << (data0.q[i] * 180.0 / M_PI) << "     |"
                              << std::setw(9) << (data7.q[i] * 180.0 / M_PI) << std::endl;
                }

                // 重力力矩对比
                std::cout << "重力力矩(Nm):" << std::endl;
                std::cout << "  关节  |  手臂0(ACM0)  |  手臂7(ACM7)" << std::endl;
                std::cout << "---------+---------------+---------------" << std::endl;
                for (int i = 0; i < n; ++i) {
                    std::cout << "  joint" << (i+1) << "  |"
                              << std::setw(10) << data0.G[i] << "    |"
                              << std::setw(10) << data7.G[i] << std::endl;
                }

                last_print_time = now;
            }

            loop_count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        std::cout << "\n重力补偿已停止，电机会掉电（析构函数自动处理）。" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
