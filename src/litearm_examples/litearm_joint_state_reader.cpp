#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <iomanip>

class JointStateReader : public rclcpp::Node
{
public:
    JointStateReader()
        : Node("litearm_joint_state_reader")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&JointStateReader::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "LiteArm Joint State Reader started. Listening to /joint_states...");
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to exit.");
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty())
            return;

        // Print header
        std::cout << "\033[2J\033[1;1H";  // Clear screen
        std::cout << "\033[1;36m" << std::string(70, '=') << "\033[0m" << std::endl;
        std::cout << "\033[1;33m LiteArm A10 Right Arm - Joint States\033[0m" << std::endl;
        std::cout << "\033[1;36m" << std::string(70, '=') << "\033[0m" << std::endl;
        std::cout << "Timestamp: " << std::fixed << std::setprecision(3) << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 << " s" << std::endl;
        std::cout << "\033[1;36m" << std::string(70, '-') << "\033[0m" << std::endl;

        std::cout << std::fixed << std::setprecision(4);

        // Filter for right arm joints
        std::vector<std::string> arm_joints = {
            "r_joint1_joint", "r_joint2_joint", "r_joint3_joint",
            "r_joint4_joint", "r_joint5_joint", "r_joint6_joint", "r_joint7_joint"
        };
        std::vector<std::string> gripper_joints = {"r_r_finger_joint", "r_l_finger_joint"};

        bool found_arm = false;
        bool found_gripper = false;

        for (size_t i = 0; i < msg->name.size(); i++)
        {
            std::string joint_name = msg->name[i];

            // Check if it's a right arm joint
            bool is_arm = false;
            for (const auto& arm_joint : arm_joints)
            {
                if (joint_name == arm_joint)
                {
                    is_arm = true;
                    break;
                }
            }

            // Check if it's a gripper joint
            bool is_gripper = false;
            for (const auto& gripper_joint : gripper_joints)
            {
                if (joint_name == gripper_joint)
                {
                    is_gripper = true;
                    break;
                }
            }

            if (is_arm)
            {
                if (!found_arm)
                {
                    std::cout << "\033[1;32m[Right Arm Joints]\033[0m" << std::endl;
                    found_arm = true;
                }

                double pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
                double vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
                double eff = (i < msg->effort.size()) ? msg->effort[i] : 0.0;

                // Extract joint number
                std::string joint_num = joint_name.substr(8, 1);  // "r_jointX_joint" -> X

                std::cout << "  Joint " << joint_num << " (" << std::setw(15) << joint_name << "): "
                         << "pos=" << std::setw(10) << pos << " rad  "
                         << "vel=" << std::setw(10) << vel << " rad/s  "
                         << "eff=" << std::setw(8) << eff << " Nm" << std::endl;
            }
            else if (is_gripper)
            {
                if (!found_gripper)
                {
                    std::cout << "\033[1;35m[Gripper Joints]\033[0m" << std::endl;
                    found_gripper = true;
                }

                double pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
                double vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
                double eff = (i < msg->effort.size()) ? msg->effort[i] : 0.0;

                std::cout << "  " << std::setw(20) << joint_name << ": "
                         << "pos=" << std::setw(10) << pos << " m  "
                         << "vel=" << std::setw(10) << vel << " m/s  "
                         << "eff=" << std::setw(8) << eff << " N" << std::endl;
            }
        }

        std::cout << "\033[1;36m" << std::string(70, '-') << "\033[0m" << std::endl;
        std::cout << "\033[1;90m Press Ctrl+C to exit.\033[0m" << std::endl;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateReader>());
    rclcpp::shutdown();
    return 0;
}
