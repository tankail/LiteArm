#!/usr/bin/env python3
"""
LiteArm Joint Test Script
测试每个关节，确认硬件电机顺序与 URDF 关节名称的对应关系

用法:
  1. 先启动硬件: ros2 launch litearm_config hardware.launch.py
  2. 再运行测试: ros2 run litearm_examples test_single_joint.py

按 1-7 测试右臂关节，g 测试夹爪
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
import sys

class JointTestNode(Node):
    def __init__(self):
        super().__init__('litearm_joint_test')

        # 发布轨迹命令到 right_arm_controller
        self.pub = self.create_publisher(JointTrajectory, '/right_arm_controller/joint_trajectory', 10)

        # 订阅关节状态
        self.joint_states = {}
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 右臂关节名称
        self.arm_joints = [
            'r_joint1_joint', 'r_joint2_joint', 'r_joint3_joint',
            'r_joint4_joint', 'r_joint5_joint', 'r_joint6_joint', 'r_joint7_joint'
        ]
        self.gripper_joint = 'r_r_finger_joint'

        self.get_logger().info('LiteArm Joint Test Node Started')
        self.print_help()

    def print_help(self):
        print("\n" + "="*60)
        print("LiteArm 关节测试")
        print("="*60)
        print("按 1-7: 测试右臂关节 (r_joint1-7)")
        print("按 g:   测试夹爪 (r_r_finger_joint)")
        print("按 s:   停止所有关节 (回到当前位置)")
        print("按 q:   退出程序")
        print("="*60)
        print("\n当前关节状态:")
        self.print_joint_states()
        print("\n输入命令: ", end="", flush=True)

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.arm_joints or name == self.gripper_joint:
                if i < len(msg.position):
                    self.joint_states[name] = msg.position[i]

    def print_joint_states(self):
        for joint in self.arm_joints:
            pos = self.joint_states.get(joint, 0.0)
            print(f"  {joint}: {pos:.4f} rad")
        gripper_pos = self.joint_states.get(self.gripper_joint, 0.0)
        print(f"  {self.gripper_joint}: {gripper_pos:.4f} m")

    def move_joint(self, joint_index, delta=0.5):
        """移动指定索引的关节"""
        # 获取当前状态
        current_pos = [self.joint_states.get(j, 0.0) for j in self.arm_joints]

        # 移动指定关节
        target_pos = current_pos.copy()
        target_pos[joint_index] += delta

        self.send_trajectory(target_pos)

        print(f"\n>>> 移动 r_joint{joint_index+1}_joint (delta={delta})")
        print(f"    当前位置: {current_pos[joint_index]:.4f}")
        print(f"    目标位置: {target_pos[joint_index]:.4f}")

    def move_gripper(self, delta=0.01):
        """移动夹爪"""
        current = self.joint_states.get(self.gripper_joint, 0.0)
        target = current + delta

        msg = JointTrajectory()
        msg.joints = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start = rclpy.duration.Duration(seconds=1).to_msg()
        msg.points = [point]

        self.pub.publish(msg)

        print(f"\n>>> 移动夹爪 (delta={delta})")
        print(f"    当前位置: {current:.4f} m")
        print(f"    目标位置: {target:.4f} m")

    def stop_all(self):
        """停止所有关节，回到当前位置"""
        current_pos = [self.joint_states.get(j, 0.0) for j in self.arm_joints]
        self.send_trajectory(current_pos)
        print("\n>>> 停止并保持在当前位置")

    def send_trajectory(self, positions):
        """发送轨迹命令"""
        msg = JointTrajectory()
        msg.joints = self.arm_joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
        msg.points = [point]
        self.pub.publish(msg)

    def run(self):
        print("\n输入命令: ", end="", flush=True)
        while rclpy.ok():
            try:
                cmd = input().strip().lower()

                if cmd == 'q':
                    print("退出程序...")
                    rclpy.shutdown()
                    return
                elif cmd == 's':
                    self.stop_all()
                elif cmd == 'g':
                    self.move_gripper(0.02)  # 打开夹爪
                elif cmd in ['1', '2', '3', '4', '5', '6', '7']:
                    joint_idx = int(cmd) - 1
                    self.move_joint(joint_idx, 0.3)
                elif cmd == '8':
                    self.move_gripper(-0.02)  # 关闭夹爪
                else:
                    print("无效命令!")
                    self.print_help()

                print("\n输入命令: ", end="", flush=True)

            except EOFError:
                break
            except Exception as e:
                print(f"错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JointTestNode()
    node.run()

if __name__ == '__main__':
    main()