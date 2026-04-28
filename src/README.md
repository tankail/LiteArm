# LiteArm ROS2 工作空间

## 目录结构

```
src/
├── litearm_a10_251125/      # LiteArm A10 机械臂 URDF 和描述文件
├── litearm_config/          # MoveIt 配置文件、launch 文件、YAML 参数
├── litearm_examples/        # 示例代码
├── litearm_hardware/        # 硬件接口插件（ros2_control）
├── litearm_robot/           # 核心 SDK 和控制例程
└── litearm_robot/urdf/      # 右臂 URDF（用于重力补偿计算）
```

## 右手臂重力补偿

### 原理

通过 Pinocchio 计算当前姿态下的重力补偿力矩 G(q)，发送给电机实现零重力拖动效果：

```
τ = G(q)
```

其中：
- `q` - 当前关节角度（7个关节）
- `G(q)` - RNEA 算法计算的重力力矩

### 相关文件

| 文件 | 说明 |
|------|------|
| `litearm_robot/urdf/LiteArm_A10_251224_right_arm.urdf` | 右臂 URDF（含质量、惯量、关节轴方向） |
| `litearm_robot/examples/right_arm_gravity_calc.cpp` | 重力补偿计算器（只打印，不输出力矩） |
| `litearm_robot/examples/right_arm_gravity_compensation.cpp` | 重力补偿控制器（计算并输出力矩） |
| `litearm_config/robot_param/litearm_right_arm.yaml` | 右臂电机配置文件 |

### 运行方法

```bash
# 1. 进入工作空间
cd ~/LiteArm

# 2. 编译
colcon build --packages-select litearm_robot
source install/setup.bash

# 3. 仅计算重力补偿力矩（安全诊断）
ros2 run litearm_robot right_arm_gravity_calc

# 4. 启用重力补偿输出
ros2 run litearm_robot right_arm_gravity_compensation
```

### 参数调整

编辑 `right_arm_gravity_compensation.cpp` 中的 `gravity_gain` 参数：

```cpp
// 第 119 行
std::vector<double> gravity_gain = {1.0, 1.2, 1.0, 0.8, 1.0, 1.0, 1.0};
//                  [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
```

- `> 1.0` - 力矩放大
- `< 1.0` - 力矩缩小
- `= 1.0` - 不变

**调参步骤**：
1. 松手让手臂在某个姿态下自然悬停
2. 观察手臂往哪个方向漂移
3. 往上漂移 → 减小该关节增益；往下掉 → 增大该关节增益
4. 修改后重新编译

### URDF 关节轴方向说明

如果发现某个关节的力矩方向始终反的，需要修改 `LiteArm_A10_251224_right_arm.urdf` 中的 `axis` 属性：

| 关节 | 当前 axis | 说明 |
|------|-----------|------|
| joint1 | `0 1 0` | 绕 Y 轴旋转 |
| joint2 | `1 0 0` | 绕 X 轴旋转 |
| joint3 | `0 0 -1` | 绕 Z 轴负方向旋转 |
| joint4 | `0 1 0` | 绕 Y 轴旋转 |
| joint5 | `0 0 -1` | 绕 Z 轴负方向旋转 |
| joint6 | `1 0 0` | 绕 X 轴旋转 |
| joint7 | `0 1 0` | 绕 Y 轴旋转 |

修改后重新编译：`colcon build --packages-select litearm_robot`

### 力矩安全限幅

| 关节 | 电机型号 | 限幅 (Nm) | 当前增益 |
|------|----------|-----------|----------|
| joint1 | 5047_36 | 15.0 | 1.0 |
| joint2 | 6056_36 | 25.0 | 1.2 |
| joint3 | 6056_36 | 25.0 | 1.0 |
| joint4 | 5047_36 | 15.0 | 0.8 |
| joint5 | 4438_30 | 6.0 | 1.0 |
| joint6 | 4438_30 | 6.0 | 1.0 |
| joint7 | 4438_30 | 4.0 | 1.0 |

## 其他例程

```bash
# 读取电机状态
ros2 run litearm_robot 0_robot_get_state

# PD 控制
ros2 run litearm_robot 1_PD_control

# 位置速度控制
ros2 run litearm_robot 1_PosVel_control

# 关节空间阻抗控制（需 Pinocchio）
ros2 run litearm_robot 2_joint_impedance_control

# 笛卡尔空间阻抗控制（需 Pinocchio）
ros2 run litearm_robot 3_cartesian_impedance_control
```

## 注意事项

1. **退出时电机掉电** - 按 Ctrl+C 退出后，电机将停止输出力矩，请注意安全
2. **初始姿态** - 建议在运行重力补偿前将手臂摆放到一个安全的初始姿态
3. **观测调参** - 建议先用 `right_arm_gravity_calc` 观察不同姿态下的力矩值，再进行调参