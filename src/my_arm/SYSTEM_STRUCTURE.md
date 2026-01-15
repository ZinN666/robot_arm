# My Arm 系统结构说明

## 一、系统架构概览

整个系统分为两个主要包：

### 1. `my_arm_description` - 机器人描述包
负责机器人的**几何描述**和**可视化配置**

### 2. `my_arm` - 机器人控制包
负责机器人的**硬件接口**、**控制器**和**启动配置**

---

## 二、`my_arm_description` 包结构

```
my_arm_description/
├── my_arm/
│   ├── urdf/                    # URDF 描述文件
│   │   ├── inc/                 # 宏定义文件
│   │   │   ├── joint_module.xacro    # 标准关节模块宏（球体+双连杆）
│   │   │   └── create_link.xacro     # 创建链接的辅助宏（未使用）
│   │   └── my_arm_description.urdf.xacro  # 机器人几何描述主文件
│   ├── meshes/                  # 3D 模型文件（当前未使用，使用几何体）
│   ├── rviz/                    # RViz 配置文件
│   │   └── view_robot.rviz
│   └── srdf/                    # SRDF 文件（语义描述）
└── package.xml, CMakeLists.txt  # 包配置文件
```

### 功能说明

1. **joint_module.xacro**: 定义标准关节模块
   - `joint_module` 宏：创建关节（球体 + 两个圆柱连杆）
   - `base_module` 宏：创建基座（球体 + 一个向上连杆）

2. **my_arm_description.urdf.xacro**: 组装机器人
   - 使用宏创建 `base_link`、`link_1`、`link_2`、`end_effector`
   - 定义关节连接关系

---

## 三、`my_arm` 包结构

```
my_arm/
├── description/                 # 机器人描述（包含 ros2_control）
│   ├── urdf/
│   │   └── my_arm.urdf.xacro    # 主 URDF 文件（整合几何+控制）
│   ├── ros2_control/
│   │   └── my_arm.ros2_control.xacro  # ros2_control 硬件接口配置
│   └── launch/
│       └── view_my_arm.launch.py      # 可视化启动文件
│
├── hardware/                    # 硬件接口实现
│   ├── include/my_arm/
│   │   └── my_arm_hardware.hpp # 硬件接口头文件
│   └── my_arm_hardware.cpp      # 硬件接口实现
│
├── controller/                  # 控制器实现
│   ├── include/my_arm/
│   │   └── my_arm_controller.hpp # 控制器头文件
│   └── my_arm_controller.cpp    # 控制器实现（轨迹跟踪）
│
├── bringup/                     # 启动和配置
│   ├── config/
│   │   └── my_arm_controller.yaml  # 控制器配置文件
│   └── launch/
│       └── my_arm_controller.launch.py  # 完整控制系统启动文件
│
├── scripts/                     # Python 脚本
│   └── joint_state_to_forward_command.py  # GUI 到控制器的桥接节点
│
├── reference_generator/         # 参考轨迹生成器
│   └── send_trajectory.cpp
│
└── test/                        # 测试文件
```

---

## 四、系统工作流程

### 数据流向

```
用户输入 (GUI)
    ↓
joint_state_publisher_gui
    ↓ 发布 /joint_states (sensor_msgs/JointState)
joint_state_to_forward_command (桥接节点)
    ↓ 转换并发布 /forward_position_controller/commands (std_msgs/Float64MultiArray)
forward_position_controller
    ↓ 写入硬件接口命令
RobotSystem (硬件接口)
    ↓ read() 读取命令，更新状态
    ↓ write() 写入硬件（当前为空实现）
joint_state_broadcaster
    ↓ 发布 /joint_states
robot_state_publisher
    ↓ 发布 TF 和 /robot_description
RViz2
    ↓ 可视化显示
```

### 关键组件说明

1. **硬件接口 (RobotSystem)**
   - 实现 `hardware_interface::SystemInterface`
   - `read()`: 从命令接口读取命令，更新状态接口
   - `write()`: 写入硬件（当前为空，因为是仿真）

2. **控制器 (RobotController)**
   - 实现 `controller_interface::ControllerInterface`
   - 订阅轨迹消息 (`~/joint_trajectory`)
   - 插值轨迹点并写入硬件接口

3. **forward_position_controller**
   - 标准控制器，将 topic 命令转发到硬件接口
   - 用于 GUI 控制

---

## 五、文件关系图

```
my_arm_description/
  └── my_arm_description.urdf.xacro
           ↑ 被包含
my_arm/
  └── description/urdf/my_arm.urdf.xacro
           ├── 包含几何描述
           └── 包含 ros2_control 配置
                  └── 指定硬件插件: my_arm/RobotSystem
                           └── 实现: hardware/my_arm_hardware.cpp
```

---

## 六、关键接口

### 硬件接口
- **命令接口**: `joint_X/position`, `joint_X/velocity`
- **状态接口**: `joint_X/position`, `joint_X/velocity`

### ROS 2 Topics
- `/joint_states`: 关节状态（来自 joint_state_broadcaster）
- `/forward_position_controller/commands`: 位置命令（发送到硬件）
- `/my_arm_controller/joint_trajectory`: 轨迹命令（发送到控制器）

---

## 七、扩展方法

### 添加新关节
1. 在 `my_arm_description.urdf.xacro` 中添加 `joint_module`
2. 在 `my_arm.ros2_control.xacro` 中添加关节配置
3. 更新硬件接口中的关节数量（`my_arm_hardware.cpp`）
4. 更新控制器配置（`my_arm_controller.yaml`）

### 修改关节参数
- 修改 `joint_module.xacro` 中的几何参数
- 修改 `my_arm.ros2_control.xacro` 中的限制参数

