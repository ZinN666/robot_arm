# My Arm Robot Package - 2 Joint Example

这是一个使用标准关节模块构建的2关节机器人示例。

## 标准关节设计

每个关节模块包含：
- **关节主体**：直径 20mm 的白色球体
- **两个连接杆**：
  - 第一个连杆：直径 10mm，可调长度的白色圆柱（向上）
  - 第二个连杆：直径 10mm，可调长度的白色圆柱（向前/向外）

## 机器人结构

- `world` - 世界坐标系
- `base_link` - 基座（球体 + 50mm 向上连杆，固定到 world）
- `link_1` - 第一个关节（球体 + 150mm 向上连杆 + 150mm 向前连杆）
- `link_2` - 第二个关节（球体 + 150mm 向上连杆 + 150mm 向前连杆）
- `end_effector` - 末端执行器（小球）

## 使用方法

### 编译包

```bash
cd /home/kai/robot_ws
colcon build --packages-select my_arm_description my_arm
source install/setup.bash
```

### 可视化（仅查看模型）

```bash
ros2 launch my_arm view_my_arm.launch.py
```

### 启动完整控制系统（包含控制器）

```bash
ros2 launch my_arm my_arm_controller.launch.py
```

### 使用 joint_state_publisher_gui 控制关节

在另一个终端：

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

然后可以通过 GUI 滑块控制两个关节的角度。

## 文件结构

- `my_arm_description/my_arm/urdf/inc/joint_module.xacro` - 标准关节模块宏
  - `joint_module`: 球体 + 两个圆柱连杆
  - `base_module`: 球体 + 一个向上圆柱连杆
- `my_arm_description/my_arm/urdf/my_arm_description.urdf.xacro` - 机器人描述
- `my_arm/description/ros2_control/my_arm.ros2_control.xacro` - ros2_control 配置
- `my_arm/hardware/` - 硬件接口实现（2个关节）
- `my_arm/controller/` - 控制器实现
- `my_arm/bringup/` - 启动文件和配置

## 扩展

要添加更多关节，只需在 `my_arm_description.urdf.xacro` 中添加：

```xml
<xacro:joint_module 
  link_name="link_3" 
  joint_name="joint_3" 
  parent_link="link_2"
  rod1_length="0.15"
  rod2_length="0.15"/>
```

然后在 `my_arm.ros2_control.xacro` 中添加对应的关节配置。

## 参数说明

- `rod1_length`: 向上连杆的长度（米）
- `rod2_length`: 向前连杆的长度（米）
