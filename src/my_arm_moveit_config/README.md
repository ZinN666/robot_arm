# My Arm MoveIt Configuration Package

这是 my_arm 机器人的 MoveIt 配置包。

## 文件结构

```
my_arm_moveit_config/
├── config/                    # MoveIt 配置文件
│   ├── my_arm.srdf           # SRDF 语义描述文件
│   ├── joint_limits.yaml     # 关节限制配置
│   ├── kinematics.yaml       # 运动学配置
│   ├── moveit_controllers.yaml  # MoveIt 控制器配置
│   ├── planning_pipeline.yaml    # 规划管道配置
│   ├── trajectory_execution.yaml # 轨迹执行配置
│   ├── moveit_cpp.yaml       # MoveIt C++ 配置
│   └── planning_scene_monitor.yaml  # 规划场景监控配置
├── launch/                   # 启动文件
│   ├── demo.launch.py        # 完整演示启动文件（包含 MoveIt 和 RViz）
│   └── move_group.launch.py  # MoveGroup 启动文件
└── package.xml
```

## 使用方法

### 1. 启动 MoveIt 演示（包含 RViz）

```bash
# 首先启动机器人控制器
ros2 launch my_arm my_arm_controller.launch.py

# 在另一个终端启动 MoveIt
ros2 launch my_arm_moveit_config demo.launch.py
```

### 2. 只启动 MoveGroup（不包含 RViz）

```bash
ros2 launch my_arm_moveit_config move_group.launch.py
```

## 配置说明

### SRDF 文件

定义了：
- **规划组**：`my_arm_manipulator`（从 base_link 到 end_effector）
- **预设姿态**：`home`（所有关节为 0）
- **禁用碰撞对**：相邻链接之间的碰撞检测

### 控制器配置

MoveIt 使用 `my_arm_trajectory_controller` 来执行轨迹。确保在 `my_arm` 包的控制器配置中已启用该控制器。

## 注意事项

1. 确保 `my_arm_trajectory_controller` 已在 `my_arm/bringup/config/my_arm_controller.yaml` 中配置
2. 确保机器人控制器已启动并运行
3. 如果使用 RViz，需要配置 RViz 配置文件（`config/moveit.rviz`）

## 下一步

1. 使用 MoveIt Setup Assistant 进一步配置（可选）
2. 配置 RViz 配置文件以优化可视化
3. 添加更多预设姿态（group states）
4. 配置碰撞检测和规划场景


