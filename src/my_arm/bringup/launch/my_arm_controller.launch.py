# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="是否自动启动 RViz2。",
        )
    )
    # 通过 xacro 获取 URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_arm"),
                    "urdf",
                    "my_arm.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 控制器配置文件路径
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_arm"),
            "config",
            "my_arm_controller.yaml",
        ]
    )
    # RViz 配置文件路径
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_arm_description"), "my_arm/rviz", "view_robot.rviz"]
    )

    # ros2_control 节点：管理硬件接口和控制器
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )
    # 机器人状态发布器：发布 TF 和机器人描述
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    gui = LaunchConfiguration("gui")
    # RViz2 可视化节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # 关节状态发布器 GUI：提供滑块控制界面
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    # 桥接节点：将 joint_states 转换为 forward_position_controller 命令
    joint_state_to_forward_command_node = Node(
        package="my_arm",
        executable="joint_state_to_forward_command.py",
        name="joint_state_to_forward_command",
        condition=IfCondition(gui),
    )

    # 启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 启动前向位置控制器
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    # 启动自定义轨迹控制器
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_arm_controller", "-c", "/controller_manager"],
    )

    # 延迟启动 RViz：在 joint_state_broadcaster 启动后
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # 延迟启动 forward_position_controller：在 joint_state_broadcaster 启动后
    delay_forward_position_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    # 延迟启动 robot_controller：在 forward_position_controller 启动后
    delay_robot_controller_spawner_after_forward_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_publisher_gui_node,
        joint_state_to_forward_command_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_forward_position_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_forward_position_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

