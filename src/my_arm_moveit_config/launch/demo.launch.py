#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="是否使用仿真时间",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否启动 RViz",
        )
    )

    # 获取参数
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    # 获取 URDF
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

    # SRDF 文件内容（需要读取文件内容，而不是路径）
    srdf_file = os.path.join(
        get_package_share_directory("my_arm_moveit_config"),
        "config",
        "my_arm.srdf"
    )
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # MoveIt 配置文件路径
    robot_description_kinematics = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_moveit_config"),
            "config",
            "kinematics.yaml",
        ]
    )

    planning_pipeline_config = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_moveit_config"),
            "config",
            "planning_pipeline.yaml",
        ]
    )

    trajectory_execution_config = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_moveit_config"),
            "config",
            "trajectory_execution.yaml",
        ]
    )

    moveit_controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_moveit_config"),
            "config",
            "moveit_controllers.yaml",
        ]
    )

    planning_scene_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_moveit_config"),
            "config",
            "planning_scene_monitor.yaml",
        ]
    )

    joint_limits_config = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_moveit_config"),
            "config",
            "joint_limits.yaml",
        ]
    )

    # MoveGroup 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            {"robot_description_semantic": ParameterValue(robot_description_semantic, value_type=str)},
            {"robot_description_kinematics": robot_description_kinematics},
            planning_pipeline_config,
            trajectory_execution_config,
            moveit_controllers_config,
            planning_scene_monitor_config,
            joint_limits_config,
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
    )

    # RViz 配置（使用 my_arm_description 的 RViz 配置作为备用）
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_description"),
            "my_arm",
            "rviz",
            "view_robot.rviz",
        ]
    )

    # RViz 节点（可选）
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            {"robot_description_semantic": ParameterValue(robot_description_semantic, value_type=str)},
            {"robot_description_kinematics": robot_description_kinematics},
            planning_pipeline_config,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
