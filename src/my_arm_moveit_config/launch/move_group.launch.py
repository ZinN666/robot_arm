#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

    # 获取参数
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    # SRDF 文件路径
    robot_description_semantic = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_moveit_config"),
            "config",
            "my_arm.srdf",
        ]
    )

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

    # MoveGroup 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_kinematics": robot_description_kinematics},
            planning_pipeline_config,
            trajectory_execution_config,
            moveit_controllers_config,
            planning_scene_monitor_config,
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription(declared_arguments + [move_group_node])
