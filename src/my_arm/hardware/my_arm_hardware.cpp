// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "my_arm/my_arm_hardware.hpp"
#include <string>
#include <vector>

namespace my_arm
{
// 硬件接口初始化
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 遍历所有关节，收集状态接口信息
  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  // 根据实际关节数量动态分配（每个关节有位置和速度接口）
  size_t num_joints = info_.joints.size();
  joint_position_.assign(num_joints, 0);
  joint_velocities_.assign(num_joints, 0);
  joint_position_command_.assign(num_joints, 0);
  joint_velocities_command_.assign(num_joints, 0);

  return CallbackReturn::SUCCESS;
}

// 导出状态接口：供控制器读取关节状态
std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 导出位置状态接口
  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  // 导出速度状态接口
  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

// 导出命令接口：供控制器写入关节命令
std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // 导出位置命令接口
  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  // 导出速度命令接口
  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

// 读取函数：从硬件读取状态（仿真中从命令更新状态）
return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // 如果使用速度命令，根据速度积分更新位置
  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }

  // 如果使用位置命令，直接设置位置
  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }

  return return_type::OK;
}

// 写入函数：向硬件写入命令（当前为空实现，因为是仿真）
return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 在实际硬件中，这里应该将命令发送到电机驱动器
  return return_type::OK;
}

}  // namespace my_arm

// 导出硬件接口插件，使其可被 ros2_control 加载
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_arm::RobotSystem, hardware_interface::SystemInterface)
