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

#ifndef MY_ARM__MY_ARM_HARDWARE_HPP_
#define MY_ARM__MY_ARM_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace my_arm
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// 硬件接口类：实现 ros2_control 的 SystemInterface
class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  // 初始化硬件接口
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // 导出状态接口：供控制器读取关节状态
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 导出命令接口：供控制器写入关节命令
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 读取函数：从硬件读取状态（仿真中从命令更新状态）
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 写入函数：向硬件写入命令（当前为空实现，因为是仿真）
  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  // 向量大小为：(接口类型数量 x 关节数量)
  std::vector<double> joint_position_command_;      // 位置命令
  std::vector<double> joint_velocities_command_;    // 速度命令
  std::vector<double> joint_position_;              // 位置状态
  std::vector<double> joint_velocities_;            // 速度状态

  // 关节接口映射：按接口类型组织关节名称
  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
};

}  // namespace my_arm

#endif  // MY_ARM__MY_ARM_HARDWARE_HPP_
