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

#include "my_arm/my_arm_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace my_arm
{
RobotController::RobotController() : controller_interface::ControllerInterface() {}

// 控制器初始化
controller_interface::CallbackReturn RobotController::on_init()
{
  // 从参数服务器读取配置（应该有错误处理）
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  // 初始化插值点
  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);

  return CallbackReturn::SUCCESS;
}

// 配置命令接口：定义控制器需要的命令接口
controller_interface::InterfaceConfiguration RobotController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  // 为每个关节的每个命令接口类型创建接口名称
  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

// 配置状态接口：定义控制器需要的状态接口
controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  // 为每个关节的每个状态接口类型创建接口名称
  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

// 配置控制器：订阅轨迹消息
controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
  // 定义轨迹消息回调函数
  auto callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
  {
    RCLCPP_INFO(get_node()->get_logger(), "收到新的轨迹消息。");
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    new_msg_ = true;
  };

  // 订阅关节轨迹话题
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  return CallbackReturn::SUCCESS;
}

// 激活控制器：分配接口引用
controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  // 清空向量，以防重启
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // 分配命令接口引用
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // 分配状态接口引用
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return CallbackReturn::SUCCESS;
}

// 插值函数：在两个轨迹点之间进行线性插值
void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
{
  // 插值位置
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_1.positions[i];
  }
  // 插值速度
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.velocities[i] =
      delta * point_2.velocities[i] + (1.0 - delta) * point_1.velocities[i];
  }
}

// 轨迹插值函数：根据当前时间计算轨迹中的插值点
void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, bool & reached_end)
{
  double traj_len = static_cast<double>(traj_msg.points.size());
  auto last_time = traj_msg.points.back().time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;
  double cur_time_sec = cur_time.seconds();
  reached_end = (cur_time_sec >= total_time);

  // 如果到达轨迹末端，将速度设置为零
  if (reached_end)
  {
    point_interp.positions = traj_msg.points.back().positions;
    std::fill(point_interp.velocities.begin(), point_interp.velocities.end(), 0.0);
    return;
  }

  // 计算当前时间对应的轨迹点索引（假设轨迹点均匀分布）
  size_t ind =
    static_cast<size_t>(cur_time_sec * (traj_len / total_time));
  ind = std::min(ind, static_cast<size_t>(traj_len) - 2);
  double delta = std::min(cur_time_sec - static_cast<double>(ind) * (total_time / traj_len), 1.0);
  interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

// 更新函数：每个控制周期调用，执行轨迹跟踪
controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // 如果有新轨迹消息，加载它
  if (new_msg_)
  {
    trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
    start_time_ = time;
    new_msg_ = false;
  }

  // 如果存在轨迹，执行插值并发送命令
  if (trajectory_msg_ != nullptr)
  {
    bool reached_end;
    interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_, reached_end);

    // 如果到达轨迹末端，重置轨迹
    if (reached_end)
    {
      RCLCPP_INFO(get_node()->get_logger(), "轨迹执行完成。");
      trajectory_msg_.reset();
    }

    // 设置位置命令
    for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
    {
      joint_position_command_interface_[i].get().set_value(point_interp_.positions[i]);
    }
    // 设置速度命令
    for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
    {
      joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i]);
    }
  }

  return controller_interface::return_type::OK;
}

// 停用控制器：释放接口
controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

}  // namespace my_arm

// 导出控制器插件，使其可被 controller_manager 加载
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_arm::RobotController, controller_interface::ControllerInterface)

