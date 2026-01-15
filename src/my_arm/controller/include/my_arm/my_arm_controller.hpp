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

#ifndef MY_ARM__MY_ARM_CONTROLLER_HPP_
#define MY_ARM__MY_ARM_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace my_arm
{
// 控制器类：实现轨迹跟踪控制器
class RobotController : public controller_interface::ControllerInterface
{
public:
  RobotController();

  // 配置命令接口：定义控制器需要的命令接口
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // 配置状态接口：定义控制器需要的状态接口
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // 更新函数：每个控制周期调用，执行轨迹跟踪
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 初始化控制器
  controller_interface::CallbackReturn on_init() override;

  // 配置控制器：订阅轨迹消息
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // 激活控制器：分配接口引用
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 停用控制器：释放接口
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;                    // 关节名称列表
  std::vector<std::string> command_interface_types_;        // 命令接口类型列表
  std::vector<std::string> state_interface_types_;          // 状态接口类型列表

  // 轨迹消息订阅器
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;
  // 实时缓冲区：存储轨迹消息（线程安全）
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    traj_msg_external_point_ptr_;
  bool new_msg_ = false;                                    // 是否有新消息标志
  rclcpp::Time start_time_;                                 // 轨迹开始时间
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;  // 当前执行的轨迹
  trajectory_msgs::msg::JointTrajectoryPoint point_interp_; // 插值后的轨迹点

  // 命令接口引用（从硬件接口借用）
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;                      // 位置命令接口
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;                      // 速度命令接口
  // 状态接口引用（从硬件接口借用）
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;                        // 位置状态接口
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;                        // 速度状态接口

  // 命令接口映射：按接口类型组织接口引用
  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"position", &joint_position_command_interface_},
      {"velocity", &joint_velocity_command_interface_}};

  // 状态接口映射：按接口类型组织接口引用
  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"velocity", &joint_velocity_state_interface_}};
};

}  // namespace my_arm

#endif  // MY_ARM__MY_ARM_CONTROLLER_HPP_

