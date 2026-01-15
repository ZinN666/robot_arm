#!/usr/bin/env python3
"""
桥接节点：将 joint_state_publisher_gui 的输出转换为 forward_position_controller 的命令
这使得 joint_state_publisher_gui 可以控制 ros2_control 机器人
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointStateToForwardCommand(Node):
    def __init__(self):
        super().__init__('joint_state_to_forward_command')
        
        # 订阅来自 joint_state_publisher_gui 的关节状态
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 发布命令到 forward_position_controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        
        # 关节顺序（应该与控制器配置中的顺序匹配）
        self.joint_names = ['base_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        self.get_logger().info('关节状态到前向命令桥接节点已启动')
        self.get_logger().info(f'订阅话题: /joint_states')
        self.get_logger().info(f'发布话题: /forward_position_controller/commands')
        self.get_logger().info(f'控制关节: {self.joint_names}')

    def joint_state_callback(self, msg):
        """将 JointState 转换为 Float64MultiArray 命令"""
        # 创建与关节顺序匹配的命令数组
        command = Float64MultiArray()
        
        # 将关节位置从 joint_states 映射到命令数组
        for joint_name in self.joint_names:
            try:
                idx = msg.name.index(joint_name)
                command.data.append(msg.position[idx])
            except ValueError:
                # 关节在消息中未找到，使用 0.0
                self.get_logger().warn(f'关节 {joint_name} 在 joint_states 中未找到')
                command.data.append(0.0)
        
        # 发布命令
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToForwardCommand()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

