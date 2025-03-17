#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class TestNode(Node):
    """
    继承Node类,并创建一个节点,打印节点名称
    """
    def __init__(self, node):
        super().__init__(node)
        self.node = node
        self.get_logger().info(f"节点名称为{self.node}")


def main(args=None):
    """
    ros2运行该节点的入口函数
    """
    rclpy.init(args=args) # 初始化rclpy
    node = TestNode("helloworld_node")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy