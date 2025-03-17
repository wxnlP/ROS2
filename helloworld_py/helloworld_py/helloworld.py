import rclpy

def main():
    # 初始化 ROS2
    rclpy.init()
    # 创建节点
    node = rclpy.create_node("helloworld_node")
    # 输出文本
    node.get_logger().error("damage world!")
    rclpy.spin(node)
    # 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()
