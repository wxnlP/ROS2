import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self, name):
        super().__init__(name)
        # 创建订阅方
        self.subscription = self.create_subscription(String, "TopicTest", self.listener_callback, 10)


    def listener_callback(self, msg):
        """处理订阅到的消息"""
        self.get_logger().info(f'订阅的消息:{msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber("subscriber_py_node")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()