import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MiniPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"话题名称:{name}")
        # 创建发布者
        self.publisher1 = self.create_publisher(String, "TopicTest", 10)
        # 创建定时器
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.callback)
        self.cg = 0

    def callback(self):
        """定时器反馈函数"""
        # 创建数据
        msg = String()
        msg.data = f"这是一条发布者[{self.cg}]信息"
        # 发布数据
        self.publisher1.publish(msg)
        self.cg = self.cg + 1

def main(args=None):
    rclpy.init(args=args)
    node = MiniPublisher("publisher_py_node")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()