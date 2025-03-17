import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus
# 动态监控系统资源，如CPU、内存使用等
import psutil
# 静态获取系统基本信息，如操作系统信息、硬件架构等
import platform


class SysStatusPub(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"话题名称:{name}")
        self.publisher_ = self.create_publisher(SystemStatus, name, 10)
        self.timer_ = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        """定时器回调函数"""
        msg = SystemStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.host_name = platform.node()
        msg.cpu_percent = psutil.cpu_percent()
        msg.memory_percent = psutil.virtual_memory().percent
        # 内存的默认单位均是字节，除两个1024单位就变成MB了
        msg.memory_total = psutil.virtual_memory().total / 1024 / 1024
        msg.memory_available = psutil.virtual_memory().available / 1024 / 1024
        msg.net_send = psutil.net_io_counters().bytes_sent / 1024 / 1024
        msg.net_receive = psutil.net_io_counters().bytes_recv / 1024 / 1024
        self.get_logger().info(f'发布{str(msg)}')
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = SysStatusPub("sys_status")
    rclpy.spin(node)
    rclpy.shutdown()