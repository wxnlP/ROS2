import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion


class TfListener(Node):
    def __init__(self, name):
        super().__init__(name)
        self.buffer = Buffer()
        self.tf_listener = TransformListener(node=self, buffer=self.buffer)
        self.timer = self.create_timer(1, self.get_transform)

    def get_transform(self):
        """获取坐标变换"""
        try:
            result = self.buffer.lookup_transform('base_link', 'bottle_link',
                # 获取transform的时间段，0代表获取最新的
                rclpy.time.Time(seconds=0.0), 
                # 超时时间
                rclpy.time.Duration(seconds=1.0))
            # 解析结果
            transform = result.transform
            # 四元数转欧拉角
            euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ])
            self.get_logger().info(f"平移:{transform.translation}\n旋转四元数:{transform.rotation}\n旋转欧拉角:{euler}")
        except Exception as e:
            self.get_logger().warn(f"不能够获取坐标变换，原因: {str(e)}")


def main():
    rclpy.init()
    node = TfListener("tf_listener")
    rclpy.spin(node)
    rclpy.shutdown()