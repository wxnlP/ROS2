import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math


class StaticTfBroadcaster(Node):
    def __init__(self, name):
        super().__init__(name)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        """发布静态坐标变换"""
        transform = TransformStamped()
        # 消息头
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        # 子坐标名称
        transform.child_frame_id = 'camera_link'
        # 平面坐标关系
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6

        # 欧拉角(角度转弧度)转四元数
        q = quaternion_from_euler(math.radians(180), 0.0, 0.0)

        # 旋转坐标关系
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # 发布静态坐标变换
        self.static_tf.sendTransform(transform)
        self.get_logger().info(f"发布 TF:{transform}")

def main():
    rclpy.init()
    node = StaticTfBroadcaster("static_tf")
    rclpy.spin(node)
    rclpy.shutdown()