import rclpy
from rclpy.node import Node
from detect_interfaces.srv import FaceDetect
from ament_index_python.packages import get_package_share_directory
import cv2
# ROS2提供，用于将OpenCV的图像格式转为ROS2格式
from cv_bridge import CvBridge
import face_recognition
import time
from rcl_interfaces.msg import SetParametersResult


class FaceDetectService(Node):
    def __init__(self, name):
        super().__init__(name)
        # 格式转换对象
        self.bridge = CvBridge()
        # 创建服务
        self.service = self.create_service(FaceDetect, "/face_detect", self.srv_callback)
        
        """添加参数声明"""
        # 检测参数
        # self.upsample_times = 1
        # self.model = "hog"
        # 声明参数
        self.declare_parameter("face_locations_upsample_times", 1)
        self.declare_parameter("face_locations_model", "hog")
        # 获取参数
        self.upsample_times = self.get_parameter("face_locations_upsample_times").value
        self.model = self.get_parameter("face_locations_model").value
        # 添加参数回调函数
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # 默认图片
        self.default_pic = get_package_share_directory("detect_service_py") + "/resource/pic1.jpg"

    def parameters_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(f"参数{parameter.name}被设置为{parameter.value}")
            if parameter.name == "face_locations_upsample_times":
                self.upsample_times = self.get_parameter("face_locations_upsample_times").value
            elif parameter.name == "face_locations_model":
                self.model = self.get_parameter("face_locations_model").value
        return SetParametersResult(successful=True)

    def srv_callback(self, request, response):
        # 如果有数据
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_pic)
        start_time = time.time()
        self.get_logger().info("加载图像完成，开始检测。。。")
        face_locations = face_recognition.face_locations(
            cv_image, number_of_times_to_upsample=self.upsample_times, model=self.model)
        end_time = time.time()
        self.get_logger().info(f"检测完成，耗时：{end_time-start_time}")
        response.number = len(face_locations)
        response.use_time = end_time-start_time
        # 接收响应数据
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectService("face_detect")
    rclpy.spin(node)
    rclpy.shutdown()