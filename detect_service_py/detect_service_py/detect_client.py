import rclpy
from rclpy.node import Node
# ROS2自带图像消息接口
from sensor_msgs.msg import Image
from detect_interfaces.srv import FaceDetect
from ament_index_python.packages import get_package_share_directory
import cv2
# ROS2提供，用于将OpenCV的图像格式转为ROS2格式
from cv_bridge import CvBridge
# 导入设置参数的服务消息接口
from rcl_interfaces.srv import SetParameters
# 导入设置参数的话题消息接口
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class FaceDetectClient(Node):
    def __init__(self, name):
        super().__init__(name)
        # 格式转换对象
        self.bridge = CvBridge()
        # 创建客户端
        self.client = self.create_client(FaceDetect, "/face_detect")
        # 图像路径
        self.pic_path = get_package_share_directory("detect_service_py") + "/resource/pic1.jpg"
        # 图像
        self.pic = cv2.imread(self.pic_path)

    def call_set_parameter(self, parameters):
        # 1.创建客户端，并等待服务上线
        client = self.create_client(SetParameters, "/face_detect/set_parameters")
        while client.wait_for_service(timeout_sec=1) is False:
             self.get_logger().info("等待服务上线...")
        # 2.构造Request(由于消息接口复杂，故消息赋值单开一个函数完成)
        request = SetParameters.Request()
        request.parameters = parameters
        # 3.发送请求并等待服务端处理完成(异步获取结果)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future=future)
        response = future.result()
        return response
    
    def update_detect_parameter(self, times=1):
        # 1.创建一个参数对象
        param = Parameter()
        param.name = "face_locations_upsample_times"
        # 2.创建参数值对象
        param_value = ParameterValue()
        # 整数数据
        param_value.type = ParameterType.PARAMETER_INTEGER
        param_value.integer_value = times
        # 3.赋值参数对象
        param.value = param_value
        # 4.发送更新参数请求
        response = self.call_set_parameter([param])
        for result in response.results:
            if result.successful:
                self.get_logger().info(f"参数{param.name}被设置为{times}")     
            else:
                self.get_logger().error(f"参数设置错误{result.reason}")

    def show_face_loactions(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.pic, (left, top), (right, bottom), (0, 0, 255), 6)
        cv2.imwrite("/home/sunrise/wkspace1/src/detect_service_py/resource/outs.jpg", self.pic)

    def send_request(self):
        # 1.等待服务上线
        while self.client.wait_for_service(timeout_sec=1) is False:
            self.get_logger().info("等待服务上线...")
        # 2.构造Request
        request = FaceDetect.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.pic)
        # 3.发送请求并等待服务端处理完成(异步获取结果)
        future = self.client.call_async(request)
        # 4.等待future为完成
        rclpy.spin_until_future_complete(self, future=future)
        # 5.得到响应数据
        response = future.result()
        self.get_logger().info(
            f'收到响应：图像人脸数量 {response.number}, 耗时 {response.use_time}')
        self.show_face_loactions(response)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectClient("face_detect_client")
    node.update_detect_parameter(times=1)
    node.send_request()
    node.update_detect_parameter(times=2)
    node.send_request()
    rclpy.shutdown()
