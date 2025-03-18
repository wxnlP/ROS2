#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"

using StaticTransformBroadcaster = tf2_ros::StaticTransformBroadcaster;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class StaticTfBroadcaster : public rclcpp::Node
{
public:
    StaticTfBroadcaster(std::string name) : Node(name)
    {
        static_tf_ = std::make_shared<StaticTransformBroadcaster>(this);
        this->publish_static_tf();
    }

private:
    void publish_static_tf(void)
    {
        TransformStamped transform;
        // 时间戳
        transform.header.stamp = this->get_clock()->now();
        // 父坐标系名称
        transform.header.frame_id = "map";
        // 子坐标系名称
        transform.child_frame_id = "target_point";
        // 平面移动
        transform.transform.translation.x = 5.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;
        // 欧拉角(弧度)转四元数
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 60 * M_PI / 180);
        // 旋转
        transform.transform.rotation.x = quat[0];
        transform.transform.rotation.y = quat[1];
        transform.transform.rotation.z = quat[2];
        transform.transform.rotation.w = quat[3];
        // 发布
        static_tf_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Static_Transform发送成功");
    }
    
private:
    std::shared_ptr<StaticTransformBroadcaster> static_tf_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTfBroadcaster>("static_tf");
    rclcpp::spin(node);
    rclcpp::shutdown();
}