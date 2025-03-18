#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"


using namespace std::chrono_literals;
using TransformBroadcaster = tf2_ros::TransformBroadcaster;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class DynamicTfBroadcaster : public rclcpp::Node
{
public:
DynamicTfBroadcaster(std::string name) : Node(name)
    {
        dynamic_tf_ = std::make_shared<TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(1s, std::bind(&DynamicTfBroadcaster::publish_dynamic_tf, this));
    }

private:
    void publish_dynamic_tf(void)
    {
        TransformStamped transform;
        // 时间戳
        transform.header.stamp = this->get_clock()->now();
        // 父坐标系名称
        transform.header.frame_id = "map";
        // 子坐标系名称
        transform.child_frame_id = "base_link";
        // 平面移动
        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;
        // 欧拉角(弧度)转四元数
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 30 * M_PI / 180);
        // 旋转
        transform.transform.rotation.x = quat[0];
        transform.transform.rotation.y = quat[1];
        transform.transform.rotation.z = quat[2];
        transform.transform.rotation.w = quat[3];
        // 发布
        dynamic_tf_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Static_Transform发送成功");
    }
    
private:
    std::shared_ptr<TransformBroadcaster> dynamic_tf_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTfBroadcaster>("dyanmic_tf");
    rclcpp::spin(node);
    rclcpp::shutdown();
}