#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.h"                  // 提供 tf2::getEulerYPR 函数
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

using namespace std::chrono_literals;
using TransformListener = tf2_ros::TransformListener;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class DynamicTfBroadcaster : public rclcpp::Node
{
public:
DynamicTfBroadcaster(std::string name) : Node(name)
    {
        buffer_  = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);
        timer_ = this->create_wall_timer(1s, std::bind(&DynamicTfBroadcaster::get_transform, this));
    }

private:
    void get_transform(void)
    {   
        try {
            auto result = buffer_->lookupTransform("base_link", "target_point",
                rclcpp::Time(0.0f), rclcpp::Duration::from_seconds(1.0f));
    
            RCLCPP_INFO(this->get_logger(), "平移 %f %f %f", \
                result.transform.translation.x,
                result.transform.translation.y,
                result.transform.translation.z
            );
            RCLCPP_INFO(this->get_logger(), "旋转(四元数) %f %f %f %f", \
                result.transform.rotation.x,
                result.transform.rotation.y,
                result.transform.rotation.z,
                result.transform.rotation.w
            );
            double roll, pitch, yaw;
            const auto &q = result.transform.rotation;
            tf2::getEulerYPR(q, yaw, pitch, roll);
            RCLCPP_INFO(this->get_logger(), "旋转(欧拉角) %f %f %f", \
                roll,
                pitch,
                yaw
            );
        } catch(tf2::TransformException &e) {
            RCLCPP_WARN(this->get_logger(), "异常: %s", e.what());
        }
        
    }
    
private:
    std::shared_ptr<TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> buffer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTfBroadcaster>("tf_listener");
    rclcpp::spin(node);
    rclcpp::shutdown();
}