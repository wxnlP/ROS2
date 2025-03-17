#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MiniSubscriber : public rclcpp::Node
{
public:
    MiniSubscriber(std::string name) : Node(name)
    {
        // 创建订阅者
        subscription = this->create_subscription<std_msgs::msg::String>(
            "TopicTest",
            10,
            std::bind(&MiniSubscriber::callback, this, std::placeholders::_1));
    }

private:
    void callback(const std_msgs::msg::String & msg) const
    {
        // 输出订阅话题的信息
        RCLCPP_INFO(this->get_logger(), "订阅的消息:'%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MiniSubscriber>("subscriber_cpp_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}