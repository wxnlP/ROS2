#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class MiniPublisher : public rclcpp::Node
{

public:
    // 创建构造函数，类定义时自动调用
    // 这里的那么为：MiniPublisher传入的name和Node的name为同一个值
    MiniPublisher(std::string name): Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点的名称为:%s.",name.c_str());
        // 创建发布者，话题为"TopicTest"，发布 std_msgs::msg::String 数据类型， 消息深度为10
        publisher_cpp = this->create_publisher<std_msgs::msg::String>("TopicTest", 10);
        timer = this->create_wall_timer(500ms, std::bind(&MiniPublisher::callback, this));

    }


private:
    void callback(void)
    {
        // 创建发布的数据
        auto msg = std_msgs::msg::String();
        msg.data = "这是一个C++发布者信息" + std::to_string(count++);
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
        publisher_cpp->publish(msg);
    }
    // 定义使用到的变量
    int count=0;
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer;
    // 声明话题发布者指针
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_cpp;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MiniPublisher>("publisher_cpp_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}