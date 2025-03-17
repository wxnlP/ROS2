#include "rclcpp/rclcpp.hpp"
#include <string>

using  namespace std;
/*
    创建一个类节点，名字叫做TestNode,继承自Node.
*/
class TestNode : public rclcpp::Node
{

public:
    // 构造函数,有一个参数为节点名称
    TestNode(string name) : Node(name)
    {
        // 打印一句自我介绍
        RCLCPP_INFO(this->get_logger(), "节点的名称为:%s.",name.c_str());
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个Wang2的节点*/
    auto node = make_shared<TestNode>("helloworld_node");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
