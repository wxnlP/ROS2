#include "rclcpp/rclcpp.hpp"
// 注意头文件的格式，自定义接口目录：install/status_interfaces/include
#include "status_interfaces/msg/system_status.hpp"
// QT头文件
#include <QApplication>
#include <QLabel>
#include <QString>

// 自定义接口的命名空间
using SystemStatus =  status_interfaces::msg::SystemStatus;

class SysStatusSub : public rclcpp::Node
{
public:
    /* 构造函数 */
    SysStatusSub(std::string name) : Node(name)
    {
        // 创建发布者
        subscriber_ = this->create_subscription<SystemStatus>(
            name, 
            10,
            std::bind(&SysStatusSub::sub_callback, this, std::placeholders::_1));
        // 创建一个空的 SystemStatus 对象，转化成 QString 进行显示
        label_ = new QLabel(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->show();
    }

    /* 显示QString文本 */
    QString get_qstr_from_msg(SystemStatus::SharedPtr  msg) {
        std::stringstream show_str;
        show_str
            << "===========系统状态可视化显示工具============\n"
            << "数 据 时 间:\t" << msg->stamp.sec << "\ts\n"
            << "用  户  名:\t" << msg->host_name << "\t\n"
            << "CPU使用率:\t" << msg->cpu_percent << "\t%\n"
            << "内存使用率:\t" << msg->memory_percent << "\t%\n"
            << "内存总大小:\t" << msg->memory_total << "\tMB\n"
            << "剩余有效内存:\t" << msg->memory_available << "\tMB\n"
            << "网络发送量:\t" << msg->net_receive << "\tMB\n"
            << "网络接收量:\t" << msg->net_send<< "\tMB\n"
            << "==========================================";

        return QString::fromStdString(show_str.str());
    }
private:
    /* 订阅回调函数 */
    void sub_callback(const SystemStatus::SharedPtr msg) 
    {
        label_->setText(get_qstr_from_msg(msg));
    }
    /* 指针 */
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;
    QLabel* label_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusSub>("sys_status");
    // 使用lamda表达式创建一个线程单独执行ROS2节点
    std::thread spin_thread([&]() -> void { rclcpp::spin(node); });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}