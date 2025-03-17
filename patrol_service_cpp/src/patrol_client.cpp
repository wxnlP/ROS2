#include "rclcpp/rclcpp.hpp"
#include "detect_interfaces/srv/patrol.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using namespace std::chrono_literals;
using Patrol = detect_interfaces::srv::Patrol;
using SetParameters = rcl_interfaces::srv::SetParameters;
using Parameter = rcl_interfaces::msg::Parameter;
using ParameterValue = rcl_interfaces::msg::ParameterValue;

class PatrolClient : public rclcpp::Node
{
public:
    PatrolClient(std::string name) : Node(name)
    {
        patrol_client_ = this->create_client<Patrol>("/patrol");
        timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
    }
    /* 设置参数 */
    std::shared_ptr<SetParameters::Response> set_patrol_parameter(Parameter &parameters)
    {
        // 1.创建参数客户端，并等待服务上线
        auto param_client = this->create_client<SetParameters>("/turtle_controller/set_parameters");
        while(!param_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务被打断。。。");
                return nullptr;
            } else {
                RCLCPP_INFO(this->get_logger(), "等待参数服务上线。。。");
            }
        }
        // 2.构造Request(由于消息接口复杂，故消息赋值单开一个函数完成)
        auto request = std::make_shared<SetParameters::Request>();
        request->parameters.push_back(parameters);
        // 3.发送请求并等待服务端处理完成(异步获取结果)
        auto future = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        return response;
    }

    void update_patrol_parameter(double k)
    {
        // 1.创建一个参数对象
        auto param = Parameter();
        param.name = "k";
        // 2.创建一个参数值对象
        auto param_value = ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        // 3.赋值参数对象
        param.value = param_value;
        // 4.发送更新参数请求
        auto response = set_patrol_parameter(param);
        if (response == nullptr) {
            RCLCPP_WARN(this->get_logger(), "参数修改失败");
            return;
        } else {
            // 循环取出结果，判断successful
            for (auto result : response->results) {
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), "参数k 已修改为：%f", k);
                } else {
                    RCLCPP_WARN(this->get_logger(), "参数k 失败原因：%s", result.reason.c_str());
                }
            }
        }
    }

private:
    void timer_callback(void)
    {
        while(!patrol_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务被打断。。。");
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "等待服务上线。。。");
            }
        }
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = rand()%15;
        request->target_y = rand()%15;
        RCLCPP_INFO(this->get_logger(), "请求巡逻：(%f,%f)", request->target_x, request->target_y);
        // 异步发送请求
        patrol_client_->async_send_request(
            request,
            // lamda表示式回调函数
            [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void
            {
                auto response = result_future.get();
                if (response->result == Patrol::Response::SUCCESS)
                {
                    RCLCPP_INFO(this->get_logger(), "目标点处理成功");
                }
                else if (response->result == Patrol::Response::FAIL)
                {
                    RCLCPP_INFO(this->get_logger(), "目标点处理失败");
                }
            });
    }

private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    rclcpp::TimerBase::SharedPtr timer_; 
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>("patrol_client");
    // 修改参数
    node->update_patrol_parameter(1.5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}