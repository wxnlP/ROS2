#include "rclcpp/rclcpp.hpp"
// 小海龟 /turtle1/cmd_vel 话题(速度控制)消息接口
#include "geometry_msgs/msg/twist.hpp"
// 小海龟 /turtle1/pose 话题(位置、速度获取)消息接口
#include "turtlesim/msg/pose.hpp"
#include "detect_interfaces/srv/patrol.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using Patrol = detect_interfaces::srv::Patrol;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        // 声明和设置参数
        this->declare_parameter("k", 1.0);
        this->declare_parameter("max_speed", 3.0);
        this->get_parameter("k", k_);
        this->get_parameter("max_speed", max_speed_);
        // 发布者，发布小海龟的线速度、角速度
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        // 订阅者订阅小海龟的位置、线速度、角速度
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleController::on_pose_received_, this, std::placeholders::_1));
        // 创建服务端
        patrol_server_ = this->create_service<Patrol>(
            "/patrol",
            std::bind(&TurtleController::patrol_callback, this, std::placeholders::_1, std::placeholders::_2));
        // 添加参数回调函数
        parameters_callback_handle_ = this->add_on_set_parameters_callback(
            [&](const std::vector<rclcpp::Parameter> &params) -> SetParametersResult
            {
                for (const auto &param : params)
                {
                    // 打印日志，c_str()将C++字符串转为C语言字符串
                    RCLCPP_INFO(this->get_logger(), "更新参数 %s 值为：%f", param.get_name().c_str(), param.as_double());
                    // 判断参数名称
                    if (param.get_name() == "k")
                    {
                        // 取出浮点型的参数值
                        k_ = param.as_double();
                    }
                    else if (param.get_name() == "max_speed")
                    {
                        // 取出浮点型的参数值
                        max_speed_ = param.as_double();
                    }
                }
                // 返回结果
                auto result = SetParametersResult();
                result.successful = true;
                return result;
            });
    }

private:
    /* 服务端回调函数 */
    void patrol_callback(const Patrol::Request::SharedPtr request, const Patrol::Response::SharedPtr response)
    {
        // 判断请求的目标位置是否在0~13
        if ((0 < request->target_x && request->target_x < 12.0f) && (0 < request->target_y && request->target_y < 12.0f))
        {
            // 设置目标值
            target_x_ = request->target_x;
            target_y_ = request->target_y;
            // 返回成功
            response->result = Patrol::Response::SUCCESS;
        }
        else
        {
            // 返回失败
            response->result = Patrol::Response::FAIL;
        }
    }

    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto message = geometry_msgs::msg::Twist();
        // 1.记录当前位置
        double current_x = pose->x;
        double current_y = pose->y;
        RCLCPP_INFO(this->get_logger(), "当前位置:(x=%f,y=%f)", current_x, current_y);

        // 2.计算距离目标的距离，与当前海龟朝向的角度差
        // 勾股定理计算直线距离
        double distance =
            std::sqrt((target_x_ - current_x) * (target_x_ - current_x) +
                      (target_y_ - current_y) * (target_y_ - current_y));
        // 目标方向的角度 - 当前朝向角度，得到夹角差
        double angle =
            std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

        // 3.控制策略：距离大于0.1继续运动，角度差大于0.2则原地旋转，否则直行
        if (distance > 0.1)
        {
            if (fabs(angle) > 0.2)
            {
                // 以角度差的绝对值为角速度进行原地旋转
                message.angular.z = fabs(angle);
            }
            else
            {
                // 通过比例控制器计算输出线速度
                message.linear.x = k_ * distance;
            }
        }

        // 4.限制最大值并发布消息
        if (message.linear.x > max_speed_)
        {
            message.linear.x = max_speed_;
        }
        velocity_publisher_->publish(message);
    }
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Service<Patrol>::SharedPtr patrol_server_;
    double target_x_{1.0};  // 目标位置X,设置默认值1.0
    double target_y_{2.0};  // 目标位置Y,设置默认值1.0
    double k_{1.0};         // 比例系数，控制输出=误差*比例系数
    double max_speed_{3.0}; // 最大线速度，设置默认值3.0
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}