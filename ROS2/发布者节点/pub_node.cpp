#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;//方便直接使用时间

class Publisher : public rclcpp::Node
{
private:
    //时间和发布者共享指针
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    //回调函数
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello,robomaster";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

public:
    explicit Publisher(const std::string& node_name)
        : Node(node_name)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&Publisher::timer_callback, this));
    }
};

//创建并运行发布者节点
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Publisher>("public_rm");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}