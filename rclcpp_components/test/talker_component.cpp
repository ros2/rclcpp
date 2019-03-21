#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace test_rclcpp_components
{
class Talker : public rclcpp::Node
{
public:
  Talker(std::string node_name, std::string node_namespace, rclcpp::NodeOptions options)
  : Node(node_name, node_namespace, options),
    count_(0)
  {
    // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
    pub_ = create_publisher<std_msgs::msg::String>("chatter");

    if(!get_parameter<std::string>("message", message_))
    {
      message_ = "Hello World: ";
    }

    // Use a timer to schedule periodic message publishing.
    timer_ = create_wall_timer(1s, std::bind(&Talker::on_timer, this));
  }

  void on_timer()
  {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = message_ + std::to_string(++count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
    std::flush(std::cout);

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(msg);
  }

private:
  std::string message_;
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(test_rclcpp_components::Talker)
