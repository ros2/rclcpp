#pragma once

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"

namespace rclcpp
{
namespace node
{

template<class MessageT>
class SimplePublisherNode : public rclcpp::Node
{
public:
  SimplePublisherNode( std::string node_name, std::string topic, size_t frequency ):
    Node(std::move(node_name), true),
    topic_(std::move(topic)),
    frequency_(frequency)
  {
    pub_ = this->create_publisher<MessageT>(topic_, rmw_qos_profile_default);
  }

  virtual ~SimplePublisherNode( )
  {
  }

  void run( )
  {
    auto callback = std::bind(&SimplePublisherNode::publish, this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frequency_), callback);
  }

  void stop( )
  {
    timer_->cancel();
  }

  virtual void publish( ) = 0;

protected:
  std::string topic_;
  size_t frequency_;

  typename rclcpp::Publisher<MessageT>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace node
} // namespace rclcpp
