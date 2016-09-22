#pragma once

#include "rclcpp/node.hpp"

namespace rclcpp
{
namespace node
{

template<class MessageT>
class SimpleSubscriberNode : public rclcpp::Node
{
public:
  SimpleSubscriberNode( std::string node_name, std::string topic ):
    Node(std::move(node_name), true),
    topic_(std::move(topic))
  {
  }

  void run( )
  {
    auto callback = std::bind(&SimpleSubscriberNode::callback, this, std::placeholders::_1);
    sub_ = this->create_subscription<MessageT>(topic_, callback);
    printf("Going to setup subscriber on topic %s\n", topic_.c_str());
  }

  virtual void callback( typename MessageT::UniquePtr msg ) = 0;

protected:
  std::string topic_;

  typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
};

} //namespace node
} //namespace rclcpp
