#ifndef __rclcpp__Node__h__
#define __rclcpp__Node__h__

#include <memory>
#include <string>
#include <vector>

#include "Publisher.h"
#include "Subscriber.h"

#include "ros_middleware_interface/functions.h"
#include "ros_middleware_interface/handles.h"

#include "ros_middleware_interface/get_type_support_handle.h"


namespace rclcpp
{

namespace executor
{
  class SingleThreadExecutor;
}

class Node
{
  friend class rclcpp::executor::SingleThreadExecutor;
public:
  Node()
  {
    node_handle_ = ::ros_middleware_interface::create_node();
  }

  template<typename ROSMessage>
  Publisher<ROSMessage>* create_publisher(const char * topic_name)
  {
    const rosidl_generator_cpp::MessageTypeSupportHandle & type_support_handle = ::ros_middleware_interface::get_type_support_handle<ROSMessage>();
    ros_middleware_interface::PublisherHandle publisher_handle = ::ros_middleware_interface::create_publisher(node_handle_, type_support_handle, topic_name);
    return new Publisher<ROSMessage>(publisher_handle);
  }

  template<typename ROSMessage>
  Subscriber<ROSMessage>* create_subscriber(const char * topic_name)
  {
    const rosidl_generator_cpp::MessageTypeSupportHandle & type_support_handle = ::ros_middleware_interface::get_type_support_handle<ROSMessage>();
    ros_middleware_interface::SubscriberHandle subscriber_handle = ::ros_middleware_interface::create_subscriber(node_handle_, type_support_handle, topic_name);
    SubscriberInterface *sub = new Subscriber<ROSMessage>(subscriber_handle, std::string(topic_name));
    this->subscribers_.push_back(sub);
    return static_cast<Subscriber<ROSMessage> *>(sub);
  }

private:
  ros_middleware_interface::NodeHandle node_handle_;
  std::vector<SubscriberInterface *> subscribers_;

};

rclcpp::Node* create_node()
{
  return new rclcpp::Node();
}

} /* namespace rclcpp */


#endif  // __rclcpp__Node__h__
