#ifndef __rclcpp__Subscriber__h__
#define __rclcpp__Subscriber__h__

#include <functional>
#include <string>

#include "ros_middleware_interface/functions.h"
#include "ros_middleware_interface/handles.h"


namespace rclcpp
{

class Node;

namespace executor
{
  class SingleThreadExecutor;
}

class SubscriberInterface
{
  friend class rclcpp::executor::SingleThreadExecutor;
public:
  SubscriberInterface(const ros_middleware_interface::SubscriberHandle &subscriber_handle, std::string topic_name)
    : subscriber_handle_(subscriber_handle), topic_name_(topic_name)
  {}
private:
  ros_middleware_interface::SubscriberHandle subscriber_handle_;
  std::string topic_name_;

};

template<typename ROSMessage>
class Subscriber : public SubscriberInterface
{
  friend class rclcpp::Node;
public:
  typedef std::function<void(ROSMessage *)> CallbackType;
  Subscriber(const ros_middleware_interface::SubscriberHandle &subscriber_handle, std::string topic_name)
    : SubscriberInterface(subscriber_handle, topic_name)
  {}
};

}

#endif  // __rclcpp__Subscriber__h__
