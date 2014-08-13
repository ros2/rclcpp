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
  virtual void * create_message() = 0;

  virtual void delete_message(void * ros_message) = 0;

  virtual void handle_message(void * ros_message) = 0;

  ros_middleware_interface::SubscriberHandle subscriber_handle_;
  std::string topic_name_;

};

template<typename ROSMessage>
class Subscriber : public SubscriberInterface
{
  friend class rclcpp::Node;
public:
  typedef std::function<void(const ROSMessage *)> CallbackType;
  Subscriber(const ros_middleware_interface::SubscriberHandle &subscriber_handle, std::string topic_name, CallbackType callback)
    : SubscriberInterface(subscriber_handle, topic_name), callback_(callback)
  {}

private:
  void * create_message()
  {
    return new ROSMessage();
  }

  void delete_message(void * ros_message)
  {
    ROSMessage* msg = (ROSMessage*)ros_message;
    delete msg;
    ros_message = 0;
  }

  void handle_message(void * ros_message)
  {
    ROSMessage* msg = (ROSMessage*)ros_message;
    callback_(msg);
  }

  CallbackType callback_;

};

}

#endif  // __rclcpp__Subscriber__h__
