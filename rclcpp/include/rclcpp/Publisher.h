#ifndef __rclcpp__Publisher__h__
#define __rclcpp__Publisher__h__

#include "ros_middleware_interface/functions.h"


namespace rclcpp
{

template<typename ROSMessage>
class Publisher
{
public:
  Publisher(void * publisher_handle)
    : publisher_handle_(publisher_handle)
  {}

  void publish(const ROSMessage& ros_message)
  {
    ::ros_middleware_interface::publish(publisher_handle_, &ros_message);
  }

private:
  void * publisher_handle_;
};

}

#endif  // __rclcpp__Publisher__h__
