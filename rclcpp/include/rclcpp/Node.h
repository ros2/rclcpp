#ifndef __rclcpp__Node__h__
#define __rclcpp__Node__h__

#include "Publisher.h"

#include "ros_middleware_interface/functions.h"
#include "ros_middleware_interface/handles.h"

#include "ros_middleware_interface/get_type_support_handle.h"


namespace rclcpp
{

class Node
{
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

private:
  ros_middleware_interface::NodeHandle node_handle_;
};

}

rclcpp::Node* create_node()
{
  return new rclcpp::Node();
}


#endif  // __rclcpp__Node__h__
