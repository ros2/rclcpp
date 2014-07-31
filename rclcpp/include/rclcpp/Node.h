#ifndef __rclcpp__Node__h__
#define __rclcpp__Node__h__

#include "Publisher.h"

#include "ros_middleware_interface/functions.h"


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
    const rosidl_generator_cpp::MessageTypeSupportMembers & members = rosidl_generator_cpp::MessageTypeSupport<ROSMessage>::get_members();
    void * publisher_handle = ::ros_middleware_interface::create_publisher(node_handle_, members, topic_name);
    return new Publisher<ROSMessage>(publisher_handle);
  }

private:
  void * node_handle_;
};

}

rclcpp::Node* create_node()
{
  return new rclcpp::Node();
}


#endif  // __rclcpp__Node__h__
