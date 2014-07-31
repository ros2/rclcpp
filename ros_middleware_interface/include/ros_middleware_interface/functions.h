#ifndef __ros_middleware_interface__functions__h__
#define __ros_middleware_interface__functions__h__

#include "rosidl_generator_cpp/MessageTypeSupport.h"

namespace ros_middleware_interface
{

void * create_node();

void * create_publisher(void * node, const rosidl_generator_cpp::MessageTypeSupportMembers & members, const char * topic_name);

void publish(void * publisher, const void * ros_message);

}

#endif  // __ros_middleware_interface__functions__h__
