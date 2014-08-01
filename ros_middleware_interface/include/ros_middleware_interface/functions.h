#ifndef __ros_middleware_interface__functions__h__
#define __ros_middleware_interface__functions__h__

#include "rosidl_generator_cpp/MessageTypeSupport.h"

#include "handles.h"

namespace ros_middleware_interface
{

NodeHandle create_node();

PublisherHandle create_publisher(const NodeHandle& node_handle, const rosidl_generator_cpp::MessageTypeSupportHandle & type_support_handle, const char * topic_name);

void publish(const PublisherHandle& publisher_handle, const void * ros_message);

}

#endif  // __ros_middleware_interface__functions__h__
