// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rclcpp:msg/MessageWithHeader.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rclcpp/msg/detail/message_with_header__functions.h"
#include "rclcpp/msg/detail/message_with_header__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rclcpp
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MessageWithHeader_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rclcpp::msg::MessageWithHeader(_init);
}

void MessageWithHeader_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rclcpp::msg::MessageWithHeader *>(message_memory);
  typed_message->~MessageWithHeader();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MessageWithHeader_message_member_array[1] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rclcpp::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rclcpp::msg::MessageWithHeader, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MessageWithHeader_message_members = {
  "rclcpp::msg",  // message namespace
  "MessageWithHeader",  // message name
  1,  // number of fields
  sizeof(rclcpp::msg::MessageWithHeader),
  MessageWithHeader_message_member_array,  // message members
  MessageWithHeader_init_function,  // function to initialize message memory (memory has to be allocated)
  MessageWithHeader_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MessageWithHeader_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MessageWithHeader_message_members,
  get_message_typesupport_handle_function,
  &rclcpp__msg__MessageWithHeader__get_type_hash,
  &rclcpp__msg__MessageWithHeader__get_type_description,
  &rclcpp__msg__MessageWithHeader__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rclcpp


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rclcpp::msg::MessageWithHeader>()
{
  return &::rclcpp::msg::rosidl_typesupport_introspection_cpp::MessageWithHeader_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rclcpp, msg, MessageWithHeader)() {
  return &::rclcpp::msg::rosidl_typesupport_introspection_cpp::MessageWithHeader_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
