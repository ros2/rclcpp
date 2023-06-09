// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from rclcpp:msg/Header.idl
// generated code does not contain a copyright notice

#ifndef RCLCPP__MSG__DETAIL__HEADER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define RCLCPP__MSG__DETAIL__HEADER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rclcpp/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "rclcpp/msg/detail/header__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace rclcpp
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
cdr_serialize(
  const rclcpp::msg::Header & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rclcpp::msg::Header & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
get_serialized_size(
  const rclcpp::msg::Header & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rclcpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rclcpp, msg, Header)();

#ifdef __cplusplus
}
#endif

#endif  // RCLCPP__MSG__DETAIL__HEADER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
