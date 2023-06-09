// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rclcpp:msg/MessageWithHeader.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/message_with_header.hpp"


#ifndef RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__TRAITS_HPP_
#define RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rclcpp/msg/detail/message_with_header__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "rclcpp/msg/detail/header__traits.hpp"

namespace rclcpp
{

namespace msg
{

inline void to_flow_style_yaml(
  const MessageWithHeader & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MessageWithHeader & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MessageWithHeader & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rclcpp

namespace rosidl_generator_traits
{

[[deprecated("use rclcpp::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rclcpp::msg::MessageWithHeader & msg,
  std::ostream & out, size_t indentation = 0)
{
  rclcpp::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rclcpp::msg::to_yaml() instead")]]
inline std::string to_yaml(const rclcpp::msg::MessageWithHeader & msg)
{
  return rclcpp::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rclcpp::msg::MessageWithHeader>()
{
  return "rclcpp::msg::MessageWithHeader";
}

template<>
inline const char * name<rclcpp::msg::MessageWithHeader>()
{
  return "rclcpp/msg/MessageWithHeader";
}

template<>
struct has_fixed_size<rclcpp::msg::MessageWithHeader>
  : std::integral_constant<bool, has_fixed_size<rclcpp::msg::Header>::value> {};

template<>
struct has_bounded_size<rclcpp::msg::MessageWithHeader>
  : std::integral_constant<bool, has_bounded_size<rclcpp::msg::Header>::value> {};

template<>
struct is_message<rclcpp::msg::MessageWithHeader>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__TRAITS_HPP_
