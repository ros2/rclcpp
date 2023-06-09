// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rclcpp:msg/String.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/string.hpp"


#ifndef RCLCPP__MSG__DETAIL__STRING__TRAITS_HPP_
#define RCLCPP__MSG__DETAIL__STRING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rclcpp/msg/detail/string__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rclcpp
{

namespace msg
{

inline void to_flow_style_yaml(
  const String & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const String & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const String & msg, bool use_flow_style = false)
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
  const rclcpp::msg::String & msg,
  std::ostream & out, size_t indentation = 0)
{
  rclcpp::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rclcpp::msg::to_yaml() instead")]]
inline std::string to_yaml(const rclcpp::msg::String & msg)
{
  return rclcpp::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rclcpp::msg::String>()
{
  return "rclcpp::msg::String";
}

template<>
inline const char * name<rclcpp::msg::String>()
{
  return "rclcpp/msg/String";
}

template<>
struct has_fixed_size<rclcpp::msg::String>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rclcpp::msg::String>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rclcpp::msg::String>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCLCPP__MSG__DETAIL__STRING__TRAITS_HPP_
