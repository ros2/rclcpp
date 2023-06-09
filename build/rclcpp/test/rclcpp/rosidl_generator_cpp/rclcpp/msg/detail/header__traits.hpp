// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rclcpp:msg/Header.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/header.hpp"


#ifndef RCLCPP__MSG__DETAIL__HEADER__TRAITS_HPP_
#define RCLCPP__MSG__DETAIL__HEADER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rclcpp/msg/detail/header__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rclcpp
{

namespace msg
{

inline void to_flow_style_yaml(
  const Header & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Header & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Header & msg, bool use_flow_style = false)
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
  const rclcpp::msg::Header & msg,
  std::ostream & out, size_t indentation = 0)
{
  rclcpp::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rclcpp::msg::to_yaml() instead")]]
inline std::string to_yaml(const rclcpp::msg::Header & msg)
{
  return rclcpp::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rclcpp::msg::Header>()
{
  return "rclcpp::msg::Header";
}

template<>
inline const char * name<rclcpp::msg::Header>()
{
  return "rclcpp/msg/Header";
}

template<>
struct has_fixed_size<rclcpp::msg::Header>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<rclcpp::msg::Header>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<rclcpp::msg::Header>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCLCPP__MSG__DETAIL__HEADER__TRAITS_HPP_
