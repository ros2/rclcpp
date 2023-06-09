// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rclcpp:msg/String.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/string.hpp"


#ifndef RCLCPP__MSG__DETAIL__STRING__BUILDER_HPP_
#define RCLCPP__MSG__DETAIL__STRING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rclcpp/msg/detail/string__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rclcpp
{

namespace msg
{

namespace builder
{

class Init_String_data
{
public:
  Init_String_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rclcpp::msg::String data(::rclcpp::msg::String::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rclcpp::msg::String msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rclcpp::msg::String>()
{
  return rclcpp::msg::builder::Init_String_data();
}

}  // namespace rclcpp

#endif  // RCLCPP__MSG__DETAIL__STRING__BUILDER_HPP_
