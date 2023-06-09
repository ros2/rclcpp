// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rclcpp:msg/Header.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/header.hpp"


#ifndef RCLCPP__MSG__DETAIL__HEADER__BUILDER_HPP_
#define RCLCPP__MSG__DETAIL__HEADER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rclcpp/msg/detail/header__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rclcpp
{

namespace msg
{

namespace builder
{

class Init_Header_stamp
{
public:
  Init_Header_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rclcpp::msg::Header stamp(::rclcpp::msg::Header::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rclcpp::msg::Header msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rclcpp::msg::Header>()
{
  return rclcpp::msg::builder::Init_Header_stamp();
}

}  // namespace rclcpp

#endif  // RCLCPP__MSG__DETAIL__HEADER__BUILDER_HPP_
