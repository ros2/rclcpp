// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rclcpp:msg/MessageWithHeader.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/message_with_header.hpp"


#ifndef RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__BUILDER_HPP_
#define RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rclcpp/msg/detail/message_with_header__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rclcpp
{

namespace msg
{

namespace builder
{

class Init_MessageWithHeader_header
{
public:
  Init_MessageWithHeader_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rclcpp::msg::MessageWithHeader header(::rclcpp::msg::MessageWithHeader::_header_type arg)
  {
    msg_.header = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rclcpp::msg::MessageWithHeader msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rclcpp::msg::MessageWithHeader>()
{
  return rclcpp::msg::builder::Init_MessageWithHeader_header();
}

}  // namespace rclcpp

#endif  // RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__BUILDER_HPP_
