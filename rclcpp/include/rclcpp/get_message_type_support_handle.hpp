// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__GET_MESSAGE_TYPE_SUPPORT_HANDLE_HPP_
#define RCLCPP__GET_MESSAGE_TYPE_SUPPORT_HANDLE_HPP_

#include <type_traits>

#include "rosidl_runtime_cpp/traits.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rclcpp/type_adaptor.hpp"

/// Versions of rosidl_typesupport_cpp::get_message_type_support_handle that handle adapted types.

namespace rclcpp
{

/// Specialization for when MessageT is an actual ROS message type.
template<typename MessageT>
constexpr
typename std::enable_if_t<
  rosidl_generator_traits::is_message<MessageT>::value,
  const rosidl_message_type_support_t &
>
get_message_type_support_handle()
{
  return *rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>();
}

/// Specialization for when MessageT is an adapted type using rclcpp::TypeAdaptor.
template<typename AdaptedType>
constexpr
typename std::enable_if_t<
  !rosidl_generator_traits::is_message<AdaptedType>::value &&
  rclcpp::TypeAdaptor<AdaptedType>::is_specialized::value,
  const rosidl_message_type_support_t &
>
get_message_type_support_handle()
{
  return *rosidl_typesupport_cpp::get_message_type_support_handle<
    typename TypeAdaptor<AdaptedType>::ROSMessageType
  >();
}

/// Specialization for when MessageT is not a ROS message nor an adapted type.
/**
 * This specialization is a pass through runtime check, which allows a better
 * static_assert to catch this issue further down the line.
 * This should never get to be called in practice, and is purely defensive.
 */
template<
  typename AdaptedType
>
constexpr
typename std::enable_if_t<
  !rosidl_generator_traits::is_message<AdaptedType>::value &&
  !TypeAdaptor<AdaptedType>::is_specialized::value,
  const rosidl_message_type_support_t &
>
get_message_type_support_handle()
{
  throw std::runtime_error(
        "this specialization of rclcpp::get_message_type_support_handle() should never be called");
}

}  // namespace rclcpp

#endif  // RCLCPP__GET_MESSAGE_TYPE_SUPPORT_HANDLE_HPP_
