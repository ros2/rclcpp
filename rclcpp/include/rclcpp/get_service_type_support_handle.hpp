// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__GET_SERVICE_TYPE_SUPPORT_HANDLE_HPP_
#define RCLCPP__GET_SERVICE_TYPE_SUPPORT_HANDLE_HPP_

#include <stdexcept>
#include <type_traits>

#include "rosidl_runtime_cpp/traits.hpp"
#include "rosidl_runtime_cpp/service_type_support_decl.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"

#include "rclcpp/type_adapter.hpp"

namespace rclcpp
{

#ifdef DOXYGEN_ONLY

/// Returns the service type support for the given `ServiceT` type.
/**
 * \tparam ServiceT an actual ROS service type or an adapted type using `rclcpp::TypeAdapter`
 */
template<typename ServiceT>
constexpr const rosidl_service_type_support_t & get_service_type_support_handle();

#else

template<typename ServiceT>
constexpr
std::enable_if_t<
  rosidl_generator_traits::is_service<ServiceT>::value,
  const rosidl_service_type_support_t &
>
get_service_type_support_handle()
{
  auto handle = rosidl_typesupport_cpp::get_service_type_support_handle<ServiceT>();
  if (!handle) {
    throw std::runtime_error("Type support handle unexpectedly nullptr");
  }
  return *handle;
}

/// The following checks will fix for the 4 different ways you could assemble
/// the TypeAdapter struct when using custom or ROS types.
template<typename AdaptedTypeStruct>
constexpr
std::enable_if_t<
  !rosidl_generator_traits::is_service<AdaptedTypeStruct>::value &&
  rclcpp::TypeAdapter<typename AdaptedTypeStruct::Request>::is_specialized::value &&
  rclcpp::TypeAdapter<typename AdaptedTypeStruct::Response>::is_specialized::value,
  const rosidl_service_type_support_t &
>
get_service_type_support_handle()
{
  struct AdaptedTypeStructTemp
  {
      using Request =
        typename rclcpp::TypeAdapter<typename AdaptedTypeStruct::Request>::ros_message_type;
      using Response =
        typename rclcpp::TypeAdapter<typename AdaptedTypeStruct::Response>::ros_message_type;
  };

  auto handle = rosidl_typesupport_cpp::get_service_type_support_handle<AdaptedTypeStructTemp
    >();
  if (!handle) {
    throw std::runtime_error("Type support handle unexpectedly nullptr");
  }
  return *handle;
}

template<typename AdaptedTypeStruct>
constexpr
std::enable_if_t<
  !rosidl_generator_traits::is_service<AdaptedTypeStruct>::value &&
  !rclcpp::TypeAdapter<typename AdaptedTypeStruct::Request>::is_specialized::value &&
  rclcpp::TypeAdapter<typename AdaptedTypeStruct::Response>::is_specialized::value,
  const rosidl_service_type_support_t &
>
get_service_type_support_handle()
{
  struct AdaptedTypeStructTemp
  {
      using Request =
        typename rclcpp::TypeAdapter<typename AdaptedTypeStruct::Request>::custom_type;
      using Response =
        typename rclcpp::TypeAdapter<typename AdaptedTypeStruct::Response>::ros_message_type;
  };

  auto handle = rosidl_typesupport_cpp::get_service_type_support_handle<AdaptedTypeStructTemp
    >();
  if (!handle) {
    throw std::runtime_error("Type support handle unexpectedly nullptr");
  }
  return *handle;
}

template<typename AdaptedTypeStruct>
constexpr
std::enable_if_t<
  !rosidl_generator_traits::is_service<AdaptedTypeStruct>::value &&
  rclcpp::TypeAdapter<typename AdaptedTypeStruct::Request>::is_specialized::value &&
  !rclcpp::TypeAdapter<typename AdaptedTypeStruct::Response>::is_specialized::value,
  const rosidl_service_type_support_t &
>
get_service_type_support_handle()
{
  struct AdaptedTypeStructTemp
  {
      using Request =
        typename rclcpp::TypeAdapter<typename AdaptedTypeStruct::Request>::ros_message_type;
      using Response =
        typename rclcpp::TypeAdapter<typename AdaptedTypeStruct::Response>::custom_type;
  };

  auto handle = rosidl_typesupport_cpp::get_service_type_support_handle<AdaptedTypeStructTemp
    >();
  if (!handle) {
    throw std::runtime_error("Type support handle unexpectedly nullptr");
  }
  return *handle;
}

// This specialization is a pass through runtime check, which allows a better
// static_assert to catch this issue further down the line.
// This should never get to be called in practice, and is purely defensive.
template<typename AdaptedTypeStruct>
constexpr
typename std::enable_if_t<
  !rosidl_generator_traits::is_service<AdaptedTypeStruct>::value &&
  !TypeAdapter<typename AdaptedTypeStruct::Request>::is_specialized::value &&
  !TypeAdapter<typename AdaptedTypeStruct::Response>::is_specialized::value,
  const rosidl_service_type_support_t &
>
get_service_type_support_handle()
{
  throw std::runtime_error(
          "this specialization of rclcpp::get_service_type_support_handle() "
          "should never be called");
}

#endif  // DOXYGEN_ONLY

}  // namespace rclcpp

#endif  // RCLCPP__GET_SERVICE_TYPE_SUPPORT_HANDLE_HPP_
