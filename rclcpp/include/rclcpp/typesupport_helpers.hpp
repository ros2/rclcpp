// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#ifndef RCLCPP__TYPESUPPORT_HELPERS_HPP_
#define RCLCPP__TYPESUPPORT_HELPERS_HPP_

#include <memory>
#include <string>
#include <tuple>

#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/action_type_support_decl.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"
#include "rosidl_runtime_cpp/service_type_support_decl.hpp"

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
/// Load the type support library for the given type.
/**
 * \param[in] type The topic type, e.g. "std_msgs/msg/String"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
 * \return A shared library
 */
RCLCPP_PUBLIC
std::shared_ptr<rcpputils::SharedLibrary>
get_typesupport_library(const std::string & type, const std::string & typesupport_identifier);

namespace internal
{

struct typesupport_message_tag {};
struct typesupport_service_tag {};
struct typesupport_action_tag {};

template<typename T>
struct typesupport_traits
{
  using type_tag = T;
};

template<>
struct typesupport_traits<rosidl_message_type_support_t>
{
  using type_tag = typesupport_message_tag;
};

template<>
struct typesupport_traits<rosidl_service_type_support_t>
{
  using type_tag = typesupport_service_tag;
};

template<>
struct typesupport_traits<rosidl_action_type_support_t>
{
  using type_tag = typesupport_action_tag;
};


RCLCPP_PUBLIC
const rosidl_message_type_support_t * _get_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library,
  typesupport_message_tag
);

RCLCPP_PUBLIC
const rosidl_service_type_support_t * _get_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library,
  typesupport_service_tag
);

RCLCPP_PUBLIC
const rosidl_action_type_support_t * _get_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library,
  typesupport_action_tag
);

}  // namespace internal

/// Extract the type support handle from the library.
/**
 * The library needs to match the type of topic, service or action.
 * The shared library must stay loaded for the lifetime of the result.
 *
 * The template parameter can be `rosidl_message_type_support_t`, `rosidl_service_type_support_t`
 * and `rosidl_action_type_support_t`.
 *
 * \param[in] type The topic type, e.g. "std_msgs/msg/String"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
 * \param[in] library The shared type support library
 * \return A type support handle
 * \throws std::runtime_error if library could not be found.
 */
template<typename T = rosidl_message_type_support_t>
const T *
get_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library)
{
  return internal::_get_typesupport_handle(
    type, typesupport_identifier, library, typename internal::typesupport_traits<T>::type_tag());
}

}  // namespace rclcpp

#endif  // RCLCPP__TYPESUPPORT_HELPERS_HPP_
