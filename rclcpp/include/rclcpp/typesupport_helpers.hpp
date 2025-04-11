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

/// Extract the message type support handle from the library.
/**
 * The library needs to match the topic type. The shared library must stay loaded for the lifetime of the result.
 *
 * \param[in] type The topic type, e.g. "std_msgs/msg/String"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
 * \param[in] library The shared type support library
 * \throws std::runtime_error if the symbol of type not found in the library.
 * \return A message type support handle
 */
RCLCPP_PUBLIC
const rosidl_message_type_support_t *
get_message_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library);

/// Extract the service type support handle from the library.
/**
 * The library needs to match the service type. The shared library must stay loaded for the lifetime of the result.
 *
 * \param[in] type The service type, e.g. "std_srvs/srv/Empty"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
 * \param[in] library The shared type support library
 * \throws std::runtime_error if the symbol of type not found in the library.
 * \return A service type support handle
 */
RCLCPP_PUBLIC
const rosidl_service_type_support_t *
get_service_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library);

/// Extract the action type support handle from the library.
/**
 * The library needs to match the action type. The shared library must stay loaded for the lifetime
 * of the result.
 *
 * \param[in] type The action type, e.g. "example_interfaces/action/Fibonacci"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
 * \param[in] library The shared type support library
 * \throws std::runtime_error if the symbol of type not found in the library.
 * \return A action type support handle
 */
RCLCPP_PUBLIC
const rosidl_action_type_support_t *
get_action_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library);

}  // namespace rclcpp

#endif  // RCLCPP__TYPESUPPORT_HELPERS_HPP_
