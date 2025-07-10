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
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"
#include "rosidl_runtime_cpp/service_type_support_decl.hpp"

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// \brief Extract the package name, middle module, and type name from a full type string.
/// \details This function takes a full type string (e.g., "std_msgs/msg/String") and extracts
/// the package name, middle module (if any), and type name. The middle module is the part
/// between the package name and the type name, which is typically used for message types.
/// For example, for "std_msgs/msg/String", it returns ("std_msgs", "msg", "String").
/// \param[in] full_type
/// \throws std::runtime_error if the input full type string is malformed or does not follow the
/// expected format.
/// \return A tuple containing the package name, middle module (if any), and type name.
RCLCPP_PUBLIC
std::tuple<std::string, std::string, std::string>
extract_type_identifier(const std::string & full_type);

/// \brief Look for the library in the ament prefix paths and return the path to the type support
/// library.
/// \param[in] package_name The name of the package containing the type support library,
/// e.g. "std_msgs".
/// \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
/// \throws std::runtime_error if the library is not found.
/// \return The path to the type support library.
RCLCPP_PUBLIC
std::string get_typesupport_library_path(
  const std::string & package_name, const std::string & typesupport_identifier);

/// \brief Load the type support library for the given type.
/// \param[in] type The topic type, e.g. "std_msgs/msg/String"
/// \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
/// \throws std::runtime_error if the library is not found or cannot be loaded.
/// \return A shared library
RCLCPP_PUBLIC
std::shared_ptr<rcpputils::SharedLibrary>
get_typesupport_library(const std::string & type, const std::string & typesupport_identifier);

/// \brief Extract the type support handle from the library.
/// \note The library needs to match the topic type. The shared library must stay loaded for the
/// lifetime of the result.
/// \deprecated Use get_message_typesupport_handle() instead
/// \param[in] type The topic type, e.g. "std_msgs/msg/String"
/// \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
/// \param[in] library The shared type support library
/// \return A type support handle
[[deprecated("Use `get_message_typesupport_handle` instead")]]
RCLCPP_PUBLIC
const rosidl_message_type_support_t *
get_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library);

/// \brief Extracts the message type support handle from the library.
/// \note The library needs to match the topic type. The shared library must stay loaded for the
/// lifetime of the result.
/// \param[in] type The topic type, e.g. "std_msgs/msg/String"
/// \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
/// \param[in] library The shared type support library
/// \throws std::runtime_error if the symbol of type not found in the library.
/// \return A message type support handle
RCLCPP_PUBLIC
const rosidl_message_type_support_t *
get_message_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library);

/// \brief Extracts the service type support handle from the library.
/// \note The library needs to match the service type. The shared library must stay loaded for the
/// lifetime of the result.
/// \param[in] type The service type, e.g. "std_srvs/srv/Empty"
/// \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
/// \param[in] library The shared type support library
/// \throws std::runtime_error if the symbol of type not found in the library.
/// \return A service type support handle
RCLCPP_PUBLIC
const rosidl_service_type_support_t *
get_service_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library);

}  // namespace rclcpp

#endif  // RCLCPP__TYPESUPPORT_HELPERS_HPP_
