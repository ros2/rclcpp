// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef RCLCPP_GENERIC__TYPESUPPORT_HELPERS_HPP_
#define RCLCPP_GENERIC__TYPESUPPORT_HELPERS_HPP_

#include <memory>
#include <string>
#include <tuple>

#include "rclcpp_generic/visibility_control.hpp"

#include "rcpputils/shared_library.hpp"

#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

namespace rclcpp_generic
{

RCLCPP_GENERIC_PUBLIC
std::shared_ptr<rcpputils::SharedLibrary>
get_typesupport_library(const std::string & type, const std::string & typesupport_identifier);

RCLCPP_GENERIC_PUBLIC
const rosidl_message_type_support_t *
get_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  std::shared_ptr<rcpputils::SharedLibrary> library);

RCLCPP_GENERIC_PUBLIC
std::tuple<std::string, std::string, std::string>
extract_type_identifier(const std::string & full_type);

}  // namespace rclcpp_generic

#endif  // RCLCPP_GENERIC__TYPESUPPORT_HELPERS_HPP_
