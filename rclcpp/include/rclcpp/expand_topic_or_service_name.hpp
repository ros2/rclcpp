// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPAND_TOPIC_OR_SERVICE_NAME_HPP_
#define RCLCPP__EXPAND_TOPIC_OR_SERVICE_NAME_HPP_

#include <string>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Expand a topic or service name and throw if it is not valid.
/**
 * This function can be used to "just" validate a topic or service name too,
 * since expanding the topic name is required to fully validate a name.
 *
 * If the name is invalid, then InvalidTopicNameError is thrown or
 * InvalidServiceNameError if is_service is true.
 *
 * This function can take any form of a topic or service name, i.e. it does not
 * have to be a fully qualified name.
 * The node name and namespace are used to expand it if necessary while
 * validating it.
 *
 * Expansion is done with rcl_expand_topic_name.
 * The validation is doen with rcl_validate_topic_name and
 * rmw_validate_full_topic_name, so details about failures can be found in the
 * documentation for those functions.
 *
 * \param name the topic or service name to be validated
 * \param node_name the name of the node associated with the name
 * \param namespace_ the namespace of the node associated with the name
 * \param is_service if true InvalidServiceNameError is thrown instead
 * \returns expanded (and validated) topic name
 * \throws InvalidTopicNameError if name is invalid and is_service is false
 * \throws InvalidServiceNameError if name is invalid and is_service is true
 * \throws std::bad_alloc if memory cannot be allocated
 * \throws RCLError if an unexpect error occurs
 * \throws std::runtime_error if the topic name is unexpectedly valid or,
 *    if the rcl name is invalid or if the rcl namespace is invalid
 */
RCLCPP_PUBLIC
std::string
expand_topic_or_service_name(
  const std::string & name,
  const std::string & node_name,
  const std::string & namespace_,
  bool is_service = false);

}  // namespace rclcpp

#endif  // RCLCPP__EXPAND_TOPIC_OR_SERVICE_NAME_HPP_
