// Copyright 2023 Intrisnic.ai
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

#ifndef RCLCPP__LIST_PARAMETER_OVERRIDE_PREFIXES_HPP_
#define RCLCPP__LIST_PARAMETER_OVERRIDE_PREFIXES_HPP_

#include <map>
#include <string>
#include <unordered_set>

#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

namespace rclcpp
{
namespace detail
{
/** @brief Internal function for unit testing.
 * @internal
*/
std::unordered_set<std::string>
list_parameter_override_prefixes(
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  std::string prefix);
}  // namespace detail

/**
 * @brief Get parameter overrides that have a given prefix
 *
 * Example:
 *  Say the given parameter overrides are foo, foo.baz, foo.bar.baz, and foobar.baz
 *  Given prefix "", this will return foo, and foobar
 *  Given prefix "foo", this will return foo.baz and foo.bar
 *  Given prefix "foo.bar", this will return foo.bar.baz
 *  Given prefix "foo.baz", this will return an empty list
 *
 *  All overrides are searched and returned, so that this can be used to
 *  conditionally declare parameters.
 *  The returned names may or may not be valid parameters, but instead are
 *  prefixes of valid parameters.
 *  The prefix itself will never be in the returned output.
 *
 *
 * @param[in] interfaces - The node interfaces used to get the parameter overrides
 * @param[in] prefix - the parameter prefix
 * @param[in] max_depth - how deep to return parameter override names, or 0 for
 *    unlimited depth.
*/
std::unordered_set<std::string>
list_parameter_override_prefixes(
  node_interfaces::NodeInterfaces<node_interfaces::NodeParametersInterface> interfaces,
  std::string prefix);
}  // namespace rclcpp

#endif  // RCLCPP__LIST_PARAMETER_OVERRIDE_PREFIXES_HPP_
