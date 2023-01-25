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

#include <rclcpp/list_parameter_override_prefixes.hpp>

#include <cassert>
#include <string>
#include <unordered_set>


namespace rclcpp
{
std::unordered_set<std::string>
list_parameter_override_prefixes(
  node_interfaces::NodeInterfaces<node_interfaces::NodeParametersInterface> interfaces,
  std::string prefix)
{
  const std::map<std::string, rclcpp::ParameterValue> & overrides =
    interfaces.get_node_parameters_interface()->get_parameter_overrides();
  return detail::list_parameter_override_prefixes(overrides, prefix);
}

std::unordered_set<std::string>
detail::list_parameter_override_prefixes(
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  std::string prefix)
{
  // TODO(sloretz) ROS 2 must have this in a header somewhere, right?
  const char kParamSeparator = '.';

  // Find all overrides starting with "prefix.", unless the prefix is empty.
  // If the prefix is empty then look at all parameter overrides.
  if (!prefix.empty() && prefix.back() != kParamSeparator) {
    prefix += kParamSeparator;
  }

  std::unordered_set<std::string> output_names;
  for (const auto & kv : overrides) {
    const std::string & name = kv.first;
    if (name.size() <= prefix.size()) {
      // Too short, no point in checking
      continue;
    }
    assert(prefix.size() < name.size());
    // TODO(sloretz) use string::starts_with in c++20
    if (name.rfind(prefix, 0) == 0) {  // if name starts with prefix
      // Truncate names to the next separator
      size_t separator_pos = name.find(kParamSeparator, prefix.size());
      // Insert truncated name
      output_names.insert(name.substr(0, separator_pos));
    }
  }
  return output_names;
}
}  // namespace rclcpp
