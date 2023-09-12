// Copyright 2023 Open Navigation LLC
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

#ifndef RCLCPP__COPY_ALL_PARAMETERS_HPP_
#define RCLCPP__COPY_ALL_PARAMETERS_HPP_

#include <vector>
#include <string>
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/parameter.hpp"

namespace rclcpp
{

/**
 * @brief A method to copy all parameters from one node (parent) to another (child).
 * May throw parameter exceptions in error conditions
 * @param parent Node to copy parameters from
 * @param child Node to copy parameters to
 */
template<typename NodeT1, typename NodeT2>
void copy_all_parameters(const NodeT1 & parent, const NodeT2 & child)
{
  using Parameters = std::vector<rclcpp::Parameter>;
  std::vector<std::string> param_names = parent->list_parameters({}, 0).names;
  Parameters params = parent->get_parameters(param_names);
  for (Parameters::const_iterator iter = params.begin(); iter != params.end(); ++iter) {
    if (!child->has_parameter(iter->get_name())) {
      child->declare_parameter(iter->get_name(), iter->get_parameter_value());
    }
  }
}

}  // namespace rclcpp

#endif  // RCLCPP__COPY_ALL_PARAMETERS_HPP_
