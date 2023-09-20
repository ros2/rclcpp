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

/// Copy all parameters from one source node to another destination node.
/**
 * \param source Node to copy parameters from
 * \param destination Node to copy parameters to
 * \param override_existing_params Default false. Whether to override existing destination params
 * if both the source and destination contain the same parameter.
 */
template<typename NodeT1, typename NodeT2>
void
copy_all_parameters(
  const NodeT1 & source, const NodeT2 & destination, const bool override_existing_params = false)
{
  using Parameters = std::vector<rclcpp::Parameter>;
  auto source_params = source->get_node_parameters_interface();
  auto dest_params = destination->get_node_parameters_interface();

  std::vector<std::string> param_names = source_params->list_parameters({}, 0).names;
  Parameters params = source_params->get_parameters(param_names);
  for (Parameters::const_iterator iter = params.begin(); iter != params.end(); ++iter) {
    if (!dest_params->has_parameter(iter->get_name())) {
      dest_params->declare_parameter(iter->get_name(), iter->get_parameter_value());
    } else if (override_existing_params) {
      dest_params->set_parameters_atomically({*iter});
    }
  }
}

}  // namespace rclcpp

#endif  // RCLCPP__COPY_ALL_PARAMETERS_HPP_
