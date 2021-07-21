// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include "./resolve_parameter_overrides.hpp"

#include <string>
#include <map>
#include <vector>

#include "rcl_yaml_param_parser/parser.h"

#include "rclcpp/scope_exit.hpp"
#include "rclcpp/parameter_map.hpp"

std::map<std::string, rclcpp::node_interfaces::ParameterInfo>
rclcpp::detail::resolve_parameter_overrides(
  const std::string & node_fqn,
  const std::vector<rclcpp::Parameter> & parameter_overrides,
  const rcl_arguments_t * local_args,
  const rcl_arguments_t * global_args)
{
  std::map<std::string, rclcpp::node_interfaces::ParameterInfo> result;

  // global before local so that local overwrites global
  std::array<const rcl_arguments_t *, 2> argument_sources = {global_args, local_args};

  // Get fully qualified node name post-remapping to use to find node's params in yaml files

  for (const rcl_arguments_t * source : argument_sources) {
    if (!source) {
      continue;
    }
    rcl_params_t * params = NULL;
    rcl_ret_t ret = rcl_arguments_get_param_overrides(source, &params);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
    if (params) {
      auto cleanup_params = make_scope_exit(
        [params]() {
          rcl_yaml_node_struct_fini(params);
        });
      rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(params);

      // Enforce wildcard matching precedence
      // TODO(cottsay) implement further wildcard matching
      const std::array<std::string, 2> node_matching_names{"/**", node_fqn};
      for (const auto & node_name : node_matching_names) {
        if (initial_map.count(node_name) > 0) {
          // Combine parameter yaml files, overwriting values in older ones
          for (const auto & param : initial_map.at(node_name)) {
            result[param.first] = param.second;
            result[param.first].descriptor.dynamic_typing = true;
          }
        }
      }
    }
  }

  // parameter overrides passed to constructor will overwrite overrides from yaml file sources
  for (auto & param : parameter_overrides) {
    result[param.get_name()].value =
      rclcpp::ParameterValue(param.get_value_message());
  }
  return result;
}
