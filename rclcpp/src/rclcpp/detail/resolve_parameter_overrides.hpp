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

#ifndef RCLCPP__DETAIL__RESOLVE_PARAMETER_OVERRIDES_HPP_
#define RCLCPP__DETAIL__RESOLVE_PARAMETER_OVERRIDES_HPP_

#include <string>
#include <map>
#include <vector>

#include "rcl/arguments.h"

#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace detail
{
/// \internal Get the parameter overrides from the arguments.
RCLCPP_LOCAL
std::map<std::string, rclcpp::node_interfaces::ParameterInfo>
resolve_parameter_overrides(
  const std::string & node_name,
  const std::vector<rclcpp::Parameter> & parameter_overrides,
  const rcl_arguments_t * local_args,
  const rcl_arguments_t * global_args);

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RESOLVE_PARAMETER_OVERRIDES_HPP_
