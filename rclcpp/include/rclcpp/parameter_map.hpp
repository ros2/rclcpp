// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__PARAMETER_MAP_HPP_
#define RCLCPP__PARAMETER_MAP_HPP_

#include <rcl_yaml_param_parser/parser.h>
#include <rcl_yaml_param_parser/types.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace rclcpp
{

/// A map of fully qualified node names to a list of parameters
using rcl_interfaces::msg::ParameterDescriptor;
using ParameterAndDescriptor = std::unordered_map<std::string, rclcpp::node_interfaces::ParameterInfo>;
using ParameterMap = std::unordered_map<std::string, ParameterAndDescriptor>;

/// Convert parameters from rcl_yaml_param_parser into C++ class instances.
/// \param[in] c_params C structures containing parameters for multiple nodes.
/// \returns a map where the keys are fully qualified node names and values a list of parameters.
/// \throws InvalidParametersException if the `rcl_params_t` is inconsistent or invalid.
RCLCPP_PUBLIC
ParameterMap
parameter_map_from(const rcl_params_t * const c_params);

/// Convert parameter value from rcl_yaml_param_parser into a C++ class instance.
/// \param[in] c_value C structure containing a value of a parameter.
/// \returns an instance of a parameter value
/// \throws InvalidParameterValueException if the `rcl_variant_t` is inconsistent or invalid.
RCLCPP_PUBLIC
ParameterValue
parameter_value_from(const rcl_variant_t * const c_value);

RCLCPP_PUBLIC
rcl_interfaces::msg::ParameterDescriptor
parameter_descriptor_from(const rcl_param_descriptor_t * const c_descriptor);

/// Get the ParameterMap from a yaml file.
/// \param[in] yaml_filename full name of the yaml file.
/// \returns an instance of a parameter map
/// \throws from rcl error of rcl_parse_yaml_file()
RCLCPP_PUBLIC
ParameterMap
parameter_map_from_yaml_file(const std::string & yaml_filename);


}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_MAP_HPP_
