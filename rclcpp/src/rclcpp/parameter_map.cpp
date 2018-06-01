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

#include <string>
#include <vector>

#include "rclcpp/parameter_map.hpp"

using rclcpp::exceptions::InvalidParametersException;
using rclcpp::exceptions::InvalidParameterValueException;
using rclcpp::ParameterMap;
using rclcpp::ParameterValue;

ParameterMap
rclcpp::parameter_map_from(const rcl_params_t * const c_params)
{
  if (NULL == c_params) {
    throw InvalidParametersException("parameters struct is NULL");
  } else if (NULL == c_params->node_names) {
    throw InvalidParametersException("node names array is NULL");
  } else if (NULL == c_params->params) {
    throw InvalidParametersException("node params array is NULL");
  }

  // Convert c structs into a list of parameters to set
  ParameterMap parameters;
  for (size_t n = 0; n < c_params->num_nodes; ++n) {
    const char * c_node_name = c_params->node_names[n];
    if (NULL == c_node_name) {
      throw InvalidParametersException("Node name at index " + std::to_string(n) + " is NULL");
    }

    /// make sure there is a leading slash on the fully qualified node name
    std::string node_name("/");
    if ('/' != c_node_name[0]) {
      node_name += c_node_name;
    } else {
      node_name = c_node_name;
    }

    const rcl_node_params_t * const c_params_node = &(c_params->params[n]);

    std::vector<Parameter> & params_node = parameters[node_name];
    params_node.reserve(c_params_node->num_params);

    for (size_t p = 0; p < c_params_node->num_params; ++p) {
      const char * const c_param_name = c_params_node->parameter_names[p];
      if (NULL == c_param_name) {
        std::string message(
          "At node " + std::to_string(n) + " parameter " + std::to_string(p) + " name is NULL");
        throw InvalidParametersException(message);
      }
      const rcl_variant_t * const c_param_value = &(c_params_node->parameter_values[p]);
      params_node.emplace_back(c_param_name, parameter_value_from(c_param_value));
    }
  }
  return parameters;
}

ParameterValue
rclcpp::parameter_value_from(const rcl_variant_t * const c_param_value)
{
  if (NULL == c_param_value) {
    throw InvalidParameterValueException("Passed argument is NULL");
  }
  if (c_param_value->bool_value) {
    return ParameterValue(*(c_param_value->bool_value));
  } else if (c_param_value->integer_value) {
    return ParameterValue(*(c_param_value->integer_value));
  } else if (c_param_value->double_value) {
    return ParameterValue(*(c_param_value->double_value));
  } else if (c_param_value->string_value) {
    return ParameterValue(std::string(c_param_value->string_value));
  } else if (c_param_value->byte_array_value) {
    const rcl_byte_array_t * const byte_array = c_param_value->byte_array_value;
    std::vector<uint8_t> bytes;
    bytes.reserve(byte_array->size);
    for (size_t v = 0; v < byte_array->size; ++v) {
      bytes.push_back(byte_array->values[v]);
    }
    return ParameterValue(bytes);
  } else if (c_param_value->bool_array_value) {
    const rcl_bool_array_t * const bool_array = c_param_value->bool_array_value;
    std::vector<bool> bools;
    bools.reserve(bool_array->size);
    for (size_t v = 0; v < bool_array->size; ++v) {
      bools.push_back(bool_array->values[v]);
    }
    return ParameterValue(bools);
  } else if (c_param_value->integer_array_value) {
    const rcl_int64_array_t * const int_array = c_param_value->integer_array_value;
    std::vector<int64_t> integers;
    integers.reserve(int_array->size);
    for (size_t v = 0; v < int_array->size; ++v) {
      integers.push_back(int_array->values[v]);
    }
    return ParameterValue(integers);
  } else if (c_param_value->double_array_value) {
    const rcl_double_array_t * const double_array = c_param_value->double_array_value;
    std::vector<double> doubles;
    doubles.reserve(double_array->size);
    for (size_t v = 0; v < double_array->size; ++v) {
      doubles.push_back(double_array->values[v]);
    }
    return ParameterValue(doubles);
  } else if (c_param_value->string_array_value) {
    const rcutils_string_array_t * const string_array = c_param_value->string_array_value;
    std::vector<std::string> strings;
    strings.reserve(string_array->size);
    for (size_t v = 0; v < string_array->size; ++v) {
      strings.emplace_back(string_array->data[v]);
    }
    return ParameterValue(strings);
  }

  throw InvalidParameterValueException("No parameter value set");
}
