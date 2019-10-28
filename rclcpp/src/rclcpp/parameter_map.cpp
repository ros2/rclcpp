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

#include <limits>
#include <string>
#include <vector>

#include "rclcpp/parameter_map.hpp"

using rclcpp::exceptions::InvalidParametersException;
using rclcpp::exceptions::InvalidParameterValueException;
using rclcpp::ParameterMap;
using rclcpp::ParameterValue;
using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::FloatingPointRange;
using rcl_interfaces::msg::IntegerRange;

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

    ParameterAndDescriptor & params = parameters[node_name];

    const rcl_node_params_t * const c_params_node = &(c_params->params[n]);
    for (size_t p = 0; p < c_params_node->num_params; ++p) {
      const char * const c_param_name = c_params_node->parameter_names[p];
      if (NULL == c_param_name) {
        std::string message(
          "At node " + std::to_string(n) + " parameter " + std::to_string(p) + " name is NULL");
        throw InvalidParametersException(message);
      }
      const rcl_variant_t * const c_param_value = &(c_params_node->parameter_values[p]);
      params[c_param_name].first = Parameter(c_param_name, parameter_value_from(c_param_value));
    }

    const rcl_node_params_descriptors_t * const c_param_descriptors_node =
      &(c_params->descriptors[n]);
    for (size_t p = 0; p < c_param_descriptors_node->num_params; ++p) {
      const char * const c_param_name = c_param_descriptors_node->parameter_names[p];
      if (NULL == c_param_name) {
        std::string message(
          "At node " + std::to_string(n) + " parameter " + std::to_string(p) + " name is NULL");
        throw InvalidParametersException(message);
      }
      const rcl_param_descriptor_t * const c_param_descriptor =
        &(c_param_descriptors_node->parameter_descriptors[p]);
      if (params.count(std::string(c_param_name)) < 1) {
        params[c_param_name].first = Parameter(c_param_name);
      }
      params[c_param_name].second = parameter_descriptor_from(c_param_descriptor);
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

ParameterDescriptor
rclcpp::parameter_descriptor_from(const rcl_param_descriptor_t * const c_param_descriptor)
{
  if (NULL == c_param_descriptor) {
    throw InvalidParameterValueException("Passed argument is NULL");
  }
  ParameterDescriptor p;

  if (c_param_descriptor->name) {
    p.name = std::string(c_param_descriptor->name);
  }
  if (c_param_descriptor->type) {
    p.type = *(c_param_descriptor->type);
  }
  if (c_param_descriptor->description) {
    p.description = std::string(c_param_descriptor->description);
  }
  if (c_param_descriptor->additional_constraints) {
    p.additional_constraints = std::string(c_param_descriptor->additional_constraints);
  }
  if (c_param_descriptor->read_only) {
    p.read_only = *(c_param_descriptor->read_only);
  }

  if (c_param_descriptor->min_value_int || c_param_descriptor->max_value_int ||
    c_param_descriptor->step_int)
  {
    if (c_param_descriptor->min_value_double || c_param_descriptor->max_value_double ||
      c_param_descriptor->step_double)
    {
      throw InvalidParameterValueException(
              "Mixed 'integer' and 'double' types not supported for parameter descriptor range");
    }
    IntegerRange i;
    if (c_param_descriptor->min_value_int) {
      i.from_value = *(c_param_descriptor->min_value_int);
    } else {
      i.from_value = std::numeric_limits<int64_t>::min();
    }
    if (c_param_descriptor->max_value_int) {
      i.to_value = *(c_param_descriptor->max_value_int);
    } else {
      i.to_value = std::numeric_limits<int64_t>::max();
    }
    if (c_param_descriptor->step_int) {
      i.step = *(c_param_descriptor->step_int);
    }
    p.integer_range.push_back(i);
  } else if (c_param_descriptor->min_value_double || c_param_descriptor->max_value_double ||  // NOLINT
    c_param_descriptor->step_double)
  {
    FloatingPointRange f;
    if (c_param_descriptor->min_value_double) {
      f.from_value = *(c_param_descriptor->min_value_double);
    } else {
      f.from_value = std::numeric_limits<double>::lowest();
    }
    if (c_param_descriptor->max_value_double) {
      f.to_value = *(c_param_descriptor->max_value_double);
    } else {
      f.to_value = std::numeric_limits<double>::max();
    }
    if (c_param_descriptor->step_double) {
      f.step = *(c_param_descriptor->step_double);
    }
    p.floating_point_range.push_back(f);
  }

  return p;
}

ParameterMap
rclcpp::parameter_map_from_yaml_file(const std::string & yaml_filename)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t * rcl_parameters = rcl_yaml_node_struct_init(allocator);
  const char * path = yaml_filename.c_str();
  if (!rcl_parse_yaml_file(path, rcl_parameters)) {
    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR);
  }
  return rclcpp::parameter_map_from(rcl_parameters);
}
