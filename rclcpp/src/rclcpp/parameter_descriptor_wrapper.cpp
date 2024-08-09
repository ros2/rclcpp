// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/parameter_descriptor_wrapper.hpp"

namespace rclcpp
{

ParameterDescription::ParameterDescription()
{
  // Need to set this in the constructor, but it doesn't necessarily need to be used
  parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
}

rcl_interfaces::msg::ParameterDescriptor ParameterDescription::build() const
{
  // Return some some sort message
  return parameter_descriptor;
}

// Builder methods which set up the original class
// They all follow the same format of initing the value given within the base class
// then returning the current class
ParameterDescription & ParameterDescription::set_name(const std::string & name)
{
  parameter_descriptor.name = name;
  return *this;
}

ParameterDescription & ParameterDescription::set_type(std::uint8_t type)
{
  parameter_descriptor.type = type;
  return *this;
}

ParameterDescription & ParameterDescription::set_description_text(const std::string & description)
{
  parameter_descriptor.description = description;
  return *this;
}

ParameterDescription & ParameterDescription::set_additional_constraints(
  const std::string & constraints)
{
  parameter_descriptor.additional_constraints = constraints;
  return *this;
}

ParameterDescription & ParameterDescription::set_read_only(bool read_only)
{
  parameter_descriptor.read_only = read_only;
  return *this;
}

ParameterDescription & ParameterDescription::set_dynamic_typing(bool dynamic_typing)
{
  parameter_descriptor.dynamic_typing = dynamic_typing;
  return *this;
}

// Here is the Specific range function for this parameter description
ParameterDescription & ParameterDescription::set_floating_point_description_range(
  float min, float max, float step)
{
  if (parameter_descriptor.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
    parameter_descriptor.floating_point_range.resize(1);
    parameter_descriptor.floating_point_range.at(0).from_value = min;
    parameter_descriptor.floating_point_range.at(0).to_value = max;
    parameter_descriptor.floating_point_range.at(0).step = step;
  }
  return *this;
}

ParameterDescription & ParameterDescription::set_integer_description_range(
  int min, int max, int step)
{
  if (parameter_descriptor.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
    parameter_descriptor.integer_range.resize(1);
    parameter_descriptor.integer_range.at(0).from_value = min;
    parameter_descriptor.integer_range.at(0).to_value = max;
    parameter_descriptor.integer_range.at(0).step = step;
  }
  return *this;
}

}  // namespace rclcpp
