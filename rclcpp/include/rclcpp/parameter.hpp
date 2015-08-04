// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_RCLCPP_PARAMETER_HPP_
#define RCLCPP_RCLCPP_PARAMETER_HPP_

#include <ostream>
#include <sstream>
#include <string>

#include <rmw/rmw.h>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

namespace rclcpp
{

namespace parameter
{

enum ParameterType
{
  PARAMETER_NOT_SET=rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET,
  PARAMETER_BOOL=rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
  PARAMETER_INTEGER=rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
  PARAMETER_DOUBLE=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
  PARAMETER_STRING=rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
  PARAMETER_BYTES=rcl_interfaces::msg::ParameterType::PARAMETER_BYTES,
};

// Structure to store an arbitrary parameter with templated get/set methods
class ParameterVariant
{
public:
  ParameterVariant()
  : name_("")
  {
    value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
  }
  explicit ParameterVariant(const std::string & name, const bool bool_value)
  : name_(name)
  {
    value_.bool_value = bool_value;
    value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  }
  explicit ParameterVariant(const std::string & name, const int int_value)
  : name_(name)
  {
    value_.integer_value = int_value;
    value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  }
  explicit ParameterVariant(const std::string & name, const int64_t int_value)
  : name_(name)
  {
    value_.integer_value = int_value;
    value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  }
  explicit ParameterVariant(const std::string & name, const float double_value)
  : name_(name)
  {
    value_.double_value = double_value;
    value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  }
  explicit ParameterVariant(const std::string & name, const double double_value)
  : name_(name)
  {
    value_.double_value = double_value;
    value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  }
  explicit ParameterVariant(const std::string & name, const std::string & string_value)
  : name_(name)
  {
    value_.string_value = string_value;
    value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  }
  explicit ParameterVariant(const std::string & name, const char * string_value)
  : ParameterVariant(name, std::string(string_value)) {}
  explicit ParameterVariant(const std::string & name, const std::vector<uint8_t> & bytes_value)
  : name_(name)
  {
    value_.bytes_value = bytes_value;
    value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BYTES;
  }

  inline ParameterType get_type() const {return static_cast<ParameterType>(value_.type); }

  inline std::string get_type_name() const
  {
    switch (get_type()) {
      case rclcpp::parameter::ParameterType::PARAMETER_BOOL:
        return "bool";
      case rclcpp::parameter::ParameterType::PARAMETER_INTEGER:
        return "integer";
      case rclcpp::parameter::ParameterType::PARAMETER_DOUBLE:
        return "double";
      case rclcpp::parameter::ParameterType::PARAMETER_STRING:
        return "string";
      case rclcpp::parameter::ParameterType::PARAMETER_BYTES:
        return "bytes";
      case rclcpp::parameter::ParameterType::PARAMETER_NOT_SET:
        return "not set";
      default:
        // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
        throw std::runtime_error(
          "Unexpected type from ParameterVariant: " + std::to_string(get_type()));
        // *INDENT-ON*
    }
  }

  inline std::string get_name() const & {return name_; }

  inline rcl_interfaces::msg::ParameterValue get_parameter_value() const
  {
    return value_;
  }

  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_INTEGER, int64_t>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.integer_value;
  }
  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_DOUBLE, double>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.double_value;
  }
  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_STRING, const std::string &>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.string_value;
  }
  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_BOOL, bool>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.bool_value;
  }
  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_BYTES,
  const std::vector<uint8_t> &>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BYTES) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.bytes_value;
  }

  int64_t as_int() const {return get_value<ParameterType::PARAMETER_INTEGER>(); }

  double as_double() const {return get_value<ParameterType::PARAMETER_DOUBLE>(); }

  const std::string & as_string() const {return get_value<ParameterType::PARAMETER_STRING>(); }

  bool as_bool() const {return get_value<ParameterType::PARAMETER_BOOL>(); }

  const std::vector<uint8_t> & as_bytes() const
  {
    return get_value<ParameterType::PARAMETER_BYTES>();
  }

  static ParameterVariant from_parameter(const rcl_interfaces::msg::Parameter & parameter)
  {
    switch (parameter.value.type) {
      case PARAMETER_BOOL:
        return ParameterVariant(parameter.name, parameter.value.bool_value);
      case PARAMETER_INTEGER:
        return ParameterVariant(parameter.name, parameter.value.integer_value);
      case PARAMETER_DOUBLE:
        return ParameterVariant(parameter.name, parameter.value.double_value);
      case PARAMETER_STRING:
        return ParameterVariant(parameter.name, parameter.value.string_value);
      case PARAMETER_BYTES:
        return ParameterVariant(parameter.name, parameter.value.bytes_value);
      case PARAMETER_NOT_SET:
        throw std::runtime_error("Type from ParameterValue is not set");
      default:
        // TODO(wjwwood): use custom exception
        // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
        throw std::runtime_error(
          "Unexpected type from ParameterVariant: " + std::to_string(parameter.value.type));
        // *INDENT-ON*
    }
  }

  rcl_interfaces::msg::Parameter to_parameter()
  {
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = name_;
    parameter.value = value_;
    return parameter;
  }

  std::string to_string() const
  {
    switch (get_type()) {
      case rclcpp::parameter::ParameterType::PARAMETER_BOOL:
        return as_bool() ? "true" : "false";
      case rclcpp::parameter::ParameterType::PARAMETER_INTEGER:
        return std::to_string(as_int());
      case rclcpp::parameter::ParameterType::PARAMETER_DOUBLE:
        return std::to_string(as_double());
      case rclcpp::parameter::ParameterType::PARAMETER_STRING:
        return as_string();
      case rclcpp::parameter::ParameterType::PARAMETER_BYTES:
        {
          std::stringstream bytes;
          bool first_byte = true;
          bytes << "[" << std::hex;
          for (auto & byte : as_bytes()) {
            bytes << "0x" << byte;
            if (!first_byte) {
              bytes << ", ";
            } else {
              first_byte = false;
            }
          }
          return bytes.str();
        }
      case rclcpp::parameter::ParameterType::PARAMETER_NOT_SET:
        return "not set";
      default:
        // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
        throw std::runtime_error(
          "Unexpected type from ParameterVariant: " + std::to_string(get_type()));
        // *INDENT-ON*
    }
  }

private:
  std::string name_;
  rcl_interfaces::msg::ParameterValue value_;
};

std::ostream & operator<<(std::ostream & os, const ParameterVariant & pv)
{
  os << pv.to_string();
  return os;
}

} /* namespace parameter */

} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PARAMETER_HPP_ */
