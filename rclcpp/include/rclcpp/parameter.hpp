// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <string>

#include <rmw/rmw.h>

#include <rclcpp/macros.hpp>

#include <rcl_interfaces/Parameter.h>
#include <rcl_interfaces/ParameterDescription.h>

namespace rclcpp
{

namespace parameter
{
// Datatype for parameter names
typedef std::string ParameterName;

// Datatype for storing parameter types
enum ParameterDataType {
  INVALID_PARAMETER, INTEGER_PARAMETER, DOUBLE_PARAMETER, STRING_PARAMETER, BOOL_PARAMETER
};

// Structure to store an arbitrary parameter with templated get/set methods
class ParamContainer
{
public:
  ParamContainer()
  : typeID_(INVALID_PARAM) {}
  ParamContainer(
    const std::string & name, const int64_t int_value)
  : name_(name), typeID_(INT_PARAM), int_value_(int_value) {}
  ParamContainer(
    const std::string & name, const double double_value)
  : name_(name), typeID_(DOUBLE_PARAM), double_value_(double_value) {}
  ParamContainer(
    const std::string & name, const std::string & string_value)
  : name_(name), typeID_(STRING_PARAM), string_value_(string_value) {}
  ParamContainer(
    const std::string & name, const bool bool_value)
  : name_(name), typeID_(BOOL_PARAM), bool_value_(bool_value) {}

  ParamName name_;
  ParamDataType typeID_;

  /* Templated getter */
  template<typename T>
  T
  get_value() const;

// Structure to store an arbitrary parameter with templated get/set methods
  class ParameterContainer
  {
public:
    ParameterContainer()
    : typeID_(INVALID_PARAMETER) {}
    ParameterContainer(
      const std::string & name, const int64_t int_value)
    : name_(name), typeID_(INTEGER_PARAMETER), int_value_(int_value) {}
    ParameterContainer(
      const std::string & name, const double double_value)
    : name_(name), typeID_(DOUBLE_PARAMETER), double_value_(double_value) {}
    ParameterContainer(
      const std::string & name, const std::string & string_value)
    : name_(name), typeID_(STRING_PARAMETER), string_value_(string_value) {}
    ParameterContainer(
      const std::string & name, const bool bool_value)
    : name_(name), typeID_(BOOL_PARAMETER), bool_value_(bool_value) {}

    ParameterName name_;
    ParameterDataType typeID_;

    /* Templated getter */
    template<typename T>
    T
    get_value() const;

    inline ParameterName get_name() const {return name_; }

    inline ParameterDataType get_typeID() const {return typeID_; }

private:
    int64_t int_value_;
    double double_value_;
    std::string string_value_;
    bool bool_value_;
  };

  template<>
  inline int64_t ParameterContainer::get_value() const
  {
    if (typeID_ != INTEGER_PARAMETER) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return int_value_;
  }
  template<>
  inline double ParameterContainer::get_value() const
  {
    if (typeID_ != DOUBLE_PARAMETER) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return double_value_;
  }
  template<>
  inline std::string ParameterContainer::get_value() const
  {
    if (typeID_ != STRING_PARAMETER) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return string_value_;
  }
  template<>
  inline bool ParameterContainer::get_value() const
  {
    if (typeID_ != BOOL_PARAMETER) {
      // TODO: use custom exception
      throw std::runtime_error("Invalid type");
    }
    return bool_value_;
  }

  class ParameterQuery
  {
public:
    ParameterQuery(const std::string & name)
    : typeID_(INVALID_PARAMETER), name_(name) {}
    ParameterQuery(const ParameterDataType typeID)
    : typeID_(typeID), name_("") {}

    // TODO: make this extendable for potential regex or other dynamic queryies
    // Possibly use a generator pattern?
    // For now just store a single datatype and provide accessors.

    inline ParameterDataType get_type() const
    {
      return typeID_;
    }
    inline ParameterName get_name() const
    {
      return name_;
    }

private:
    ParameterDataType typeID_;
    ParameterName name_;
  };

  template<typename T>
  T get_parameter_value(rcl_interfaces::Parameter & parameter);

  template<>
  bool get_parameter_value(rcl_interfaces::Parameter & parameter)
  {
    if (parameter.description.parameter_type !=
      rcl_interfaces::ParameterDescription::BOOL_PARAMETER)
    {
      // TODO: use custom exception
      throw std::runtime_error("Parameter value is not a boolean");
    }
    return parameter.bool_value;
  }

  template<>
  int64_t get_parameter_value(rcl_interfaces::Parameter & parameter)
  {
    if (parameter.description.parameter_type !=
      rcl_interfaces::ParameterDescription::INTEGER_PARAMETER)
    {
      // TODO: use custom exception
      throw std::runtime_error("Parameter value is not an integer");
    }
    return parameter.integer_value;
  }

  template<>
  double get_parameter_value(rcl_interfaces::Parameter & parameter)
  {
    if (parameter.description.parameter_type !=
      rcl_interfaces::ParameterDescription::DOUBLE_PARAMETER)
    {
      // TODO: use custom exception
      throw std::runtime_error("Parameter value is not a double");
    }
    return parameter.double_value;
  }

  template<>
  std::string get_parameter_value(rcl_interfaces::Parameter & parameter)
  {
    if (parameter.description.parameter_type !=
      rcl_interfaces::ParameterDescription::STRING_PARAMETER)
    {
      // TODO: use custom exception
      throw std::runtime_error("Parameter value is not a string");
    }
    return parameter.string_value;
  }

  template<typename T>
  void set_parameter_value(rcl_interfaces::Parameter & parameter, const T & value);

  template<>
  void set_parameter_value(rcl_interfaces::Parameter & parameter, const bool & value)
  {
    parameter.description.parameter_type = rcl_interfaces::ParameterDescription::BOOL_PARAMETER;
    parameter.bool_value = value;
  }

  template<>
  void set_parameter_value(rcl_interfaces::Parameter & parameter, const int64_t & value)
  {
    parameter.description.parameter_type = rcl_interfaces::ParameterDescription::INTEGER_PARAMETER;
    parameter.integer_value = value;
  }

  template<>
  void set_parameter_value(rcl_interfaces::Parameter & parameter, const double & value)
  {
    parameter.description.parameter_type = rcl_interfaces::ParameterDescription::DOUBLE_PARAMETER;
    parameter.double_value = value;
  }

  template<>
  void set_parameter_value(rcl_interfaces::Parameter & parameter, const std::string & value)
  {
    parameter.description.parameter_type = rcl_interfaces::ParameterDescription::STRING_PARAMETER;
    parameter.string_value = value;
  }
};
} /* namespace parameter */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PARAMETER_HPP_ */
