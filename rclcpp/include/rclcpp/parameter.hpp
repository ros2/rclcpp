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


namespace rclcpp
{

namespace parameter
{
  // Datatype for parameter names
  typedef std::string ParamName;

  // Datatype for storing parameter types
  enum ParamDataType {INVALID_PARAM, INT_PARAM, DOUBLE_PARAM, STRING_PARAM, BOOL_PARAM};


  // Structure to store an arbitrary parameter with templated get/set methods
  class ParamContainer
  {
    public:
      ParamDataType typeID_;
      ParamName name_;

      /* Templated getter */
      template <typename T>
      T&
      get_value (T& value) const;

      inline ParamName get_name() const { return name_; };
      /* Templated setter */
      template <typename T>
      void
      set_value(const ParamName& name, const T& value);

    private:
      int64_t int_value_;
      double double_value_;
      std::string string_value_;
      bool bool_value_;
  };

  template <>
  inline int64_t& ParamContainer::get_value(int64_t& value) const
  {
    if (typeID_!= INT_PARAM)
      {
        // TODO: use custom exception
        throw std::runtime_error("Invalid type");
      }
      value = int_value_;
      return value;
  }
  template <>
  inline double& ParamContainer::get_value(double& value) const
  {
    if (typeID_!= DOUBLE_PARAM)
      {
        // TODO: use custom exception
        throw std::runtime_error("Invalid type");
      }
      value = double_value_;
      return value;
  }
  template <>
  inline std::string& ParamContainer::get_value(std::string& value) const
  {
    if (typeID_!= STRING_PARAM)
      {
        // TODO: use custom exception
        throw std::runtime_error("Invalid type");
      }
      value = string_value_;
      return value;
  }
  template <>
  inline bool& ParamContainer::get_value(bool& value) const
  {
    if (typeID_!= BOOL_PARAM)
      {
        // TODO: use custom exception
        throw std::runtime_error("Invalid type");
      }
      value = bool_value_;
      return value;
  }

  template <>
  inline void ParamContainer::set_value(const ParamName& name, const int64_t& value)
  {
    typeID_ = INT_PARAM;
    int_value_ = value;
  }

  template <>
  inline void ParamContainer::set_value(const ParamName& name, const double& value)
  {
    typeID_ = DOUBLE_PARAM;
    double_value_ = value;
  }

  template <>
  inline void ParamContainer::set_value(const ParamName& name, const std::string& value)
  {
    typeID_ = STRING_PARAM;
    string_value_ = value;
  }

  template <>
  inline void ParamContainer::set_value(const ParamName& name, const bool& value)
  {
    typeID_ = BOOL_PARAM;
    bool_value_ = value;
  }

  class ParamQuery
  {
    public:
      ParamQuery(const std::string& name) : typeID_(INVALID_PARAM), name_(name) {}
      ParamQuery(const ParamDataType typeID) : typeID_(typeID), name_("") {}

      // TODO: make this extendable for potential regex or other dynamic queryies
      // Possibly use a generator pattern?
      // For now just store a single datatype and provide accessors.

      inline ParamDataType get_type(void) const
      {
        return typeID_;
      }
      inline ParamName get_name(void) const
      {
        return name_;
      }

    private:
      ParamDataType typeID_;
      ParamName name_;
  };
} /* namespace parameter */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PARAMETER_HPP_ */
