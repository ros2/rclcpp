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
  typedef std::string param_name_t;

  // Datatype for storing parameter types
  enum ParamDataType {INT_PARAM, DOUBLE_PARAM, STRING_PARAM, BOOL_PARAM};


  // Structure to store an arbitrary parameter with templated get/set methods
  class ParamContainer
  {
    public:
      ParamDataType typeID_;
      param_name_t name_;

      /* Templated getter */
      template <class T>
      T&
      get_value(T& val);

      inline param_name_t getName() { return name_; };
      /* Templated setter */
      template <class T>
      bool
      set_value(const param_name_t& name, T& value);

    private:
      int64_t i_;
      double d_;
      std::string s_;
      bool b_;
  };

  template <>
  inline int64_t& ParamContainer::get_value(int64_t& val)
  {
    if (typeID_!= INT_PARAM)
      {
        // TODO: use custom exception
        throw std::runtime_error("Invalid type");
      }
      val = i_;
      return val;
  }
  template <>
  inline double& ParamContainer::get_value(double& val)
  {
    if (typeID_!= DOUBLE_PARAM)
      {
        // TODO: use custom exception
        throw std::runtime_error("Invalid type");
      }
      val = d_;
      return val;
  }
  template <>
  inline std::string& ParamContainer::get_value(std::string& val)
  {
    if (typeID_!= STRING_PARAM)
      {
        // TODO: use custom exception
        throw std::runtime_error("Invalid type");
      }
      val = s_;
      return val;
  }
  template <>
  inline bool& ParamContainer::get_value(bool& val)
  {
    if (typeID_!= BOOL_PARAM)
      {
        // TODO: use custom exception
        throw std::runtime_error("Invalid type");
      }
      val = b_;
      return val;
  }
  struct ParamQuery
  {
    // TODO: make this extendable for potential regex or other dynamic queryies
    // Possibly use a generator pattern?
    // For now just store a single datatype and provide accessors.

    inline ParamDataType getType(void){ return typeID_;};
    inline param_name_t getName(void){ return name_;};

    ParamDataType typeID_;
    param_name_t name_;
  };
} /* namespace parameter */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PARAMETER_HPP_ */
