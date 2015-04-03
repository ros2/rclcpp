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
  struct ParamContainer
  {
    enum {INT, DOUBLE, STRING, BOOL} typeID;
    union {
      int64_t i;
      double d;
      std::string s;
      bool b;
    }
  };
} /* namespace parameter */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PARAMETER_HPP_ */
