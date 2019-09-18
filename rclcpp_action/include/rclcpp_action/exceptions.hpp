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

#ifndef RCLCPP_ACTION__EXCEPTIONS_HPP_
#define RCLCPP_ACTION__EXCEPTIONS_HPP_

#include <stdexcept>

namespace rclcpp_action
{
namespace exceptions
{
class UnknownGoalHandleError : public std::invalid_argument
{
public:
  UnknownGoalHandleError()
  : std::invalid_argument("Goal handle is not known to this client.")
  {
  }
};

class UnawareGoalHandleError : public std::runtime_error
{
public:
  UnawareGoalHandleError()
  : std::runtime_error("Goal handle is not tracking the goal result.")
  {
  }
};

}  // namespace exceptions

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__EXCEPTIONS_HPP_
