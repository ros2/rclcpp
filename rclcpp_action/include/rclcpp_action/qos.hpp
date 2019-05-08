// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_ACTION__QOS_HPP_
#define RCLCPP_ACTION__QOS_HPP_

#include <rclcpp/qos.hpp>

#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{

class DefaultActionStatusQoS : public rclcpp::QoS
{
public:
  RCLCPP_ACTION_PUBLIC
  DefaultActionStatusQoS();
};

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__QOS_HPP_
