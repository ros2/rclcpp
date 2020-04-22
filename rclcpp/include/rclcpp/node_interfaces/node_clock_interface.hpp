// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_CLOCK_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_CLOCK_INTERFACE_HPP_

#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Pure virtual interface class for the NodeClock part of the Node API.
class NodeClockInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeClockInterface)

  RCLCPP_PUBLIC
  virtual
  ~NodeClockInterface() = default;

  /// Get a ROS clock which will be kept up to date by the node.
  RCLCPP_PUBLIC
  virtual
  rclcpp::Clock::SharedPtr
  get_clock() = 0;

  /// Get a const ROS clock which will be kept up to date by the node.
  RCLCPP_PUBLIC
  virtual
  rclcpp::Clock::ConstSharedPtr
  get_clock() const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_CLOCK_INTERFACE_HPP_
