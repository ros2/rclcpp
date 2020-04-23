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

#ifndef RCLCPP__NODE_INTERFACES__NODE_WAITABLES_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_WAITABLES_INTERFACE_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Pure virtual interface class for the NodeWaitables part of the Node API.
class NodeWaitablesInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeWaitablesInterface)

  RCLCPP_PUBLIC
  virtual
  ~NodeWaitablesInterface() = default;

  RCLCPP_PUBLIC
  virtual
  void
  add_waitable(
    rclcpp::Waitable::SharedPtr waitable_ptr,
    rclcpp::CallbackGroup::SharedPtr group) = 0;

  /// \note this function should not throw because it may be called in destructors
  RCLCPP_PUBLIC
  virtual
  void
  remove_waitable(
    rclcpp::Waitable::SharedPtr waitable_ptr,
    rclcpp::CallbackGroup::SharedPtr group) noexcept = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_WAITABLES_INTERFACE_HPP_
