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

#ifndef RCLCPP__NODE_INTERFACES__NODE_WAITABLES_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_WAITABLES_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeWaitables part of the Node API.
class NodeWaitables : public NodeWaitablesInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeWaitables)

  RCLCPP_PUBLIC
  explicit NodeWaitables(rclcpp::node_interfaces::NodeBaseInterface * node_base);

  RCLCPP_PUBLIC
  virtual
  ~NodeWaitables();

  RCLCPP_PUBLIC
  void
  add_waitable(
    rclcpp::Waitable::SharedPtr waitable_base_ptr,
    rclcpp::CallbackGroup::SharedPtr group) override;

  RCLCPP_PUBLIC
  void
  remove_waitable(
    rclcpp::Waitable::SharedPtr waitable_ptr,
    rclcpp::CallbackGroup::SharedPtr group) noexcept override;

private:
  RCLCPP_DISABLE_COPY(NodeWaitables)

  rclcpp::node_interfaces::NodeBaseInterface * node_base_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_WAITABLES_HPP_
