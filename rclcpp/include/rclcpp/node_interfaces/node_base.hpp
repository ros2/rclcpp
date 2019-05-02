// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rcl/node.h"
#include "rclcpp/context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeBase part of the Node API.
class NodeBase : public NodeBaseInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeBase)

  RCLCPP_PUBLIC
  NodeBase(
    const std::string & node_name,
    const std::string & namespace_,
    rclcpp::Context::SharedPtr context,
    const rcl_node_options_t & rcl_node_options,
    bool use_intra_process_default);

  RCLCPP_PUBLIC
  virtual
  ~NodeBase();

  RCLCPP_PUBLIC
  virtual
  const char *
  get_name() const;

  RCLCPP_PUBLIC
  virtual
  const char *
  get_namespace() const;

  RCLCPP_PUBLIC
  virtual
  const char *
  get_fully_qualified_name() const;

  RCLCPP_PUBLIC
  virtual
  rclcpp::Context::SharedPtr
  get_context();

  RCLCPP_PUBLIC
  virtual
  rcl_node_t *
  get_rcl_node_handle();

  RCLCPP_PUBLIC
  virtual
  const rcl_node_t *
  get_rcl_node_handle() const;

  RCLCPP_PUBLIC
  virtual
  std::shared_ptr<rcl_node_t>
  get_shared_rcl_node_handle();

  RCLCPP_PUBLIC
  virtual
  std::shared_ptr<const rcl_node_t>
  get_shared_rcl_node_handle() const;

  RCLCPP_PUBLIC
  virtual
  bool
  assert_liveliness() const;

  RCLCPP_PUBLIC
  virtual
  rclcpp::callback_group::CallbackGroup::SharedPtr
  create_callback_group(rclcpp::callback_group::CallbackGroupType group_type);

  RCLCPP_PUBLIC
  virtual
  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_default_callback_group();

  RCLCPP_PUBLIC
  virtual
  bool
  callback_group_in_node(rclcpp::callback_group::CallbackGroup::SharedPtr group);

  RCLCPP_PUBLIC
  virtual
  const std::vector<rclcpp::callback_group::CallbackGroup::WeakPtr> &
  get_callback_groups() const;

  RCLCPP_PUBLIC
  virtual
  std::atomic_bool &
  get_associated_with_executor_atomic();

  RCLCPP_PUBLIC
  virtual
  rcl_guard_condition_t *
  get_notify_guard_condition();

  RCLCPP_PUBLIC
  virtual
  std::unique_lock<std::recursive_mutex>
  acquire_notify_guard_condition_lock() const;

  RCLCPP_PUBLIC
  virtual
  bool
  get_use_intra_process_default() const;

private:
  RCLCPP_DISABLE_COPY(NodeBase)

  rclcpp::Context::SharedPtr context_;
  bool use_intra_process_default_;

  std::shared_ptr<rcl_node_t> node_handle_;

  rclcpp::callback_group::CallbackGroup::SharedPtr default_callback_group_;
  std::vector<rclcpp::callback_group::CallbackGroup::WeakPtr> callback_groups_;

  std::atomic_bool associated_with_executor_;

  /// Guard condition for notifying the Executor of changes to this node.
  mutable std::recursive_mutex notify_guard_condition_mutex_;
  rcl_guard_condition_t notify_guard_condition_ = rcl_get_zero_initialized_guard_condition();
  bool notify_guard_condition_is_valid_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_
