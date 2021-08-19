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

#ifndef RCLCPP__NODE_INTERFACES__NODE_BASE_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_BASE_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl/node.h"

#include "rclcpp/callback_group.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Pure virtual interface class for the NodeBase part of the Node API.
class NodeBaseInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeBaseInterface)

  using CallbackGroupFunction = std::function<void (rclcpp::CallbackGroup::SharedPtr)>;

  RCLCPP_PUBLIC
  virtual
  ~NodeBaseInterface() = default;

  /// Return the name of the node.
  /** \return The name of the node. */
  RCLCPP_PUBLIC
  virtual
  const char *
  get_name() const = 0;

  /// Return the namespace of the node.
  /** \return The namespace of the node. */
  RCLCPP_PUBLIC
  virtual
  const char *
  get_namespace() const = 0;

  /// Return the fully qualified name of the node.
  /** \return The fully qualified name of the node. */
  RCLCPP_PUBLIC
  virtual
  const char *
  get_fully_qualified_name() const = 0;

  /// Return the context of the node.
  /** \return SharedPtr to the node's context. */
  RCLCPP_PUBLIC
  virtual
  rclcpp::Context::SharedPtr
  get_context() = 0;

  /// Return the rcl_node_t node handle (non-const version).
  RCLCPP_PUBLIC
  virtual
  rcl_node_t *
  get_rcl_node_handle() = 0;

  /// Return the rcl_node_t node handle (const version).
  RCLCPP_PUBLIC
  virtual
  const rcl_node_t *
  get_rcl_node_handle() const = 0;

  /// Return the rcl_node_t node handle in a std::shared_ptr.
  /**
   * This handle remains valid after the Node is destroyed.
   * The actual rcl node is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  virtual
  std::shared_ptr<rcl_node_t>
  get_shared_rcl_node_handle() = 0;

  /// Return the rcl_node_t node handle in a std::shared_ptr.
  /**
   * This handle remains valid after the Node is destroyed.
   * The actual rcl node is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  virtual
  std::shared_ptr<const rcl_node_t>
  get_shared_rcl_node_handle() const = 0;

  /// Create and return a callback group.
  RCLCPP_PUBLIC
  virtual
  rclcpp::CallbackGroup::SharedPtr
  create_callback_group(
    rclcpp::CallbackGroupType group_type,
    bool automatically_add_to_executor_with_node = true) = 0;

  /// Return the default callback group.
  RCLCPP_PUBLIC
  virtual
  rclcpp::CallbackGroup::SharedPtr
  get_default_callback_group() = 0;

  /// Return true if the given callback group is associated with this node.
  RCLCPP_PUBLIC
  virtual
  bool
  callback_group_in_node(rclcpp::CallbackGroup::SharedPtr group) = 0;

  /// Return list of callback groups associated with this node.
  RCLCPP_PUBLIC
  virtual
  const std::vector<rclcpp::CallbackGroup::WeakPtr> &
  get_callback_groups() const = 0;

  /// Return the atomic bool which is used to ensure only one executor is used.
  RCLCPP_PUBLIC
  virtual
  std::atomic_bool &
  get_associated_with_executor_atomic() = 0;

  /// Return guard condition that should be notified when the internal node state changes.
  /**
   * For example, this should be notified when a publisher is added or removed.
   *
   * \return the rcl_guard_condition_t if it is valid, else nullptr
   */
  RCLCPP_PUBLIC
  virtual
  rcl_guard_condition_t *
  get_notify_guard_condition() = 0;

  /// Acquire and return a scoped lock that protects the notify guard condition.
  /** This should be used when triggering the notify guard condition. */
  RCLCPP_PUBLIC
  virtual
  std::unique_lock<std::recursive_mutex>
  acquire_notify_guard_condition_lock() const = 0;

  /// Return the default preference for using intra process communication.
  RCLCPP_PUBLIC
  virtual
  bool
  get_use_intra_process_default() const = 0;

  /// Return the default preference for enabling topic statistics collection.
  RCLCPP_PUBLIC
  virtual
  bool
  get_enable_topic_statistics_default() const = 0;

  /// Expand and remap a given topic or service name.
  RCLCPP_PUBLIC
  virtual
  std::string
  resolve_topic_or_service_name(
    const std::string & name, bool is_service, bool only_expand = false) const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_BASE_INTERFACE_HPP_
