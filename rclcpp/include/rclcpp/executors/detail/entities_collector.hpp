// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__DETAIL__ENTITIES_COLLECTOR_HPP_
#define RCLCPP__EXECUTORS__DETAIL__ENTITIES_COLLECTOR_HPP_

#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace executors
{
namespace detail
{
typedef std::map<rclcpp::CallbackGroup::WeakPtr,
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>> WeakCallbackGroupsToNodesMap;

class EntitiesCollector
  : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(EntitiesCollector)

  // Constructor
  RCLCPP_PUBLIC
  EntitiesCollector() = default;

  // Destructor
  RCLCPP_PUBLIC
  virtual ~EntitiesCollector();

  /// Take the data so that it can be consumed with `execute`.
  /**
   * For `EntitiesCollector`, this always return `nullptr`.
   * \sa rclcpp::Waitable::take_data()
   */
  RCLCPP_PUBLIC
  std::shared_ptr<void>
  take_data() override;

  /// Add a callback group to an executor.
  /**
   * \see rclcpp::Executor::add_callback_group
   */
  RCLCPP_PUBLIC
  bool
  add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  /// Add a callback group to an executor.
  /**
   * \see rclcpp::Executor::add_callback_group
   * \return boolean whether the node from the callback group is new
   */
  RCLCPP_PUBLIC
  bool
  add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    WeakCallbackGroupsToNodesMap & weak_groups_to_nodes);

  /// Remove a callback group from the executor.
  /**
   * \see rclcpp::Executor::remove_callback_group
   */
  RCLCPP_PUBLIC
  bool
  remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr);

  /// Remove a callback group from the executor.
  /**
   * \see rclcpp::Executor::remove_callback_group_from_map
   */
  RCLCPP_PUBLIC
  bool
  remove_callback_group_from_map(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    WeakCallbackGroupsToNodesMap & weak_groups_to_nodes);

  /**
   * \see rclcpp::Executor::add_node()
   * \throw std::runtime_error if node was already added
   */
  RCLCPP_PUBLIC
  bool
  add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  /**
   * \see rclcpp::Executor::remove_node()
   * \throw std::runtime_error if no guard condition is associated with node.
   */
  RCLCPP_PUBLIC
  bool
  remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_all_callback_groups();

  /// Get callback groups that belong to executor.
  /**
   * \see rclcpp::Executor::get_manually_added_callback_groups()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_manually_added_callback_groups();

  /// Get callback groups that belong to executor.
  /**
   * \see rclcpp::Executor::get_automatically_added_callback_groups_from_nodes()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups_from_nodes();

protected:
  virtual
  void
  callback_group_added_impl(
    rclcpp::CallbackGroup::SharedPtr group) = 0;

  virtual
  void
  node_added_impl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) = 0;

  virtual
  void
  callback_group_removed_impl(
    rclcpp::CallbackGroup::SharedPtr group) = 0;

  virtual
  void
  node_removed_impl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) = 0;

  /// Return true if the node belongs to the collector
  /**
   * \param[in] node_ptr a node base interface shared pointer
   * \param[in] weak_groups_to_nodes map to nodes to lookup
   * \return boolean whether a node belongs the collector
   */
  bool
  has_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) const;

  /// Add all callback groups that can be automatically added by any executor
  /// and is not already associated with an executor from nodes
  /// that are associated with executor
  /**
   * \see rclcpp::Executor::add_callback_groups_from_nodes_associated_to_executor()
   */
  void
  add_callback_groups_from_nodes_associated_to_executor();

  // maps callback groups to nodes.
  WeakCallbackGroupsToNodesMap weak_groups_associated_with_executor_to_nodes_;
  // maps callback groups to nodes.
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes_associated_with_executor_;

  /// List of weak nodes registered in the static executor
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;
};

}  // namespace detail
}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__DETAIL__ENTITIES_COLLECTOR_HPP_
