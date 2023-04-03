// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__EXECUTOR_ENTITIES_COLLECTOR_HPP_
#define RCLCPP__EXECUTORS__EXECUTOR_ENTITIES_COLLECTOR_HPP_

#include <list>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"

#include <rclcpp/any_executable.hpp>
#include <rclcpp/node_interfaces/node_base.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/executor_notify_waitable.hpp>
#include <rclcpp/visibility_control.hpp>
#include <rclcpp/wait_set.hpp>
#include <rclcpp/wait_result.hpp>

namespace rclcpp
{
namespace executors
{

/// Class to monitor a set of nodes and callback groups for changes in entity membership
/**
 * This is to be used with an executor to track the membership of various nodes, groups,
 * and entities (timers, subscriptions, clients, services, etc) and report status to the
 * executor.
 *
 * In general, users will add either nodes or callback groups to an executor.
 * Each node may have callback groups that are automatically associated with executors,
 * or callback groups that must be manually associated with an executor.
 *
 * This object tracks both types of callback groups as well as nodes that have been
 * previously added to the executor.
 * When a new callback group is added/removed or new entities are added/removed, the
 * corresponding node or callback group will signal this to the executor so that the
 * entity collection may be rebuilt according to that executor's implementation.
 *
 */
class ExecutorEntitiesCollector
{
public:
  /// Constructor
  /**
   * \param[in] on_notify_waitable Callback to execute when one of the associated
   *   nodes or callback groups trigger their guard condition, indicating that their
   *   corresponding entities have changed.
   */
  RCLCPP_PUBLIC
  explicit ExecutorEntitiesCollector(
    std::shared_ptr<ExecutorNotifyWaitable> notify_waitable);

  /// Destructor
  RCLCPP_PUBLIC
  ~ExecutorEntitiesCollector();

  bool has_pending();

  /// Add a node to the entity collector
  /**
   * \param[in] node_ptr a shared pointer that points to a node base interface
   * \throw std::runtime_error if the node is associated with an executor
   */
  RCLCPP_PUBLIC
  void
  add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  /// Remove a node from the entity collector
  /**
   * \param[in] node_ptr a shared pointer that points to a node base interface
   * \throw std::runtime_error if the node is associated with an executor
   * \throw std::runtime_error if the node is associated with this executor
   */
  RCLCPP_PUBLIC
  void
  remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  /// Check if a node is associated with this executor/collector.
  /**
   * \param[in] node_ptr a shared pointer that points to a node base interface
   * \return true if the node is tracked in this collector, false otherwise
   */
  RCLCPP_PUBLIC
  bool
  has_node(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  /// Add a callback group to the entity collector
  /**
   * \param[in] group_ptr a shared pointer that points to a callback group
   * \throw std::runtime_error if the callback_group is associated with an executor
   */
  RCLCPP_PUBLIC
  void
  add_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr);

  /// Remove a callback group from the entity collector
  /**
   * \param[in] group_ptr a shared pointer that points to a callback group
   * \throw std::runtime_error if the callback_group is not associated with an executor
   * \throw std::runtime_error if the callback_group is not associated with this executor
   */
  RCLCPP_PUBLIC
  void
  remove_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr);

  /// Get all callback groups known to this entity collector
  /**
   * This will include manually added and automatically added (node associated) groups
   * \return vector of all callback groups
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_all_callback_groups();

  /// Get manually-added callback groups known to this entity collector
  /**
   * This will include callback groups that have been added via add_callback_group
   * \return vector of manually-added callback groups
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_manually_added_callback_groups();

  /// Get automatically-added callback groups known to this entity collector
  /**
   * This will include callback groups that are associated with nodes added via add_node
   * \return vector of automatically-added callback groups
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups();

  /// Update the underlying collections
  /**
   * This will prune nodes and callback groups that are no longer valid as well
   * as adding new callback groups from any associated nodes.
   */
  RCLCPP_PUBLIC
  void
  update_collections();

protected:
  using NodeCollection = std::set<
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>>;

  using CallbackGroupCollection = std::set<
    rclcpp::CallbackGroup::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>>;

  using WeakNodesToGuardConditionsMap = std::map<
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    rclcpp::GuardCondition::WeakPtr,
    std::owner_less<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>>;

  using WeakGroupsToGuardConditionsMap = std::map<
    rclcpp::CallbackGroup::WeakPtr,
    rclcpp::GuardCondition::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>>;

  /// Implementation of removing a node from the collector.
  /**
   * This will disassociate the node from the collector and remove any
   * automatically-added callback groups
   *
   * This takes and returns an iterator so it may be used as:
   *
   * it = remove_weak_node(it);
   *
   * \param[in] weak_node iterator to the weak node to be removed
   * \return Valid updated iterator in the same collection
   */
  RCLCPP_PUBLIC
  NodeCollection::iterator
  remove_weak_node(NodeCollection::iterator weak_node);

  /// Implementation of removing a callback gruop from the collector.
  /**
   * This will disassociate the callback group from the collector
   *
   * This takes and returns an iterator so it may be used as:
   *
   * it = remove_weak_callback_group(it);
   *
   * \param[in] weak_group_it iterator to the weak group to be removed
   * \param[in] collection the collection to remove the group from
   *   (manually or automatically added)
   * \return Valid updated iterator in the same collection
   */
  RCLCPP_PUBLIC
  CallbackGroupCollection::iterator
  remove_weak_callback_group(
    CallbackGroupCollection::iterator weak_group_it,
    CallbackGroupCollection & collection);

  /// Implementation of adding a callback group
  /**
   * \param[in] group_ptr the group to add
   * \param[in] collection the collection to add the group to
   */
  RCLCPP_PUBLIC
  void
  add_callback_group_to_collection(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    CallbackGroupCollection & collection);

  /// Implementation of removing a callback group
  /**
   * \param[in] group_ptr the group to remove
   * \param[in] collection the collection to remove the group from
   */
  RCLCPP_PUBLIC
  void
  remove_callback_group_from_collection(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    CallbackGroupCollection & collection);

  /// Iterate over queued added/remove nodes and callback_groups
  RCLCPP_PUBLIC
  void
  process_queues();

  /// Check a collection of nodes and add any new callback_groups that
  /// are set to be automatically associated via the node.
  RCLCPP_PUBLIC
  void
  add_automatically_associated_callback_groups(
    const NodeCollection & nodes_to_check);

  /// Check all nodes and group for expired weak pointers and remove them.
  RCLCPP_PUBLIC
  void
  prune_invalid_nodes_and_groups();

  /// Callback groups that were added via `add_callback_group`
  CallbackGroupCollection manually_added_groups_;

  /// Callback groups that were added by their association with added nodes
  CallbackGroupCollection automatically_added_groups_;

  /// nodes that are associated with the executor
  NodeCollection weak_nodes_;

  /// mutex to protect pending queues
  std::mutex pending_mutex_;

  /// nodes that have been added since the last update.
  NodeCollection pending_added_nodes_;

  /// nodes that have been removed since the last update.
  NodeCollection pending_removed_nodes_;

  /// callback groups that have been added since the last update.
  CallbackGroupCollection pending_manually_added_groups_;

  /// callback groups that have been removed since the last update.
  CallbackGroupCollection pending_manually_removed_groups_;

  /// Track guard conditions associated with added nodes
  WeakNodesToGuardConditionsMap weak_nodes_to_guard_conditions_;

  /// Track guard conditions associated with added callback groups
  WeakGroupsToGuardConditionsMap weak_groups_to_guard_conditions_;

  /// Waitable to add guard conditions to
  std::shared_ptr<ExecutorNotifyWaitable> notify_waitable_;
};
}  // namespace executors
}  // namespace rclcpp
//
#endif  // RCLCPP__EXECUTORS__EXECUTOR_ENTITIES_COLLECTOR_HPP_
