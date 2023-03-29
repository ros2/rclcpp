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

class ExecutorEntitiesCollector
{
public:
  RCLCPP_PUBLIC
  ExecutorEntitiesCollector();

  RCLCPP_PUBLIC
  ~ExecutorEntitiesCollector();

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

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_all_callback_groups();

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_manually_added_callback_groups();

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups();

  RCLCPP_PUBLIC
  void
  update_collections();

  RCLCPP_PUBLIC
  std::shared_ptr<ExecutorNotifyWaitable>
  get_executor_notify_waitable();

protected:
  using CallbackGroupCollection = std::set<
    rclcpp::CallbackGroup::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>>;

  RCLCPP_PUBLIC
  void
  add_callback_group_to_collection(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    CallbackGroupCollection & collection) RCPPUTILS_TSA_REQUIRES(mutex_);

  RCLCPP_PUBLIC
  void
  remove_callback_group_from_collection(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    CallbackGroupCollection & collection) RCPPUTILS_TSA_REQUIRES(mutex_);

  RCLCPP_PUBLIC
  void
  add_automatically_associated_callback_groups() RCPPUTILS_TSA_REQUIRES(mutex_);

  RCLCPP_PUBLIC
  void
  prune_invalid_nodes_and_groups() RCPPUTILS_TSA_REQUIRES(mutex_);

  mutable std::mutex mutex_;

  CallbackGroupCollection
  manually_added_groups_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  CallbackGroupCollection
  automatically_added_groups_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// nodes that are associated with the executor
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>
  weak_nodes_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  std::shared_ptr<ExecutorNotifyWaitable> notify_waitable_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};
}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EXECUTOR_ENTITIES_COLLECTOR_HPP_
