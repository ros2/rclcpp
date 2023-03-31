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

#include <set>

#include "rclcpp/executors/executor_entities_collector.hpp"
#include "rclcpp/executors/executor_notify_waitable.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"

namespace rclcpp
{
namespace executors
{

ExecutorEntitiesCollector::ExecutorEntitiesCollector(
  std::shared_ptr<ExecutorNotifyWaitable> notify_waitable)
: notify_waitable_(notify_waitable)
{
}

ExecutorEntitiesCollector::~ExecutorEntitiesCollector()
{
  std::lock_guard<std::mutex> guard{mutex_};

  for (auto weak_node_it = weak_nodes_.begin(); weak_node_it != weak_nodes_.end(); ) {
    weak_node_it = remove_weak_node(weak_node_it);
  }

  for (auto weak_group_it = automatically_added_groups_.begin();
    weak_group_it != automatically_added_groups_.end(); )
  {
    weak_group_it = remove_weak_callback_group(weak_group_it, automatically_added_groups_);
  }

  for (auto weak_group_it = manually_added_groups_.begin();
    weak_group_it != manually_added_groups_.end(); )
  {
    weak_group_it = remove_weak_callback_group(weak_group_it, manually_added_groups_);
  }
}

void
ExecutorEntitiesCollector::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  std::lock_guard<std::mutex> guard{mutex_};

  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error(
            std::string("Node '") + node_ptr->get_fully_qualified_name() +
            "' has already been added to an executor.");
  }
  weak_nodes_.insert(node_ptr);
  this->add_automatically_associated_callback_groups({node_ptr});

  // Store node guard condition in map and add it to the notify waitable
  rclcpp::GuardCondition * node_guard_condition = &node_ptr->get_notify_guard_condition();
  weak_nodes_to_guard_conditions_.insert({node_ptr, node_guard_condition});

  this->notify_waitable_->add_guard_condition(node_guard_condition);
}

void
ExecutorEntitiesCollector::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  std::lock_guard<std::mutex> guard{mutex_};

  if (!node_ptr->get_associated_with_executor_atomic().load()) {
    throw std::runtime_error("Node needs to be associated with an executor.");
  }

  auto node_it = weak_nodes_.find(node_ptr);
  if (node_it != weak_nodes_.end()) {
    remove_weak_node(node_it);
  } else {
    throw std::runtime_error("Node needs to be associated with this executor.");
  }

  for (auto group_it = automatically_added_groups_.begin();
    group_it != automatically_added_groups_.end(); )
  {
    auto group_ptr = group_it->lock();
    if (node_ptr->callback_group_in_node(group_ptr)) {
      group_it = remove_weak_callback_group(group_it, automatically_added_groups_);
    } else {
      ++group_it;
    }
  }
}

void
ExecutorEntitiesCollector::add_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr)
{
  std::lock_guard<std::mutex> guard{mutex_};
  this->add_callback_group_to_collection(group_ptr, manually_added_groups_);
}

void
ExecutorEntitiesCollector::remove_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr)
{
  std::lock_guard<std::mutex> guard{mutex_};
  this->remove_callback_group_from_collection(group_ptr, manually_added_groups_);
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
ExecutorEntitiesCollector::get_all_callback_groups()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  std::lock_guard<std::mutex> guard{mutex_};

  for (const auto & group_ptr : manually_added_groups_) {
    groups.push_back(group_ptr);
  }
  for (auto const & group_ptr : automatically_added_groups_) {
    groups.push_back(group_ptr);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
ExecutorEntitiesCollector::get_manually_added_callback_groups()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  std::lock_guard<std::mutex> guard{mutex_};
  for (const auto & group_ptr : manually_added_groups_) {
    groups.push_back(group_ptr);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
ExecutorEntitiesCollector::get_automatically_added_callback_groups()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  std::lock_guard<std::mutex> guard{mutex_};
  for (auto const & group_ptr : automatically_added_groups_) {
    groups.push_back(group_ptr);
  }
  return groups;
}

void
ExecutorEntitiesCollector::update_collections()
{
  std::lock_guard<std::mutex> guard{mutex_};
  this->add_automatically_associated_callback_groups(this->weak_nodes_);
  this->prune_invalid_nodes_and_groups();
}

ExecutorEntitiesCollector::NodeCollection::iterator
ExecutorEntitiesCollector::remove_weak_node(NodeCollection::iterator weak_node)
{
  // Disassociate the guard condition from the executor notify waitable
  auto guard_condition_it = weak_nodes_to_guard_conditions_.find(*weak_node);
  if (guard_condition_it != weak_nodes_to_guard_conditions_.end()) {
    this->notify_waitable_->remove_guard_condition(guard_condition_it->second);
    weak_nodes_to_guard_conditions_.erase(guard_condition_it);
  }

  // Mark the node as disassociated (if the node is still valid)
  auto node_ptr = weak_node->lock();
  if (node_ptr) {
    std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
    has_executor.store(false);
  }

  // Remove the node from tracked nodes
  return weak_nodes_.erase(weak_node);
}

ExecutorEntitiesCollector::CallbackGroupCollection::iterator
ExecutorEntitiesCollector::remove_weak_callback_group(
  CallbackGroupCollection::iterator weak_group_it,
  CallbackGroupCollection & collection
)
{
  // Disassociate the guard condition from the executor notify waitable
  auto guard_condition_it = weak_groups_to_guard_conditions_.find(*weak_group_it);
  if (guard_condition_it != weak_groups_to_guard_conditions_.end()) {
    this->notify_waitable_->remove_guard_condition(guard_condition_it->second);
    weak_groups_to_guard_conditions_.erase(guard_condition_it);
  }

  // Mark the node as disassociated (if the group is still valid)
  auto group_ptr = weak_group_it->lock();

  if (group_ptr) {
    if (!group_ptr->has_valid_node()) {
      throw std::runtime_error("Node must not be deleted before its callback group(s).");
    }
    std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
    has_executor.store(false);
  }

  // Remove the node from tracked nodes
  return collection.erase(weak_group_it);
}

void
ExecutorEntitiesCollector::add_callback_group_to_collection(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  CallbackGroupCollection & collection)
{
  std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }
  auto iter = collection.insert(group_ptr);
  if (iter.second == false) {
    throw std::runtime_error("Callback group has already been added to this executor.");
  }

  // Store node guard condition in map and add it to the notify waitable
  rclcpp::GuardCondition * group_guard_condition = group_ptr->get_notify_guard_condition().get();
  weak_groups_to_guard_conditions_.insert({group_ptr, group_guard_condition});
  this->notify_waitable_->add_guard_condition(group_guard_condition);
}

void
ExecutorEntitiesCollector::remove_callback_group_from_collection(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  CallbackGroupCollection & collection)
{
  auto group_it = collection.find(group_ptr);
  remove_weak_callback_group(group_it, collection);
}

void
ExecutorEntitiesCollector::add_automatically_associated_callback_groups(
  const NodeCollection & nodes_to_check)
{
  for (auto & weak_node : nodes_to_check) {
    auto node = weak_node.lock();
    if (node) {
      node->for_each_callback_group(
        [this, node](rclcpp::CallbackGroup::SharedPtr group_ptr)
        {
          if (!group_ptr->get_associated_with_executor_atomic().load() &&
          group_ptr->automatically_add_to_executor_with_node())
          {
            this->add_callback_group_to_collection(group_ptr, this->automatically_added_groups_);
          }
        });
    }
  }
}

void
ExecutorEntitiesCollector::prune_invalid_nodes_and_groups()
{
  for (auto node_it = weak_nodes_.begin();
    node_it != weak_nodes_.end(); )
  {
    if (node_it->expired()) {
      node_it = remove_weak_node(node_it);
    } else {
      node_it++;
    }
  }
  for (auto group_it = automatically_added_groups_.begin();
    group_it != automatically_added_groups_.end(); )
  {
    if (group_it->expired()) {
      group_it = remove_weak_callback_group(group_it, automatically_added_groups_);
    } else {
      group_it++;
    }
  }
  for (auto group_it = manually_added_groups_.begin();
    group_it != manually_added_groups_.end(); )
  {
    if (group_it->expired()) {
      group_it = remove_weak_callback_group(group_it, manually_added_groups_);
    } else {
      group_it++;
    }
  }
}

}  // namespace executors
}  // namespace rclcpp
