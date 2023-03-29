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
  std::function<void(void)> on_notify_waitable_callback)
: notify_waitable_(std::make_shared<ExecutorNotifyWaitable>(on_notify_waitable_callback))
{
}

ExecutorEntitiesCollector::~ExecutorEntitiesCollector()
{
  std::lock_guard<std::mutex> guard{mutex_};

  //  Disassociate each node from this executor collector
  std::for_each(
    weak_nodes_.begin(), weak_nodes_.end(), []
      (rclcpp::node_interfaces::NodeBaseInterface::WeakPtr weak_node_ptr) {
      auto shared_node_ptr = weak_node_ptr.lock();
      if (shared_node_ptr) {
        std::atomic_bool & has_executor = shared_node_ptr->get_associated_with_executor_atomic();
        has_executor.store(false);
      }
    });
  weak_nodes_.clear();

  //  Disassociate each automatically-added callback group from this executor collector
  std::for_each(
    automatically_added_groups_.begin(), automatically_added_groups_.end(), []
      (rclcpp::CallbackGroup::WeakPtr weak_group_ptr) {
      auto group_ptr = weak_group_ptr.lock();
      if (group_ptr) {
        std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
        has_executor.store(false);
      }
    });
  automatically_added_groups_.clear();

  //  Disassociate each manually-added callback group from this executor collector
  std::for_each(
    manually_added_groups_.begin(), manually_added_groups_.end(), []
      (rclcpp::CallbackGroup::WeakPtr weak_group_ptr) {
      auto group_ptr = weak_group_ptr.lock();
      if (group_ptr) {
        std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
        has_executor.store(false);
      }
    });
  manually_added_groups_.clear();
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
  weak_nodes_.push_back(node_ptr);
  this->notify_waitable_->add_guard_condition(&node_ptr->get_notify_guard_condition());

  this->add_automatically_associated_callback_groups({node_ptr});
}

void
ExecutorEntitiesCollector::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  std::lock_guard<std::mutex> guard{mutex_};
  if (!node_ptr->get_associated_with_executor_atomic().load()) {
    throw std::runtime_error("Node needs to be associated with an executor.");
  }

  bool found_node = false;
  auto node_it = weak_nodes_.begin();
  while (node_it != weak_nodes_.end()) {
    bool matched = (node_it->lock() == node_ptr);
    if (matched) {
      found_node = true;
      node_it = weak_nodes_.erase(node_it);
    } else {
      ++node_it;
    }
  }
  if (!found_node) {
    throw std::runtime_error("Node needs to be associated with this executor.");
  }

  for (auto group_it = automatically_added_groups_.begin();
    group_it != automatically_added_groups_.end(); )
  {
    auto group_ptr = group_it->lock();
    if (node_ptr->callback_group_in_node(group_ptr)) {
      std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
      has_executor.store(false);
      group_it = automatically_added_groups_.erase(group_it);
    } else {
      group_it++;
    }
  }

  this->notify_waitable_->remove_guard_condition(&node_ptr->get_notify_guard_condition());
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
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

  this->update_collections();

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
  this->add_automatically_associated_callback_groups(this->weak_nodes_);
  this->prune_invalid_nodes_and_groups();
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
  this->notify_waitable_->add_guard_condition(group_ptr->get_notify_guard_condition().get());
}

void
ExecutorEntitiesCollector::remove_callback_group_from_collection(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  CallbackGroupCollection & collection)
{
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr(group_ptr);
  auto iter = collection.find(weak_group_ptr);
  if (iter != collection.end()) {
    // Check that the group hasn't been orphaned from it's respective node
    if (!group_ptr->has_valid_node()) {
      throw std::runtime_error("Node must not be deleted before its callback group(s).");
    }

    // Disassociate the callback group with the executor
    std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
    has_executor.store(false);
    collection.erase(iter);
    this->notify_waitable_->remove_guard_condition(group_ptr->get_notify_guard_condition().get());
  } else {
    throw std::runtime_error("Callback group needs to be associated with executor.");
  }
}

void
ExecutorEntitiesCollector::add_automatically_associated_callback_groups(
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> nodes_to_check
)
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
      node_it = weak_nodes_.erase(node_it);
    } else {
      node_it++;
    }
  }

  for (auto group_it = automatically_added_groups_.begin();
    group_it != automatically_added_groups_.end(); )
  {
    if (group_it->expired()) {
      group_it = automatically_added_groups_.erase(group_it);
    } else {
      group_it++;
    }
  }
  for (auto group_it = manually_added_groups_.begin();
    group_it != manually_added_groups_.end(); )
  {
    if (group_it->expired()) {
      group_it = manually_added_groups_.erase(group_it);
    } else {
      group_it++;
    }
  }
}

}  // namespace executors
}  // namespace rclcpp
