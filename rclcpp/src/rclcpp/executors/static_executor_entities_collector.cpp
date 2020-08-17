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

#include "rclcpp/executors/static_executor_entities_collector.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/executors/static_single_threaded_executor.hpp"

using rclcpp::executors::StaticExecutorEntitiesCollector;

StaticExecutorEntitiesCollector::~StaticExecutorEntitiesCollector()
{
  // Disassocate all callback groups and thus nodes.
  for (auto & pair : weak_groups_associated_with_executor_to_nodes_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  for (auto & pair : weak_groups_to_nodes_associated_with_executor_) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
    if (node) {
      std::atomic_bool & has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  weak_groups_associated_with_executor_to_nodes_.clear();
  weak_groups_to_nodes_associated_with_executor_.clear();
  exec_list_.clear();
  weak_nodes_.clear();
  weak_groups_to_nodes_.clear();
  weak_nodes_to_guard_conditions_.clear();
}

void
StaticExecutorEntitiesCollector::init(
  rcl_wait_set_t * p_wait_set,
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr & memory_strategy,
  rcl_guard_condition_t * executor_guard_condition)
{
  // Empty initialize executable list
  exec_list_ = rclcpp::experimental::ExecutableList();
  // Get executor's wait_set_ pointer
  p_wait_set_ = p_wait_set;
  // Get executor's memory strategy ptr
  if (memory_strategy == nullptr) {
    throw std::runtime_error("Received NULL memory strategy in executor waitable.");
  }
  memory_strategy_ = memory_strategy;

  // Add executor's guard condition
  memory_strategy_->add_guard_condition(executor_guard_condition);
  // Get memory strategy and executable list. Prepare wait_set_
  execute();
}

void
StaticExecutorEntitiesCollector::execute()
{
  // Fill memory strategy with entities coming from weak_nodes_
  fill_memory_strategy();
  // Fill exec_list_ with entities coming from weak_nodes_ (same as memory strategy)
  fill_executable_list();
  // Resize the wait_set_ based on memory_strategy handles (rcl_wait_set_resize)
  prepare_wait_set();
}

void
StaticExecutorEntitiesCollector::fill_memory_strategy()
{
  memory_strategy_->clear_handles();
  bool has_invalid_weak_groups_or_nodes =
    memory_strategy_->collect_entities(weak_groups_to_nodes_associated_with_executor_);
  // Clean up any invalid nodes, if they were detected
  if (has_invalid_weak_nodes) {
    std::vector<rclcpp::CallbackGroup::WeakPtr> invalid_group_ptrs;
    for (auto pair : weak_groups_to_nodes_associated_with_executor_) {
      auto weak_group_ptr = pair.first;
      auto weak_node_ptr = pair.second;
      if (weak_group_ptr.expired() || weak_node_ptr.expired()) {
        invalid_group_ptrs.push_back(weak_group_ptr);
      }
    }
    std::for_each(
      invalid_group_ptrs.begin(), invalid_group_ptrs.end(),
      [this](rclcpp::CallbackGroup::WeakPtr group_ptr) {
        weak_groups_to_nodes_associated_with_executor_.erase(group_ptr);
      });
  }
  has_invalid_weak_groups_or_nodes =
    memory_strategy_->collect_entities(weak_groups_associated_with_executor_to_nodes_);
  // Clean up any invalid nodes, if they were detected
  if (has_invalid_weak_nodes) {
    std::vector<rclcpp::CallbackGroup::WeakPtr> invalid_group_ptrs;
    for (auto pair : weak_groups_associated_with_executor_to_nodes_) {
      auto weak_group_ptr = pair.first;
      auto weak_node_ptr = pair.second;
      if (weak_group_ptr.expired() || weak_node_ptr.expired()) {
        invalid_group_ptrs.push_back(weak_group_ptr);
      }
    }
    std::for_each(
      invalid_group_ptrs.begin(), invalid_group_ptrs.end(),
      [this](rclcpp::CallbackGroup::WeakPtr group_ptr) {
        weak_groups_associated_with_executor_to_nodes_.erase(group_ptr);
      });
  }

  // Add the static executor waitable to the memory strategy
  memory_strategy_->add_waitable_handle(this->shared_from_this());
}

void
StaticExecutorEntitiesCollector::fill_executable_list()
{
  add_allowable_unassigned_callback_groups();
  fill_executable_list_from_map(weak_groups_associated_with_executor_to_nodes_);
  fill_executable_list_from_map(weak_groups_to_nodes_associated_with_executor_);
}
void
StaticExecutorEntitiesCollector::fill_executable_list_from_map(WeakCallbackGroupsToNodesMap weak_groups_to_nodes)
{
  exec_list_.clear();
  for (const auto & pair : weak_groups_to_nodes_) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (!node || !group || !group->can_be_taken_from().load()) {
      continue;
    }
    group->find_timer_ptrs_if(
      [this](const rclcpp::TimerBase::SharedPtr & timer) {
        if (timer) {
          exec_list_.add_timer(timer);
        }
        return false;
      });
    group->find_subscription_ptrs_if(
      [this](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
        if (subscription) {
          exec_list_.add_subscription(subscription);
        }
        return false;
      });
    group->find_service_ptrs_if(
      [this](const rclcpp::ServiceBase::SharedPtr & service) {
        if (service) {
          exec_list_.add_service(service);
        }
        return false;
      });
    group->find_client_ptrs_if(
      [this](const rclcpp::ClientBase::SharedPtr & client) {
        if (client) {
          exec_list_.add_client(client);
        }
        return false;
      });
    group->find_waitable_ptrs_if(
      [this](const rclcpp::Waitable::SharedPtr & waitable) {
        if (waitable) {
          exec_list_.add_waitable(waitable);
        }
        return false;
      });
  }

  // Add the executor's waitable to the executable list
  exec_list_.add_waitable(shared_from_this());
}

void
StaticExecutorEntitiesCollector::prepare_wait_set()
{
  // clear wait set
  if (rcl_wait_set_clear(p_wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }

  // The size of waitables are accounted for in size of the other entities
  rcl_ret_t ret = rcl_wait_set_resize(
    p_wait_set_, memory_strategy_->number_of_ready_subscriptions(),
    memory_strategy_->number_of_guard_conditions(), memory_strategy_->number_of_ready_timers(),
    memory_strategy_->number_of_ready_clients(), memory_strategy_->number_of_ready_services(),
    memory_strategy_->number_of_ready_events());

  if (RCL_RET_OK != ret) {
    throw std::runtime_error(
            std::string("Couldn't resize the wait set : ") + rcl_get_error_string().str);
  }
}

void
StaticExecutorEntitiesCollector::refresh_wait_set(std::chrono::nanoseconds timeout)
{
  // clear wait set (memeset to '0' all wait_set_ entities
  // but keeps the wait_set_ number of entities)
  if (rcl_wait_set_clear(p_wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }

  if (!memory_strategy_->add_handles_to_wait_set(p_wait_set_)) {
    throw std::runtime_error("Couldn't fill wait set");
  }

  rcl_ret_t status =
    rcl_wait(p_wait_set_, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());

  if (status == RCL_RET_WAIT_SET_EMPTY) {
    RCUTILS_LOG_WARN_NAMED(
      "rclcpp",
      "empty wait set received in rcl_wait(). This should never happen.");
  } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(status, "rcl_wait() failed");
  }
}

bool
StaticExecutorEntitiesCollector::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  // Add waitable guard conditions (one for each registered node) into the wait set.
  for (const auto & pair : weak_nodes_to_guard_conditions_) {
    auto & gc = pair.second;
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, gc, NULL);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Executor waitable: couldn't add guard condition to wait set");
    }
  }
  return true;
}

size_t StaticExecutorEntitiesCollector::get_number_of_ready_guard_conditions()
{
  return weak_nodes_to_guard_conditions_.size();
}

bool
StaticExecutorEntitiesCollector::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  bool is_new_node = false;
  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Node has already been added to an executor.");
  }
  for (auto & weak_group : node_ptr->get_callback_groups()) {
    auto group_ptr = weak_group.lock();
    if (group_ptr != nullptr && !group_ptr->get_associated_with_executor_atomic().load() &&
      group_ptr->automatically_add_to_executor_with_node())
    {
      is_new_node = (add_callback_groups_from_node_associated_with_executor(group_ptr, node_ptr)
      || is_new_node;
    }
  }
  return is_new_node;
}

bool
StaticExecutorEntitiesCollector::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  // If the callback_group already has an executor
  std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto insert_info = weak_groups_associated_with_executor_to_nodes_.insert(std::make_pair(weak_group_ptr, node_ptr));
  bool was_inserted = insert_info.second;
  if (!was_inserted) {
    throw std::runtime_error("Callback group was already added to executor.");
  }
  bool is_new_node = !has_node_from_callback_groups_associated_with_executor(node_ptr) && !has_node_associated_with_executor(node_ptr);
  if (is_new_node) {
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr node_weak_ptr(node_ptr);
    weak_nodes_to_guard_conditions_[node_weak_ptr] = node_ptr->get_notify_guard_condition();
    return true;
  }
  return false;
}

bool
StaticExecutorEntitiesCollector::add_callback_groups_from_node_associated_with_executor(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  // If the callback_group already has an executor
  std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto insert_info = weak_groups_to_nodes_associated_with_executor_.insert(std::make_pair(weak_group_ptr, node_ptr));
  bool was_inserted = insert_info.second;
  if (!was_inserted) {
    throw std::runtime_error("Callback group was already added to executor.");
  }
  bool is_new_node = !has_node_from_callback_groups_associated_with_executor(node_ptr) && !has_node_associated_with_executor(node_ptr);
  if (is_new_node) {
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr node_weak_ptr(node_ptr);
    weak_nodes_to_guard_conditions_[node_weak_ptr] = node_ptr->get_notify_guard_condition();
    return true;
  }
  return false;
}

bool
StaticExecutorEntitiesCollector::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr)
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr;
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto iter = weak_groups_associated_with_executor_to_nodes_.find(weak_group_ptr);
  if (iter != weak_groups_associated_with_executor_to_nodes_.end()) {
    node_ptr = iter->second.lock();
    if (node_ptr == nullptr) {
      throw std::runtime_error("Node must not be deleted before its callback group(s).");
    }
    weak_groups_associated_with_executor_to_nodes_.erase(iter);
    std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
    has_executor.store(false);
  } else {
    throw std::runtime_error("Callback group needs to be associated with executor.");
  }
  // If the node was matched and removed, interrupt waiting.
  if (!has_node_from_callback_groups_associated_with_executor(node_ptr) &&
      !has_node_associated_with_executor(node_ptr)) {
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr node_weak_ptr(node_ptr);
    weak_nodes_to_guard_conditions_.erase(node_weak_ptr);
    return true;
  }
  return false;
}

bool
StaticExecutorEntitiesCollector::remove_callback_group_from_node_associated_with_executor(
  rclcpp::CallbackGroup::SharedPtr group_ptr)
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr;
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto iter = weak_groups_to_nodes_associated_with_executor_.find(weak_group_ptr);
  if (iter != weak_groups_to_nodes_associated_with_executor_.end()) {
    node_ptr = iter->second.lock();
    if (node_ptr == nullptr) {
      throw std::runtime_error("Node must not be deleted before its callback group(s).");
    }
    weak_groups_to_nodes_associated_with_executor_.erase(iter);
  } else {
    throw std::runtime_error("Callback group needs to be associated with executor.");
  }
  // If the node was matched and removed, interrupt waiting.
  if (!has_node_from_callback_groups_associated_with_executor(node_ptr) &&
      !has_node_associated_with_executor(node_ptr)) {
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr node_weak_ptr(node_ptr);
    weak_nodes_to_guard_conditions_.erase(node_weak_ptr);
    return true;
  }
  return false;
}

bool
StaticExecutorEntitiesCollector::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  std::vector<rclcpp::CallbackGroup::SharedPtr> found_group_ptrs;
  std::for_each(
    weak_groups_to_nodes_associated_with_executor_.begin(), weak_groups_to_nodes_associated_with_executor_.end(),
    [&found_group_ptrs, node_ptr](std::pair<rclcpp::CallbackGroup::WeakPtr,
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> key_value_pair) {
      auto weak_node_ptr = key_value_pair.second;
      auto shared_node_ptr = weak_node_ptr.lock();
      auto group_ptr = key_value_pair.first.lock();
      if (shared_node_ptr == node_ptr) {
        found_group_ptrs.push_back(group_ptr);
      }
    });
  std::for_each(
    found_group_ptrs.begin(), found_group_ptrs.end(), [this]
      (rclcpp::CallbackGroup::SharedPtr group_ptr) {
      remove_callback_group_from_node_associated_with_executor(group_ptr);
    });
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
  if(!found_group_ptrs.empty()) {
    return true;
  } else {
    return false;
  }
}

bool
StaticExecutorEntitiesCollector::is_ready(rcl_wait_set_t * p_wait_set)
{
  // Check wait_set guard_conditions for added/removed entities to/from a node
  for (size_t i = 0; i < p_wait_set->size_of_guard_conditions; ++i) {
    if (p_wait_set->guard_conditions[i] != NULL) {
      auto found_guard_condition = std::find_if(
        weak_nodes_to_guard_conditions_.begin(), weak_nodes_to_guard_conditions_.end(),
        [&](std::pair<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
        const rcl_guard_condition_t *> pair) -> bool {
          return pair.second == p_wait_set->guard_conditions[i];
        });
      if (found_guard_condition != weak_nodes_to_guard_conditions_.end()) {
        return true;
      }
    }
  }
  // None of the guard conditions triggered belong to a registered node
  return false;
}

// Returns true iff the weak_groups_to_nodes_ map has node_ptr as the value in any of its entry.
bool
StaticExecutorEntitiesCollector::has_node_from_callback_groups_associated_with_executor(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr) const
{
  return std::find_if(
    weak_groups_associated_with_executor_to_nodes_.begin(), weak_groups_associated_with_executor_to_nodes_.end(),
    [&](const WeakCallbackGroupsToNodesMap::value_type & other) -> bool {
      auto other_ptr = other.second.lock();
      return other_ptr == node_ptr;
    }) != weak_groups_associated_with_executor_to_nodes_.end();
}

// Returns true iff the weak_groups_to_nodes_ map has node_ptr as the value in any of its entry.
bool
StaticExecutorEntitiesCollector::has_node_associated_with_executor(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr) const
{
  return std::find_if(
    weak_groups_to_nodes_associated_with_executor_.begin(), weak_groups_to_nodes_associated_with_executor_.end(),
    [&](const WeakCallbackGroupsToNodesMap::value_type & other) -> bool {
      auto other_ptr = other.second.lock();
      return other_ptr == node_ptr;
    }) != weak_groups_to_nodes_associated_with_executor_.end();
}
bool
StaticExecutorEntitiesCollector::has_callback_group(
  const rclcpp::CallbackGroup::SharedPtr group_ptr) const
{
  return std::find_if(
    weak_groups_to_nodes_.begin(), weak_groups_to_nodes_.end(),
    [&](const WeakCallbackGroupsToNodesMap::value_type & other) -> bool {
      auto other_ptr = other.first.lock();
      return other_ptr == group_ptr;
    }) != weak_groups_to_nodes_.end();
}

void
StaticExecutorEntitiesCollector::add_allowable_unassigned_callback_groups()
{
  for (auto & other : weak_groups_to_nodes_associated_with_executor_) {
    auto node = other.second.lock();
    if (node) {
      auto group_ptrs = node->get_callback_groups();
      std::for_each(
        group_ptrs.begin(), group_ptrs.end(),
        [this, node](rclcpp::CallbackGroup::WeakPtr group_ptr)
        {
          auto shared_group_ptr = group_ptr.lock();
          if (shared_group_ptr && shared_group_ptr->automatically_add_to_executor_with_node() &&
          !shared_group_ptr->get_associated_with_executor_atomic().load())
          {
            add_callback_groups_from_node_associated_with_executor(shared_group_ptr, node);
          }
        });
    }
  }
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticExecutorEntitiesCollector::get_all_callback_groups()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  for (auto const & group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(group_node_ptr.first);
  }
  for (auto const & group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticExecutorEntitiesCollector::get_callback_groups_associated_with_executor()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  for (auto const & group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticExecutorEntitiesCollector::get_callback_groups_from_nodes_associated_with_executor()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  for (auto const & group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}
