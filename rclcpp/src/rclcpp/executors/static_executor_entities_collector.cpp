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

#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/executors/static_single_threaded_executor.hpp"

using rclcpp::executors::StaticExecutorEntitiesCollector;

StaticExecutorEntitiesCollector::~StaticExecutorEntitiesCollector()
{
  // Disassociate all nodes
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      std::atomic_bool & has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  exec_list_.clear();
  weak_nodes_.clear();
  guard_conditions_.clear();
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
  guard_conditions_.push_back(executor_guard_condition);

  // Get memory strategy and executable list. Prepare wait_set_
  execute();
}

void
StaticExecutorEntitiesCollector::fini()
{
  memory_strategy_->clear_handles();
  exec_list_.clear();
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
  bool has_invalid_weak_nodes = memory_strategy_->collect_entities(weak_nodes_);

  // Clean up any invalid nodes, if they were detected
  if (has_invalid_weak_nodes) {
    auto node_it = weak_nodes_.begin();
    while (node_it != weak_nodes_.end()) {
      if (node_it->expired()) {
        node_it = weak_nodes_.erase(node_it);
      } else {
        ++node_it;
      }
    }
  }

  // Add the static executor waitable to the memory strategy
  memory_strategy_->add_waitable_handle(this->shared_from_this());
}

void
StaticExecutorEntitiesCollector::fill_executable_list()
{
  exec_list_.clear();

  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
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
  for (const auto & gc : guard_conditions_) {
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, gc, NULL);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Executor waitable: couldn't add guard condition to wait set");
    }
  }
  return true;
}

size_t StaticExecutorEntitiesCollector::get_number_of_ready_guard_conditions()
{
  return guard_conditions_.size();
}

void
StaticExecutorEntitiesCollector::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  // Check to ensure node not already added
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node == node_ptr) {
      // TODO(jacquelinekay): Use a different error here?
      throw std::runtime_error("Cannot add node to executor, node already added.");
    }
  }

  weak_nodes_.push_back(node_ptr);
  guard_conditions_.push_back(node_ptr->get_notify_guard_condition());
}

bool
StaticExecutorEntitiesCollector::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  auto node_it = weak_nodes_.begin();

  while (node_it != weak_nodes_.end()) {
    bool matched = (node_it->lock() == node_ptr);
    if (matched) {
      // Find and remove node and its guard condition
      auto gc_it = std::find(
        guard_conditions_.begin(),
        guard_conditions_.end(),
        node_ptr->get_notify_guard_condition());

      if (gc_it != guard_conditions_.end()) {
        guard_conditions_.erase(gc_it);
        weak_nodes_.erase(node_it);
        return true;
      }

      throw std::runtime_error("Didn't find guard condition associated with node.");

    } else {
      ++node_it;
    }
  }

  return false;
}

bool
StaticExecutorEntitiesCollector::is_ready(rcl_wait_set_t * p_wait_set)
{
  // Check wait_set guard_conditions for added/removed entities to/from a node
  for (size_t i = 0; i < p_wait_set->size_of_guard_conditions; ++i) {
    if (p_wait_set->guard_conditions[i] != NULL) {
      // Check if the guard condition triggered belongs to a node
      auto it = std::find(
        guard_conditions_.begin(), guard_conditions_.end(),
        p_wait_set->guard_conditions[i]);

      // If it does, we are ready to re-collect entities
      if (it != guard_conditions_.end()) {
        return true;
      }
    }
  }
  // None of the guard conditions triggered belong to a registered node
  return false;
}
