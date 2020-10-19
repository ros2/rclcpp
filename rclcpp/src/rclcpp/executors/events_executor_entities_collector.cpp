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

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/executors/events_executor.hpp"
#include "rclcpp/executors/events_executor_entities_collector.hpp"

using rclcpp::executors::EventsExecutorEntitiesCollector;

EventsExecutorEntitiesCollector::EventsExecutorEntitiesCollector(
  EventsExecutor * executor_context,
  TimersManager::SharedPtr timers_manager)
{
  associated_executor_ = executor_context;
  timers_manager_ = timers_manager;
}

EventsExecutorEntitiesCollector::~EventsExecutorEntitiesCollector()
{
  // Disassociate all callback groups
  for (const auto & pair : weak_groups_associated_with_executor_to_nodes_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
      unset_callback_group_entities_callbacks(group);
    }
  }
  for (const auto & pair : weak_groups_to_nodes_associated_with_executor_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
      unset_callback_group_entities_callbacks(group);
    }
  }

  // Disassociate all nodes
  for (const auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      std::atomic_bool & has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }

  // Unset nodes notify guard condition executor callback
  for (const auto & pair : weak_nodes_to_guard_conditions_) {
    auto node = pair.first.lock();
    if (node) {
      auto node_gc = pair.second;
      unset_guard_condition_callback(node_gc);
    }
  }

  // Clear all lists
  weak_groups_associated_with_executor_to_nodes_.clear();
  weak_groups_to_nodes_associated_with_executor_.clear();
  weak_nodes_to_guard_conditions_.clear();
  weak_nodes_.clear();
}

void
EventsExecutorEntitiesCollector::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  // Check if the node already has an executor and if not, set this to true
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();

  if (has_executor.exchange(true)) {
    throw std::runtime_error("Node has already been added to an executor.");
  }

  // Get node callback groups, add them to weak_groups_to_nodes_associated_with_executor_
  for (const auto & weak_group : node_ptr->get_callback_groups()) {
    auto group_ptr = weak_group.lock();
    if (group_ptr != nullptr && !group_ptr->get_associated_with_executor_atomic().load() &&
      group_ptr->automatically_add_to_executor_with_node())
    {
      add_callback_group(group_ptr, node_ptr, weak_groups_to_nodes_associated_with_executor_);
    }
  }

  // Add node to weak_nodes_
  weak_nodes_.push_back(node_ptr);
}


void
EventsExecutorEntitiesCollector::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  add_callback_group(group_ptr, node_ptr, weak_groups_associated_with_executor_to_nodes_);
}

void
EventsExecutorEntitiesCollector::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  // If the callback_group already has an executor, throw error
  std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }

  bool is_new_node = !has_node(node_ptr, weak_groups_associated_with_executor_to_nodes_) &&
    !has_node(node_ptr, weak_groups_to_nodes_associated_with_executor_);

  if (is_new_node) {
    // Set an event callback for the node's notify guard condition, so if new entities are added
    // or removed to this node we will receive an event.
    set_guard_condition_callback(node_ptr->get_notify_guard_condition());

    // Store node's notify guard condition
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr node_weak_ptr(node_ptr);
    weak_nodes_to_guard_conditions_[node_weak_ptr] = node_ptr->get_notify_guard_condition();
  }

  // Add callback group to weak_groups_to_node
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;

  auto insert_info = weak_groups_to_nodes.insert(std::make_pair(weak_group_ptr, node_ptr));

  // Throw error if the group was already registered in the executor
  bool was_inserted = insert_info.second;
  if (!was_inserted) {
    throw std::runtime_error("Callback group was already added to executor.");
  }

  // For all entities in the callback group, set their event callback
  set_callback_group_entities_callbacks(group_ptr);
}

void
EventsExecutorEntitiesCollector::execute()
{
  // This function is called when the associated executor is notified that something changed.
  // We do not know if an entity has been added or removed so we have to rebuild everything.

  timers_manager_->clear();

  // If a registered node has a new callback group, register the group.
  add_callback_groups_from_nodes_associated_to_executor();

  // For all groups registered in the executor, set their event callbacks.
  set_entities_event_callbacks_from_map(weak_groups_associated_with_executor_to_nodes_);
  set_entities_event_callbacks_from_map(weak_groups_to_nodes_associated_with_executor_);
}

void
EventsExecutorEntitiesCollector::add_callback_groups_from_nodes_associated_to_executor()
{
  // Register new callback groups added to a node while already spinning
  for (const auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
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
            add_callback_group(
              shared_group_ptr,
              node,
              weak_groups_to_nodes_associated_with_executor_);
          }
        });
    }
  }
}

void
EventsExecutorEntitiesCollector::set_entities_event_callbacks_from_map(
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (!node || !group || !group->can_be_taken_from().load()) {
      continue;
    }
    set_callback_group_entities_callbacks(group);
  }
}

void
EventsExecutorEntitiesCollector::set_callback_group_entities_callbacks(
  rclcpp::CallbackGroup::SharedPtr group)
{
  // Timers are handled by the timers manager
  group->find_timer_ptrs_if(
    [this](const rclcpp::TimerBase::SharedPtr & timer) {
      if (timer) {
        timers_manager_->add_timer(timer);
        timer->set_on_destruction_callback(
          std::bind(&TimersManager::remove_timer_raw, timers_manager_, std::placeholders::_1));
      }
      return false;
    });

  // Set callbacks for all other entity types
  group->find_subscription_ptrs_if(
    [this](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
      if (subscription) {
        subscription->set_events_executor_callback(
          associated_executor_,
          &EventsExecutor::push_event);
        subscription->set_on_destruction_callback(
          std::bind(
            &EventsExecutor::remove_entity<rclcpp::SubscriptionBase>,
            associated_executor_,
            std::placeholders::_1));
      }
      return false;
    });
  group->find_service_ptrs_if(
    [this](const rclcpp::ServiceBase::SharedPtr & service) {
      if (service) {
        service->set_events_executor_callback(
          associated_executor_,
          &EventsExecutor::push_event);
        service->set_on_destruction_callback(
          std::bind(
            &EventsExecutor::remove_entity<rclcpp::ServiceBase>,
            associated_executor_,
            std::placeholders::_1));
      }
      return false;
    });
  group->find_client_ptrs_if(
    [this](const rclcpp::ClientBase::SharedPtr & client) {
      if (client) {
        client->set_events_executor_callback(
          associated_executor_,
          &EventsExecutor::push_event);
        client->set_on_destruction_callback(
          std::bind(
            &EventsExecutor::remove_entity<rclcpp::ClientBase>,
            associated_executor_,
            std::placeholders::_1));
      }
      return false;
    });
  group->find_waitable_ptrs_if(
    [this](const rclcpp::Waitable::SharedPtr & waitable) {
      if (waitable) {
        waitable->set_events_executor_callback(
          associated_executor_,
          &EventsExecutor::push_event);
        waitable->set_on_destruction_callback(
          std::bind(
            &EventsExecutor::remove_entity<rclcpp::Waitable>,
            associated_executor_,
            std::placeholders::_1));
      }
      return false;
    });
}

void
EventsExecutorEntitiesCollector::unset_callback_group_entities_callbacks(
  rclcpp::CallbackGroup::SharedPtr group)
{
  // Timers are handled by the timers manager
  group->find_timer_ptrs_if(
    [this](const rclcpp::TimerBase::SharedPtr & timer) {
      if (timer) {
        timers_manager_->remove_timer(timer);
        timer->set_on_destruction_callback(nullptr);
      }
      return false;
    });

  // Unset callbacks for all other entity types
  group->find_subscription_ptrs_if(
    [this](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
      if (subscription) {
        subscription->set_events_executor_callback(nullptr, nullptr);
        subscription->set_on_destruction_callback(nullptr);
      }
      return false;
    });
  group->find_service_ptrs_if(
    [this](const rclcpp::ServiceBase::SharedPtr & service) {
      if (service) {
        service->set_events_executor_callback(nullptr, nullptr);
        service->set_on_destruction_callback(nullptr);
      }
      return false;
    });
  group->find_client_ptrs_if(
    [this](const rclcpp::ClientBase::SharedPtr & client) {
      if (client) {
        client->set_events_executor_callback(nullptr, nullptr);
        client->set_on_destruction_callback(nullptr);
      }
      return false;
    });
  group->find_waitable_ptrs_if(
    [this](const rclcpp::Waitable::SharedPtr & waitable) {
      if (waitable) {
        waitable->set_events_executor_callback(nullptr, nullptr);
        waitable->set_on_destruction_callback(nullptr);
      }
      return false;
    });
}

void
EventsExecutorEntitiesCollector::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr)
{
  this->remove_callback_group_from_map(
    group_ptr,
    weak_groups_associated_with_executor_to_nodes_);
}

void
EventsExecutorEntitiesCollector::remove_callback_group_from_map(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr;
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;

  // Look for the group to remove in the map
  auto iter = weak_groups_to_nodes.find(weak_group_ptr);
  if (iter != weak_groups_to_nodes.end()) {
    // Group found, get its associated node.
    node_ptr = iter->second.lock();
    if (node_ptr == nullptr) {
      throw std::runtime_error("Node must not be deleted before its callback group(s).");
    }
    // Remove group from map
    weak_groups_to_nodes.erase(iter);

    // For all the entities in the group, unset their callbacks
    unset_callback_group_entities_callbacks(group_ptr);

    // Check if this node still has other callback groups associated with the executor
    bool node_has_associated_callback_groups =
      !has_node(node_ptr, weak_groups_associated_with_executor_to_nodes_) &&
      !has_node(node_ptr, weak_groups_to_nodes_associated_with_executor_);

    if (!node_has_associated_callback_groups) {
      // Node doesn't have more callback groups associated to the executor.
      // Unset the event callback for the node's notify guard condition, to stop
      // receiving events if entities are added or removed to this node.
      unset_guard_condition_callback(node_ptr->get_notify_guard_condition());

      // Remove guard condition from list
      rclcpp::node_interfaces::NodeBaseInterface::WeakPtr weak_node_ptr(node_ptr);
      weak_nodes_to_guard_conditions_.erase(weak_node_ptr);
    }
  } else {
    throw std::runtime_error("Callback group needs to be associated with executor.");
  }
}

void
EventsExecutorEntitiesCollector::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  if (!node_ptr->get_associated_with_executor_atomic().load()) {
    throw std::runtime_error("Node needs to be associated with this executor.");
    return;
  }
  // Check if this node is currently stored here
  auto node_it = weak_nodes_.begin();
  while (node_it != weak_nodes_.end()) {
    if (node_it->lock() == node_ptr) {
      weak_nodes_.erase(node_it);
      break;
    }
  }
  if (node_it == weak_nodes_.end()) {
    // The node is not stored here, so nothing to do
    throw std::runtime_error("Tried to remove node not stored in executor.");
    return;
  }

  // Find callback groups belonging to the node to remove
  std::vector<rclcpp::CallbackGroup::SharedPtr> found_group_ptrs;

  std::for_each(
    weak_groups_to_nodes_associated_with_executor_.begin(),
    weak_groups_to_nodes_associated_with_executor_.end(),
    [&found_group_ptrs, node_ptr](std::pair<rclcpp::CallbackGroup::WeakPtr,
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> key_value_pair) {
      auto & weak_node_ptr = key_value_pair.second;
      auto shared_node_ptr = weak_node_ptr.lock();
      auto group_ptr = key_value_pair.first.lock();
      if (shared_node_ptr == node_ptr) {
        found_group_ptrs.push_back(group_ptr);
      }
    });

  // Remove those callback groups
  std::for_each(
    found_group_ptrs.begin(), found_group_ptrs.end(), [this]
      (rclcpp::CallbackGroup::SharedPtr group_ptr) {
      this->remove_callback_group_from_map(
        group_ptr,
        weak_groups_to_nodes_associated_with_executor_);
    });

  // Set that the node does not have an executor anymore
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
}

// Returns true if the map has the node_ptr
bool
EventsExecutorEntitiesCollector::has_node(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) const
{
  return std::find_if(
    weak_groups_to_nodes.begin(),
    weak_groups_to_nodes.end(),
    [&](const WeakCallbackGroupsToNodesMap::value_type & other) -> bool {
      auto other_ptr = other.second.lock();
      return other_ptr == node_ptr;
    }) != weak_groups_to_nodes.end();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsExecutorEntitiesCollector::get_all_callback_groups()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  for (const auto & group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(group_node_ptr.first);
  }
  for (const auto & group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsExecutorEntitiesCollector::get_manually_added_callback_groups()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  for (const auto & group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsExecutorEntitiesCollector::get_automatically_added_callback_groups_from_nodes()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  for (const auto & group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

void
EventsExecutorEntitiesCollector::set_guard_condition_callback(
  const rcl_guard_condition_t * guard_condition)
{
  rcl_ret_t ret = rcl_guard_condition_set_events_executor_callback(
    associated_executor_,
    &EventsExecutor::push_event,
    this,
    guard_condition,
    false /* Discard previous events */);

  if (ret != RCL_RET_OK) {
    throw std::runtime_error("Couldn't set guard condition event callback");
  }
}

void
EventsExecutorEntitiesCollector::unset_guard_condition_callback(
  const rcl_guard_condition_t * guard_condition)
{
  rcl_ret_t ret = rcl_guard_condition_set_events_executor_callback(
    nullptr,
    nullptr,
    nullptr,
    guard_condition,
    false /* Discard previous events */);

  if (ret != RCL_RET_OK) {
    throw std::runtime_error("Couldn't unset guard condition event callback");
  }
}
