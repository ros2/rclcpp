// Copyright 2023 iRobot Corporation.
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

#include "rclcpp/experimental/executors/events_executor/events_executor_entities_collector.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

using rclcpp::experimental::executors::ExecutorEvent;
using rclcpp::experimental::executors::ExecutorEventType;
using rclcpp::experimental::executors::EventsExecutorEntitiesCollector;

EventsExecutorEntitiesCollector::EventsExecutorEntitiesCollector(
  rclcpp::experimental::executors::EventsExecutor * executor)
{
  if (executor == nullptr) {
    throw std::runtime_error("Received nullptr executor in EventsExecutorEntitiesCollector.");
  }

  associated_executor_ = executor;
  timers_manager_ = associated_executor_->timers_manager_;
}

void
EventsExecutorEntitiesCollector::init()
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
  // Add the EventsExecutorEntitiesCollector shared_ptr to waitables map
  weak_waitables_map_.emplace(this, this->shared_from_this());
}

EventsExecutorEntitiesCollector::~EventsExecutorEntitiesCollector()
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  // Disassociate all callback groups and thus nodes.
  for (const auto & pair : weak_groups_associated_with_executor_to_nodes_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
      callback_group_removed_impl(group);
    }
  }
  for (const auto & pair : weak_groups_to_nodes_associated_with_executor_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
      callback_group_removed_impl(group);
    }
  }
  // Disassociate all nodes
  for (const auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      std::atomic_bool & has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
      node_removed_impl(node);
    }
  }

  // Unset callback group notify guard condition executor callback
  for (const auto & pair : weak_groups_to_guard_conditions_) {
    auto group = pair.first.lock();
    if (group) {
      auto & group_gc = pair.second;
      unset_guard_condition_callback(group_gc);
    }
  }

  weak_clients_map_.clear();
  weak_services_map_.clear();
  weak_waitables_map_.clear();
  weak_subscriptions_map_.clear();
  weak_nodes_to_guard_conditions_.clear();
  weak_groups_to_guard_conditions_.clear();

  weak_groups_associated_with_executor_to_nodes_.clear();
  weak_groups_to_nodes_associated_with_executor_.clear();
  weak_nodes_.clear();
}

void
EventsExecutorEntitiesCollector::execute(std::shared_ptr<void> & data)
{
  // This function is called when the associated executor is notified that something changed.
  // We do not know if an entity has been added or removed so we have to rebuild everything.
  (void)data;

  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
  timers_manager_->clear();

  // If a registered node has a new callback group, register the group.
  add_callback_groups_from_nodes_associated_to_executor();

  // For all groups registered in the executor, set their event callbacks.
  set_entities_event_callbacks_from_map(weak_groups_associated_with_executor_to_nodes_);
  set_entities_event_callbacks_from_map(weak_groups_to_nodes_associated_with_executor_);
}

std::shared_ptr<void>
EventsExecutorEntitiesCollector::take_data()
{
  return nullptr;
}

std::shared_ptr<void>
EventsExecutorEntitiesCollector::take_data_by_entity_id(size_t id)
{
  (void)id;
  return nullptr;
}

void
EventsExecutorEntitiesCollector::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  (void)wait_set;
}

bool
EventsExecutorEntitiesCollector::is_ready(rcl_wait_set_t * p_wait_set)
{
  (void)p_wait_set;
  return false;
}

bool
EventsExecutorEntitiesCollector::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  bool is_new_node = false;
  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Node has already been added to an executor.");
  }
  node_ptr->for_each_callback_group(
    [this, node_ptr, &is_new_node](rclcpp::CallbackGroup::SharedPtr group_ptr)
    {
      if (
        !group_ptr->get_associated_with_executor_atomic().load() &&
        group_ptr->automatically_add_to_executor_with_node())
      {
        is_new_node = (add_callback_group(
          group_ptr,
          node_ptr,
          weak_groups_to_nodes_associated_with_executor_) ||
        is_new_node);
      }
    });
  weak_nodes_.push_back(node_ptr);
  return is_new_node;
}

bool
EventsExecutorEntitiesCollector::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  // If the callback_group already has an executor
  std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }
  bool is_new_node = !has_node(node_ptr, weak_groups_associated_with_executor_to_nodes_) &&
    !has_node(node_ptr, weak_groups_to_nodes_associated_with_executor_);
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto insert_info = weak_groups_to_nodes.insert(
    std::make_pair(weak_group_ptr, node_ptr));
  bool was_inserted = insert_info.second;
  if (!was_inserted) {
    throw std::runtime_error("Callback group was already added to executor.");
  }

  if (is_new_node) {
    node_added_impl(node_ptr);
  }

  if (node_ptr->get_context()->is_valid()) {
    auto callback_group_guard_condition =
      group_ptr->get_notify_guard_condition(node_ptr->get_context());

    rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
    weak_groups_to_guard_conditions_[weak_group_ptr] = callback_group_guard_condition.get();
  }

  callback_group_added_impl(group_ptr);

  return is_new_node;
}

bool
EventsExecutorEntitiesCollector::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  return add_callback_group(group_ptr, node_ptr, weak_groups_associated_with_executor_to_nodes_);
}

bool
EventsExecutorEntitiesCollector::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr)
{
  return this->remove_callback_group_from_map(
    group_ptr,
    weak_groups_associated_with_executor_to_nodes_);
}

bool
EventsExecutorEntitiesCollector::remove_callback_group_from_map(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr;
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto iter = weak_groups_to_nodes.find(weak_group_ptr);
  if (iter != weak_groups_to_nodes.end()) {
    node_ptr = iter->second.lock();
    if (node_ptr == nullptr) {
      throw std::runtime_error("Node must not be deleted before its callback group(s).");
    }
    weak_groups_to_nodes.erase(iter);
    callback_group_removed_impl(group_ptr);
  } else {
    throw std::runtime_error("Callback group needs to be associated with executor.");
  }
  // If the node was matched and removed, interrupt waiting.
  bool node_removed = false;
  if (!has_node(node_ptr, weak_groups_associated_with_executor_to_nodes_) &&
    !has_node(node_ptr, weak_groups_to_nodes_associated_with_executor_))
  {
    node_removed_impl(node_ptr);
    node_removed = true;
  }

  weak_groups_to_guard_conditions_.erase(weak_group_ptr);

  return node_removed;
}

bool
EventsExecutorEntitiesCollector::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  if (!node_ptr->get_associated_with_executor_atomic().load()) {
    return false;
  }
  bool node_found = false;
  auto node_it = weak_nodes_.begin();
  while (node_it != weak_nodes_.end()) {
    bool matched = (node_it->lock() == node_ptr);
    if (matched) {
      weak_nodes_.erase(node_it);
      node_found = true;
      break;
    }
    ++node_it;
  }
  if (!node_found) {
    return false;
  }
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
  std::for_each(
    found_group_ptrs.begin(), found_group_ptrs.end(), [this]
      (rclcpp::CallbackGroup::SharedPtr group_ptr) {
      this->remove_callback_group_from_map(
        group_ptr,
        weak_groups_to_nodes_associated_with_executor_);
    });
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
  return true;
}

// Returns true iff the weak_groups_to_nodes map has node_ptr as the value in any of its entry.
bool
EventsExecutorEntitiesCollector::has_node(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  const rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &
  weak_groups_to_nodes) const
{
  return std::find_if(
    weak_groups_to_nodes.begin(),
    weak_groups_to_nodes.end(),
    [&](const WeakCallbackGroupsToNodesMap::value_type & other) -> bool {
      auto other_ptr = other.second.lock();
      return other_ptr == node_ptr;
    }) != weak_groups_to_nodes.end();
}

void
EventsExecutorEntitiesCollector::add_callback_groups_from_nodes_associated_to_executor()
{
  for (const auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      node->for_each_callback_group(
        [this, node](rclcpp::CallbackGroup::SharedPtr shared_group_ptr)
        {
          if (shared_group_ptr->automatically_add_to_executor_with_node() &&
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
EventsExecutorEntitiesCollector::callback_group_added_impl(
  rclcpp::CallbackGroup::SharedPtr group)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group;

  auto iter = weak_groups_to_guard_conditions_.find(weak_group_ptr);
  if (iter != weak_groups_to_guard_conditions_.end()) {
    // Set an event callback for the group's notify guard condition, so if new entities are added
    // or removed to this node we will receive an event.
    set_guard_condition_callback(iter->second);
  }
  // For all entities in the callback group, set their event callback
  set_callback_group_entities_callbacks(group);
}

void
EventsExecutorEntitiesCollector::node_added_impl(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto notify_guard_condition = &(node->get_notify_guard_condition());
  // Set an event callback for the node's notify guard condition, so if new entities are added
  // or removed to this node we will receive an event.
  set_guard_condition_callback(notify_guard_condition);

  // Store node's notify guard condition
  weak_nodes_to_guard_conditions_[node] = notify_guard_condition;
}

void
EventsExecutorEntitiesCollector::callback_group_removed_impl(
  rclcpp::CallbackGroup::SharedPtr group)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
  // For all the entities in the group, unset their callbacks
  unset_callback_group_entities_callbacks(group);
}

void
EventsExecutorEntitiesCollector::node_removed_impl(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
  // Node doesn't have more callback groups associated to the executor.
  // Unset the event callback for the node's notify guard condition, to stop
  // receiving events if entities are added or removed to this node.
  unset_guard_condition_callback(&(node->get_notify_guard_condition()));

  // Remove guard condition from list
  rclcpp::node_interfaces::NodeBaseInterface::WeakPtr weak_node_ptr(node);
  weak_nodes_to_guard_conditions_.erase(weak_node_ptr);
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
      }
      return false;
    });

  // Set callbacks for all other entity types
  group->find_subscription_ptrs_if(
    [this](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
      if (subscription) {
        weak_subscriptions_map_.emplace(subscription.get(), subscription);

        subscription->set_on_new_message_callback(
          create_entity_callback(subscription.get(), ExecutorEventType::SUBSCRIPTION_EVENT));
      }
      return false;
    });
  group->find_service_ptrs_if(
    [this](const rclcpp::ServiceBase::SharedPtr & service) {
      if (service) {
        weak_services_map_.emplace(service.get(), service);

        service->set_on_new_request_callback(
          create_entity_callback(service.get(), ExecutorEventType::SERVICE_EVENT));
      }
      return false;
    });
  group->find_client_ptrs_if(
    [this](const rclcpp::ClientBase::SharedPtr & client) {
      if (client) {
        weak_clients_map_.emplace(client.get(), client);

        client->set_on_new_response_callback(
          create_entity_callback(client.get(), ExecutorEventType::CLIENT_EVENT));
      }
      return false;
    });
  group->find_waitable_ptrs_if(
    [this](const rclcpp::Waitable::SharedPtr & waitable) {
      if (waitable) {
        weak_waitables_map_.emplace(waitable.get(), waitable);

        waitable->set_on_ready_callback(
          create_waitable_callback(waitable.get()));
      }
      return false;
    });
}

void
EventsExecutorEntitiesCollector::unset_callback_group_entities_callbacks(
  rclcpp::CallbackGroup::SharedPtr group)
{
  auto iter = weak_groups_to_guard_conditions_.find(group);

  if (iter != weak_groups_to_guard_conditions_.end()) {
    unset_guard_condition_callback(iter->second);
  }

  // Timers are handled by the timers manager
  group->find_timer_ptrs_if(
    [this](const rclcpp::TimerBase::SharedPtr & timer) {
      if (timer) {
        timers_manager_->remove_timer(timer);
      }
      return false;
    });

  // Unset callbacks for all other entity types
  group->find_subscription_ptrs_if(
    [this](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
      if (subscription) {
        subscription->clear_on_new_message_callback();
        weak_subscriptions_map_.erase(subscription.get());
      }
      return false;
    });
  group->find_service_ptrs_if(
    [this](const rclcpp::ServiceBase::SharedPtr & service) {
      if (service) {
        service->clear_on_new_request_callback();
        weak_services_map_.erase(service.get());
      }
      return false;
    });
  group->find_client_ptrs_if(
    [this](const rclcpp::ClientBase::SharedPtr & client) {
      if (client) {
        client->clear_on_new_response_callback();
        weak_clients_map_.erase(client.get());
      }
      return false;
    });
  group->find_waitable_ptrs_if(
    [this](const rclcpp::Waitable::SharedPtr & waitable) {
      if (waitable) {
        waitable->clear_on_ready_callback();
        weak_waitables_map_.erase(waitable.get());
      }
      return false;
    });
}

void
EventsExecutorEntitiesCollector::set_guard_condition_callback(
  rclcpp::GuardCondition * guard_condition)
{
  auto gc_callback = [this](size_t num_events) {
      // Override num events (we don't care more than a single event)
      num_events = 1;
      int gc_id = -1;
      ExecutorEvent event = {this, gc_id, ExecutorEventType::WAITABLE_EVENT, num_events};
      associated_executor_->events_queue_->enqueue(event);
    };

  guard_condition->set_on_trigger_callback(gc_callback);
}

void
EventsExecutorEntitiesCollector::unset_guard_condition_callback(
  rclcpp::GuardCondition * guard_condition)
{
  guard_condition->set_on_trigger_callback(nullptr);
}

rclcpp::SubscriptionBase::SharedPtr
EventsExecutorEntitiesCollector::get_subscription(const void * subscription_id)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto it = weak_subscriptions_map_.find(subscription_id);

  if (it != weak_subscriptions_map_.end()) {
    auto subscription_weak_ptr = it->second;
    auto subscription_shared_ptr = subscription_weak_ptr.lock();

    if (subscription_shared_ptr) {
      return subscription_shared_ptr;
    }

    // The subscription expired, remove from map
    weak_subscriptions_map_.erase(it);
  }
  return nullptr;
}

rclcpp::ClientBase::SharedPtr
EventsExecutorEntitiesCollector::get_client(const void * client_id)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto it = weak_clients_map_.find(client_id);

  if (it != weak_clients_map_.end()) {
    auto client_weak_ptr = it->second;
    auto client_shared_ptr = client_weak_ptr.lock();

    if (client_shared_ptr) {
      return client_shared_ptr;
    }

    // The client expired, remove from map
    weak_clients_map_.erase(it);
  }
  return nullptr;
}

rclcpp::ServiceBase::SharedPtr
EventsExecutorEntitiesCollector::get_service(const void * service_id)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto it = weak_services_map_.find(service_id);

  if (it != weak_services_map_.end()) {
    auto service_weak_ptr = it->second;
    auto service_shared_ptr = service_weak_ptr.lock();

    if (service_shared_ptr) {
      return service_shared_ptr;
    }

    // The service expired, remove from map
    weak_services_map_.erase(it);
  }
  return nullptr;
}

rclcpp::Waitable::SharedPtr
EventsExecutorEntitiesCollector::get_waitable(const void * waitable_id)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  auto it = weak_waitables_map_.find(waitable_id);

  if (it != weak_waitables_map_.end()) {
    auto waitable_weak_ptr = it->second;
    auto waitable_shared_ptr = waitable_weak_ptr.lock();

    if (waitable_shared_ptr) {
      return waitable_shared_ptr;
    }

    // The waitable expired, remove from map
    weak_waitables_map_.erase(it);
  }
  return nullptr;
}

void
EventsExecutorEntitiesCollector::add_waitable(rclcpp::Waitable::SharedPtr waitable)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  weak_waitables_map_.emplace(waitable.get(), waitable);

  waitable->set_on_ready_callback(
    create_waitable_callback(waitable.get()));
}

std::function<void(size_t)>
EventsExecutorEntitiesCollector::create_entity_callback(
  void * exec_entity_id, ExecutorEventType event_type)
{
  std::function<void(size_t)>
  callback = [this, exec_entity_id, event_type](size_t num_events) {
      ExecutorEvent event = {exec_entity_id, -1, event_type, num_events};
      associated_executor_->events_queue_->enqueue(event);
    };
  return callback;
}

std::function<void(size_t, int)>
EventsExecutorEntitiesCollector::create_waitable_callback(void * exec_entity_id)
{
  std::function<void(size_t, int)>
  callback = [this, exec_entity_id](size_t num_events, int gen_entity_id) {
      ExecutorEvent event =
      {exec_entity_id, gen_entity_id, ExecutorEventType::WAITABLE_EVENT, num_events};
      associated_executor_->events_queue_->enqueue(event);
    };
  return callback;
}
