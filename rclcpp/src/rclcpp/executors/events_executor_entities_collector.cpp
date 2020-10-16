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
  std::cout<<"entities collector destructor"<<std::endl;

  // Disassociate all nodes
  for (const auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      std::cout<<"handling with node in entities collector destructor"<<std::endl;
      std::atomic_bool & has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
      this->unset_entities_callbacks(node);
    } else {
      std::cout<<"skipping node in entities collector destructor"<<std::endl;
    }
  }

  // Make sure that the list is empty
  weak_nodes_.clear();
}

void
EventsExecutorEntitiesCollector::execute()
{
  // This function is called when the associated executor is notified that something changed.
  // We do not know if an entity has been added or remode so we have to rebuild everything.

  timers_manager_->clear_all();

  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    set_entities_callbacks(node);
  }
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

  weak_nodes_.push_back(node_ptr);

  set_entities_callbacks(node_ptr);
}

void
EventsExecutorEntitiesCollector::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  // Check if this node is currently stored here
  auto node_it = weak_nodes_.begin();
  while (node_it != weak_nodes_.end()) {
    if (node_it->lock() == node_ptr) {
      break;
    }
  }
  if (node_it == weak_nodes_.end()) {
    // The node is not stored here, so nothing to do
    return;
  }

  // Set that the node does not have an executor anymore
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);

  weak_nodes_.erase(node_it);

  unset_entities_callbacks(node_ptr);
}

void
EventsExecutorEntitiesCollector::set_entities_callbacks(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
  // Set event callbacks for all entities in this node
  // by searching them in all callback groups
  for (auto & weak_group : node->get_callback_groups()) {
    auto group = weak_group.lock();
    if (!group || !group->can_be_taken_from().load()) {
      continue;
    }

    // Timers are handled by the timers manager
    group->find_timer_ptrs_if(
      [this](const rclcpp::TimerBase::SharedPtr & timer) {
        if (timer) {
          timers_manager_->add_timer(timer);
          timer->set_on_destruction_callback(std::bind(&TimersManager::remove_timer_raw, timers_manager_, std::placeholders::_1));
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
          std::cout<<"Setting destruction callback on "<< subscription.get()<< " with count "<< subscription.use_count()<<std::endl;
          subscription->set_on_destruction_callback(std::bind(&EventsExecutor::remove_entity<rclcpp::SubscriptionBase>, associated_executor_, std::placeholders::_1));
        }
        return false;
      });
    group->find_service_ptrs_if(
      [this](const rclcpp::ServiceBase::SharedPtr & service) {
        if (service) {
          service->set_events_executor_callback(
            associated_executor_,
            &EventsExecutor::push_event);
          service->set_on_destruction_callback(std::bind(&EventsExecutor::remove_entity<rclcpp::ServiceBase>, associated_executor_, std::placeholders::_1));
        }
        return false;
      });
    group->find_client_ptrs_if(
      [this](const rclcpp::ClientBase::SharedPtr & client) {
        if (client) {
          client->set_events_executor_callback(
            associated_executor_,
            &EventsExecutor::push_event);
          client->set_on_destruction_callback(std::bind(&EventsExecutor::remove_entity<rclcpp::ClientBase>, associated_executor_, std::placeholders::_1));
        }
        return false;
      });
    group->find_waitable_ptrs_if(
      [this](const rclcpp::Waitable::SharedPtr & waitable) {
        if (waitable) {
          waitable->set_events_executor_callback(
            associated_executor_,
            &EventsExecutor::push_event);
          waitable->set_on_destruction_callback(std::bind(&EventsExecutor::remove_entity<rclcpp::Waitable>, associated_executor_, std::placeholders::_1));
        }
        return false;
      });
  }

  // Set an event callback for the node's notify guard condition, so if new entities are added
  // or removed to this node we will receive an event.
  rcl_ret_t ret = rcl_guard_condition_set_events_executor_callback(
    associated_executor_,
    &EventsExecutor::push_event,
    this,
    node->get_notify_guard_condition(),
    false /* Discard previous events */);

  if (ret != RCL_RET_OK) {
    throw std::runtime_error("Couldn't set node guard condition callback");
  }
}

void
EventsExecutorEntitiesCollector::unset_entities_callbacks(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
  // Unset event callbacks for all entities in this node
  // by searching them in all callback groups
  for (auto & weak_group : node->get_callback_groups()) {
    auto group = weak_group.lock();
    if (!group || !group->can_be_taken_from().load()) {
      continue;
    }

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
          std::cout<<"Removing destruction callback on "<< subscription.get()<<std::endl;
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

  // Unset the event callback for the node's notify guard condition, to stop receiving events
  // if entities are added or removed to this node.
  rcl_ret_t ret = rcl_guard_condition_set_events_executor_callback(
    nullptr, nullptr, nullptr,
    node->get_notify_guard_condition(),
    false);

  if (ret != RCL_RET_OK) {
    throw std::runtime_error("Couldn't set node guard condition callback");
  }
}
