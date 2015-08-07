// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_RCLCPP_EXECUTOR_HPP_
#define RCLCPP_RCLCPP_EXECUTOR_HPP_

#include <iostream>

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <list>
#include <memory>
#include <vector>

#include <rclcpp/any_executable.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/memory_strategy.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

namespace rclcpp
{
namespace executor
{

class Executor
{
  friend class memory_strategy::MemoryStrategy;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(Executor);

  explicit Executor(memory_strategy::MemoryStrategy::SharedPtr ms =
    memory_strategy::create_default_strategy())
  : interrupt_guard_condition_(rmw_create_guard_condition()),
    memory_strategy_(ms)
  {
  }

  virtual ~Executor()
  {
    if (interrupt_guard_condition_ != nullptr) {
      rmw_ret_t status = rmw_destroy_guard_condition(interrupt_guard_condition_);
      if (status != RMW_RET_OK) {
        fprintf(stderr,
          "[rclcpp::error] failed to destroy guard condition: %s\n", rmw_get_error_string_safe());
      }
    }
  }

  virtual void spin() = 0;

  virtual void
  add_node(rclcpp::node::Node::SharedPtr & node_ptr, bool notify = true)
  {
    // Check to ensure node not already added
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (node == node_ptr) {
        // TODO: Use a different error here?
        throw std::runtime_error("Cannot add node to executor, node already added.");
      }
    }
    weak_nodes_.push_back(node_ptr);
    if (notify) {
      // Interrupt waiting to handle new node
      rmw_ret_t status = rmw_trigger_guard_condition(interrupt_guard_condition_);
      if (status != RMW_RET_OK) {
        throw std::runtime_error(rmw_get_error_string_safe());
      }
    }
  }

  virtual void
  remove_node(rclcpp::node::Node::SharedPtr & node_ptr, bool notify = true)
  {
    bool node_removed = false;
    weak_nodes_.erase(
      std::remove_if(
        weak_nodes_.begin(), weak_nodes_.end(),
        [&](std::weak_ptr<rclcpp::node::Node> & i)
        {
          bool matched = (i.lock() == node_ptr);
          node_removed |= matched;
          return matched;
        }));
    if (notify) {
      // If the node was matched and removed, interrupt waiting
      if (node_removed) {
        rmw_ret_t status = rmw_trigger_guard_condition(interrupt_guard_condition_);
        if (status != RMW_RET_OK) {
          throw std::runtime_error(rmw_get_error_string_safe());
        }
      }
    }
  }

  template<typename T = std::milli>
  void spin_node_once(rclcpp::node::Node::SharedPtr & node,
    std::chrono::duration<int64_t, T> timeout = std::chrono::duration<int64_t, T>(-1))
  {
    this->add_node(node, false);
    // non-blocking = true
    auto any_exec = get_next_executable(timeout);
    if (any_exec) {
      execute_any_executable(any_exec);
    }
    this->remove_node(node, false);
  }

  void spin_node_some(rclcpp::node::Node::SharedPtr & node)
  {
    this->add_node(node, false);
    // non-blocking = true
    while (AnyExecutable::SharedPtr any_exec =
      get_next_executable(std::chrono::milliseconds::zero()))
    {
      execute_any_executable(any_exec);
    }
    this->remove_node(node, false);
  }

  // Support dynamic switching of memory strategy
  void
  set_memory_strategy(memory_strategy::MemoryStrategy::SharedPtr memory_strategy)
  {
    if (memory_strategy == nullptr) {
      throw std::runtime_error("Received NULL memory strategy in executor.");
    }
    memory_strategy_ = memory_strategy;
  }

protected:
  void
  execute_any_executable(AnyExecutable::SharedPtr & any_exec)
  {
    if (!any_exec) {
      return;
    }
    if (any_exec->timer) {
      execute_timer(any_exec->timer);
    }
    if (any_exec->subscription) {
      execute_subscription(any_exec->subscription);
    }
    if (any_exec->service) {
      execute_service(any_exec->service);
    }
    if (any_exec->client) {
      execute_client(any_exec->client);
    }
    // Reset the callback_group, regardless of type
    any_exec->callback_group->can_be_taken_from_.store(true);
    // Wake the wait, because it may need to be recalculated or work that
    // was previously blocked is now available.
    rmw_ret_t status = rmw_trigger_guard_condition(interrupt_guard_condition_);
    if (status != RMW_RET_OK) {
      throw std::runtime_error(rmw_get_error_string_safe());
    }
  }

  static void
  execute_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr & subscription)
  {
    std::shared_ptr<void> message = subscription->create_message();
    bool taken = false;
    rmw_ret_t status = rmw_take(subscription->subscription_handle_, message.get(), &taken);
    if (status == RMW_RET_OK) {
      if (taken) {
        subscription->handle_message(message);
      }
    } else {
      fprintf(stderr,
        "[rclcpp::error] take failed for subscription on topic '%s': %s\n",
        subscription->get_topic_name().c_str(), rmw_get_error_string_safe());
    }
  }

  static void
  execute_timer(
    rclcpp::timer::TimerBase::SharedPtr & timer)
  {
    timer->callback_();
  }

  static void
  execute_service(
    rclcpp::service::ServiceBase::SharedPtr & service)
  {
    std::shared_ptr<void> request_header = service->create_request_header();
    std::shared_ptr<void> request = service->create_request();
    bool taken = false;
    rmw_ret_t status = rmw_take_request(
      service->service_handle_,
      request_header.get(),
      request.get(),
      &taken);
    if (status == RMW_RET_OK) {
      if (taken) {
        service->handle_request(request_header, request);
      }
    } else {
      fprintf(stderr,
        "[rclcpp::error] take request failed for server of service '%s': %s\n",
        service->get_service_name().c_str(), rmw_get_error_string_safe());
    }
  }

  static void
  execute_client(
    rclcpp::client::ClientBase::SharedPtr & client)
  {
    std::shared_ptr<void> request_header = client->create_request_header();
    std::shared_ptr<void> response = client->create_response();
    bool taken = false;
    rmw_ret_t status = rmw_take_response(
      client->client_handle_,
      request_header.get(),
      response.get(),
      &taken);
    if (status == RMW_RET_OK) {
      if (taken) {
        client->handle_response(request_header, response);
      }
    } else {
      fprintf(stderr,
        "[rclcpp::error] take response failed for client of service '%s': %s\n",
        client->get_service_name().c_str(), rmw_get_error_string_safe());
    }
  }

/*** Populate class storage from stored weak node pointers and wait. ***/

  template<typename T = std::milli>
  void
  wait_for_work(std::chrono::duration<int64_t, T> timeout = std::chrono::duration<int64_t, T>(-1))
  {
    // Collect the subscriptions and timers to be waited on
    bool has_invalid_weak_nodes = false;
    std::vector<rclcpp::subscription::SubscriptionBase::SharedPtr> subs;
    std::vector<rclcpp::service::ServiceBase::SharedPtr> services;
    std::vector<rclcpp::client::ClientBase::SharedPtr> clients;
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        has_invalid_weak_nodes = false;
        continue;
      }
      for (auto & weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        if (!group || !group->can_be_taken_from_.load()) {
          continue;
        }
        for (auto & weak_subscription : group->subscription_ptrs_) {
          auto subscription = weak_subscription.lock();
          if (subscription) {
            subs.push_back(subscription);
          }
        }
        for (auto & service : group->service_ptrs_) {
          services.push_back(service);
        }
        for (auto & client : group->client_ptrs_) {
          clients.push_back(client);
        }
      }
    }
    // Clean up any invalid nodes, if they were detected
    if (has_invalid_weak_nodes) {
      weak_nodes_.erase(
        remove_if(weak_nodes_.begin(), weak_nodes_.end(),
        [](std::weak_ptr<rclcpp::node::Node> i)
          {
            return i.expired();
          }));
    }
    // Use the number of subscriptions to allocate memory in the handles
    size_t number_of_subscriptions = subs.size();
    rmw_subscriptions_t subscriber_handles;
    subscriber_handles.subscriber_count = number_of_subscriptions;
    // TODO(wjwwood): Avoid redundant malloc's
    subscriber_handles.subscribers =
      memory_strategy_->borrow_handles(HandleType::subscriber_handle, number_of_subscriptions);
    if (subscriber_handles.subscribers == NULL) {
      // TODO(wjwwood): Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for subscriber pointers.");
    }
    // Then fill the SubscriberHandles with ready subscriptions
    size_t subscriber_handle_index = 0;
    for (auto & subscription : subs) {
      subscriber_handles.subscribers[subscriber_handle_index] = \
        subscription->subscription_handle_->data;
      subscriber_handle_index += 1;
    }

    // Use the number of services to allocate memory in the handles
    size_t number_of_services = services.size();
    rmw_services_t service_handles;
    service_handles.service_count = number_of_services;
    service_handles.services =
      memory_strategy_->borrow_handles(HandleType::service_handle, number_of_services);
    if (service_handles.services == NULL) {
      // TODO(esteve): Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for service pointers.");
    }
    // Then fill the ServiceHandles with ready services
    size_t service_handle_index = 0;
    for (auto & service : services) {
      service_handles.services[service_handle_index] = \
        service->service_handle_->data;
      service_handle_index += 1;
    }

    // Use the number of clients to allocate memory in the handles
    size_t number_of_clients = clients.size();
    rmw_clients_t client_handles;
    client_handles.client_count = number_of_clients;
    client_handles.clients =
      memory_strategy_->borrow_handles(HandleType::client_handle, number_of_clients);
    if (client_handles.clients == NULL) {
      // TODO: Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for client pointers.");
    }
    // Then fill the ServiceHandles with ready clients
    size_t client_handle_index = 0;
    for (auto & client : clients) {
      client_handles.clients[client_handle_index] = \
        client->client_handle_->data;
      client_handle_index += 1;
    }

    // The number of guard conditions is fixed at 2: 1 for the ctrl-c guard cond,
    // and one for the executor's guard cond (interrupt_guard_condition_)
    size_t number_of_guard_conds = 2;
    rmw_guard_conditions_t guard_condition_handles;
    guard_condition_handles.guard_condition_count = number_of_guard_conds;
    guard_condition_handles.guard_conditions =
      memory_strategy_->borrow_handles(HandleType::guard_condition_handle, number_of_guard_conds);
    if (guard_condition_handles.guard_conditions == NULL) {
      // TODO(wjwwood): Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for guard condition pointers.");
    }
    // Put the global ctrl-c guard condition in
    assert(guard_condition_handles.guard_condition_count > 1);
    guard_condition_handles.guard_conditions[0] = \
      rclcpp::utilities::get_global_sigint_guard_condition()->data;
    // Put the executor's guard condition in
    guard_condition_handles.guard_conditions[1] = \
      interrupt_guard_condition_->data;

    rmw_time_t * wait_timeout = NULL;
    rmw_time_t rmw_timeout;

    auto next_timer_duration = get_earliest_timer();
    // If the next timer timeout must preempt the requested timeout
    // or if the requested timeout blocks forever, and there exists a valid timer,
    // replace the requested timeout with the next timeout.
    bool has_valid_timer = next_timer_duration >= std::chrono::nanoseconds::zero();
    if ((next_timer_duration < timeout ||
      timeout < std::chrono::duration<int64_t, T>::zero()) && has_valid_timer)
    {
      rmw_timeout.sec =
        std::chrono::duration_cast<std::chrono::seconds>(next_timer_duration).count();
      rmw_timeout.nsec = next_timer_duration.count() % (1000 * 1000 * 1000);
      wait_timeout = &rmw_timeout;
    } else if (timeout >= std::chrono::duration<int64_t, T>::zero()) {
      // Convert timeout representation to rmw_time
      rmw_timeout.sec = std::chrono::duration_cast<std::chrono::seconds>(timeout).count();
      rmw_timeout.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count() %
        (1000 * 1000 * 1000);
      wait_timeout = &rmw_timeout;
    }

    // Now wait on the waitable subscriptions and timers
    rmw_ret_t status = rmw_wait(
      &subscriber_handles,
      &guard_condition_handles,
      &service_handles,
      &client_handles,
      wait_timeout);
    if (status != RMW_RET_OK && status != RMW_RET_TIMEOUT) {
      throw std::runtime_error(rmw_get_error_string_safe());
    }
    // If ctrl-c guard condition, return directly
    if (guard_condition_handles.guard_conditions[0] != 0) {
      // Make sure to free or clean memory
      memory_strategy_->return_handles(HandleType::subscriber_handle,
        subscriber_handles.subscribers);
      memory_strategy_->return_handles(HandleType::service_handle,
        service_handles.services);
      memory_strategy_->return_handles(HandleType::client_handle,
        client_handles.clients);
      memory_strategy_->return_handles(HandleType::guard_condition_handle,
        guard_condition_handles.guard_conditions);
      return;
    }
    // Add the new work to the class's list of things waiting to be executed
    // Starting with the subscribers
    for (size_t i = 0; i < number_of_subscriptions; ++i) {
      void * handle = subscriber_handles.subscribers[i];
      if (handle) {
        subscriber_handles_.push_back(handle);
      }
    }
    // Then the services
    for (size_t i = 0; i < number_of_services; ++i) {
      void * handle = service_handles.services[i];
      if (handle) {
        service_handles_.push_back(handle);
      }
    }
    // Then the clients
    for (size_t i = 0; i < number_of_clients; ++i) {
      void * handle = client_handles.clients[i];
      if (handle) {
        client_handles_.push_back(handle);
      }
    }

    memory_strategy_->return_handles(HandleType::subscriber_handle,
      subscriber_handles.subscribers);
    memory_strategy_->return_handles(HandleType::service_handle,
      service_handles.services);
    memory_strategy_->return_handles(HandleType::client_handle,
      client_handles.clients);
    memory_strategy_->return_handles(HandleType::guard_condition_handle,
      guard_condition_handles.guard_conditions);

  }

/******************************/

  rclcpp::subscription::SubscriptionBase::SharedPtr
  get_subscription_by_handle(void * subscriber_handle)
  {
    for (auto weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto weak_subscription : group->subscription_ptrs_) {
          auto subscription = weak_subscription.lock();
          if (subscription && subscription->subscription_handle_->data == subscriber_handle) {
            return subscription;
          }
        }
      }
    }
    return rclcpp::subscription::SubscriptionBase::SharedPtr();
  }

  rclcpp::service::ServiceBase::SharedPtr
  get_service_by_handle(void * service_handle)
  {
    for (auto weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto service : group->service_ptrs_) {
          if (service->service_handle_->data == service_handle) {
            return service;
          }
        }
      }
    }
    return rclcpp::service::ServiceBase::SharedPtr();
  }

  rclcpp::client::ClientBase::SharedPtr
  get_client_by_handle(void * client_handle)
  {
    for (auto weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto client : group->client_ptrs_) {
          if (client->client_handle_->data == client_handle) {
            return client;
          }
        }
      }
    }
    return rclcpp::client::ClientBase::SharedPtr();
  }

  rclcpp::node::Node::SharedPtr
  get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr & group)
  {
    if (!group) {
      return rclcpp::node::Node::SharedPtr();
    }
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->callback_groups_) {
        auto callback_group = weak_group.lock();
        if (!callback_group) {
          continue;
        }
        if (callback_group == group) {
          return node;
        }
      }
    }
    return rclcpp::node::Node::SharedPtr();
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_timer(
    rclcpp::timer::TimerBase::SharedPtr & timer)
  {
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        for (auto & weak_timer : group->timer_ptrs_) {
          auto t = weak_timer.lock();
          if (t == timer) {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  void
  get_next_timer(AnyExecutable::SharedPtr & any_exec)
  {
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        if (!group || !group->can_be_taken_from_.load()) {
          continue;
        }
        for (auto & timer_ref : group->timer_ptrs_) {
          auto timer = timer_ref.lock();
          if (timer && timer->check_and_trigger()) {
            any_exec->timer = timer;
            any_exec->callback_group = group;
            node = get_node_by_group(group);
            return;
          }
        }
      }
    }
  }

  std::chrono::nanoseconds
  get_earliest_timer()
  {
    std::chrono::nanoseconds latest = std::chrono::nanoseconds::max();
    bool timers_empty = true;
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        if (!group || !group->can_be_taken_from_.load()) {
          continue;
        }
        for (auto & timer_ref : group->timer_ptrs_) {
          timers_empty = false;
          // Check the expected trigger time
          auto timer = timer_ref.lock();
          if (timer && timer->time_until_trigger() < latest) {
            latest = timer->time_until_trigger();
          }
        }
      }
    }
    if (timers_empty) {
      return std::chrono::nanoseconds(-1);
    }
    return latest;
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr & subscription)
  {
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        for (auto & weak_sub : group->subscription_ptrs_) {
          auto sub = weak_sub.lock();
          if (sub == subscription) {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  void
  get_next_subscription(AnyExecutable::SharedPtr & any_exec)
  {
    auto it = subscriber_handles_.begin();
    while (it != subscriber_handles_.end()) {
      auto subscription = get_subscription_by_handle(*it);
      if (subscription) {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_subscription(subscription);
        if (!group) {
          // Group was not found, meaning the subscription is not valid...
          // Remove it from the ready list and continue looking
          subscriber_handles_.erase(it++);
          continue;
        }
        if (!group->can_be_taken_from_.load()) {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          ++it;
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->subscription = subscription;
        any_exec->callback_group = group;
        any_exec->node = get_node_by_group(group);
        subscriber_handles_.erase(it++);
        return;
      }
      // Else, the subscription is no longer valid, remove it and continue
      subscriber_handles_.erase(it++);
    }
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_service(
    rclcpp::service::ServiceBase::SharedPtr & service)
  {
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        for (auto & serv : group->service_ptrs_) {
          if (serv == service) {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  void
  get_next_service(AnyExecutable::SharedPtr & any_exec)
  {
    auto it = service_handles_.begin();
    while (it != service_handles_.end()) {
      auto service = get_service_by_handle(*it);
      if (service) {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_service(service);
        if (!group) {
          // Group was not found, meaning the service is not valid...
          // Remove it from the ready list and continue looking
          service_handles_.erase(it++);
          continue;
        }
        if (!group->can_be_taken_from_.load()) {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          ++it;
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->service = service;
        any_exec->callback_group = group;
        any_exec->node = get_node_by_group(group);
        service_handles_.erase(it++);
        return;
      }
      // Else, the service is no longer valid, remove it and continue
      service_handles_.erase(it++);
    }
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_client(
    rclcpp::client::ClientBase::SharedPtr & client)
  {
    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->callback_groups_) {
        auto group = weak_group.lock();
        for (auto & cli : group->client_ptrs_) {
          if (cli == client) {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  void
  get_next_client(AnyExecutable::SharedPtr & any_exec)
  {
    auto it = client_handles_.begin();
    while (it != client_handles_.end()) {
      auto client = get_client_by_handle(*it);
      if (client) {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_client(client);
        if (!group) {
          // Group was not found, meaning the service is not valid...
          // Remove it from the ready list and continue looking
          client_handles_.erase(it++);
          continue;
        }
        if (!group->can_be_taken_from_.load()) {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          ++it;
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->client = client;
        any_exec->callback_group = group;
        any_exec->node = get_node_by_group(group);
        client_handles_.erase(it++);
        return;
      }
      // Else, the service is no longer valid, remove it and continue
      client_handles_.erase(it++);
    }
  }

  AnyExecutable::SharedPtr
  get_next_ready_executable()
  {
    return get_next_ready_executable(this->memory_strategy_->instantiate_next_executable());
  }

  AnyExecutable::SharedPtr
  get_next_ready_executable(AnyExecutable::SharedPtr any_exec)
  {
    // Check the timers to see if there are any that are ready, if so return
    get_next_timer(any_exec);
    if (any_exec->timer) {
      return any_exec;
    }
    // Check the subscriptions to see if there are any that are ready
    get_next_subscription(any_exec);
    if (any_exec->subscription) {
      return any_exec;
    }
    // Check the services to see if there are any that are ready
    get_next_service(any_exec);
    if (any_exec->service) {
      return any_exec;
    }
    // Check the clients to see if there are any that are ready
    get_next_client(any_exec);
    if (any_exec->client) {
      return any_exec;
    }
    // If there is neither a ready timer nor subscription, return a null ptr
    any_exec.reset();
    return any_exec;
  }

  template<typename T = std::milli>
  AnyExecutable::SharedPtr
  get_next_executable(std::chrono::duration<int64_t, T> timeout = std::chrono::duration<int64_t,
    T>(-1))
  {
    // Check to see if there are any subscriptions or timers needing service
    // TODO(wjwwood): improve run to run efficiency of this function
    auto any_exec = get_next_ready_executable();
    // If there are none
    if (!any_exec) {
      // Wait for subscriptions or timers to work on
      wait_for_work(timeout);
      // Try again
      any_exec = get_next_ready_executable();
    }
    // At this point any_exec should be valid with either a valid subscription
    // or a valid timer, or it should be a null shared_ptr
    if (any_exec) {
      // If it is valid, check to see if the group is mutually exclusive or
      // not, then mark it accordingly
      if (any_exec->callback_group->type_ == \
        callback_group::CallbackGroupType::MutuallyExclusive)
      {
        // It should not have been taken otherwise
        assert(any_exec->callback_group->can_be_taken_from_.load());
        // Set to false to indicate something is being run from this group
        any_exec->callback_group->can_be_taken_from_.store(false);
      }
    }
    return any_exec;
  }

  rmw_guard_condition_t * interrupt_guard_condition_;

  memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;

private:
  RCLCPP_DISABLE_COPY(Executor);

  std::vector<std::weak_ptr<rclcpp::node::Node>> weak_nodes_;
  typedef std::list<void *> SubscriberHandles;
  SubscriberHandles subscriber_handles_;
  typedef std::list<void *> ServiceHandles;
  ServiceHandles service_handles_;
  typedef std::list<void *> ClientHandles;
  ClientHandles client_handles_;

};

} /* executor */
} /* rclcpp */

#endif /* RCLCPP_RCLCPP_EXECUTOR_HPP_ */
