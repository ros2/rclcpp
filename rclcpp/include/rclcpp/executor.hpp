/* Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RCLCPP_RCLCPP_EXECUTOR_HPP_
#define RCLCPP_RCLCPP_EXECUTOR_HPP_

#include <iostream>

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <list>
#include <memory>
#include <vector>

#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

namespace rclcpp
{
namespace executor
{

class Executor
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(Executor);

  Executor() : interrupt_guard_condition_(rmw_create_guard_condition()) {}

  virtual ~Executor()
  {
    if (interrupt_guard_condition_ != nullptr)
    {
      // TODO(wjwwood): Check ret code.
      rmw_destroy_guard_condition(interrupt_guard_condition_);
    }
  }

  virtual void spin() = 0;

  virtual void
  add_node(rclcpp::node::Node::SharedPtr &node_ptr)
  {
    // Check to ensure node not already added
    for (auto &weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (node == node_ptr)
      {
        // TODO: Use a different error here?
        throw std::runtime_error(
          "Cannot add node to executor, node already added.");
      }
    }
    weak_nodes_.push_back(node_ptr);
    // Interrupt waiting to handle new node
    rmw_trigger_guard_condition(interrupt_guard_condition_);
  }

  virtual void
  remove_node(rclcpp::node::Node::SharedPtr &node_ptr)
  {
    bool node_removed = false;
    weak_nodes_.erase(
      std::remove_if(weak_nodes_.begin(), weak_nodes_.end(),
        [&](std::weak_ptr<rclcpp::node::Node> &i)
        {
          bool matched = (i.lock() == node_ptr);
          node_removed |= matched;
          return matched;
        }));
    // If the node was matched and removed, interrupt waiting
    if (node_removed)
    {
      rmw_trigger_guard_condition(interrupt_guard_condition_);
    }
  }

  void spin_node_some(rclcpp::node::Node::SharedPtr &node)
  {
    this->add_node(node);
    // non-blocking = true
    std::shared_ptr<AnyExecutable> any_exec;
    while ((any_exec = get_next_executable(true)))
    {
      execute_any_executable(any_exec);
    }
    this->remove_node(node);
  }

protected:
  struct AnyExecutable
  {
    AnyExecutable() : subscription(0), timer(0), callback_group(0), node(0) {}
    // Either the subscription or the timer will be set, but not both
    rclcpp::subscription::SubscriptionBase::SharedPtr subscription;
    rclcpp::timer::TimerBase::SharedPtr timer;
    rclcpp::service::ServiceBase::SharedPtr service;
    rclcpp::client::ClientBase::SharedPtr client;
    // These are used to keep the scope on the containing items
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;
    rclcpp::node::Node::SharedPtr node;
  };

  void
  execute_any_executable(std::shared_ptr<AnyExecutable> &any_exec)
  {
    if (!any_exec)
    {
      return;
    }
    if (any_exec->timer)
    {
      execute_timer(any_exec->timer);
    }
    if (any_exec->subscription)
    {
      execute_subscription(any_exec->subscription);
    }
    if (any_exec->service)
    {
      execute_service(any_exec->service);
    }
    if (any_exec->client)
    {
      execute_client(any_exec->client);
    }
    // Reset the callback_group, regardless of type
    any_exec->callback_group->can_be_taken_from_.store(true);
    // Wake the wait, because it may need to be recalculated or work that
    // was previously blocked is now available.
    rmw_trigger_guard_condition(interrupt_guard_condition_);
  }

  static void
  execute_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr &subscription)
  {
    std::shared_ptr<void> message = subscription->create_message();
    auto taken = rmw_take(subscription->subscription_handle_, message.get());
    // TODO(wjwwood): taken is no longer a boolean, check against return types.
    if (taken)
    {
      subscription->handle_message(message);
    }
    else
    {
      std::cout << "[rclcpp::error] take failed for subscription on topic: "
                << subscription->get_topic_name()
                << std::endl;
    }
  }

  static void
  execute_timer(
    rclcpp::timer::TimerBase::SharedPtr &timer)
  {
    timer->callback_();
  }

  static void
  execute_service(
    rclcpp::service::ServiceBase::SharedPtr &service)
  {
    std::shared_ptr<void> request = service->create_request();
    std::shared_ptr<void> request_header = service->create_request_header();
    bool taken = rmw_take_request(service->service_handle_,
                                  request.get(),
                                  request_header.get());
    if (taken)
    {
      service->handle_request(request, request_header);
    }
    else
    {
      std::cout << "[rclcpp::error] take failed for service on service: "
                << service->get_service_name()
                << std::endl;
    }
  }

  static void
  execute_client(
    rclcpp::client::ClientBase::SharedPtr &client)
  {
    std::shared_ptr<void> response = client->create_response();
    std::shared_ptr<void> request_header = client->create_request_header();
    bool taken = rmw_take_response(client->client_handle_,
                                   response.get(),
                                   request_header.get());
    if (taken)
    {
      client->handle_response(response, request_header);
    }
    else
    {
      std::cout << "[rclcpp::error] take failed for service on client" << std::endl;
    }
  }

/*** Populate class storage from stored weak node pointers and wait. ***/

  void
  wait_for_work(bool nonblocking)
  {
    // Collect the subscriptions and timers to be waited on
    bool has_invalid_weak_nodes = false;
    std::vector<rclcpp::subscription::SubscriptionBase::SharedPtr> subs;
    std::vector<rclcpp::timer::TimerBase::SharedPtr> timers;
    std::vector<rclcpp::service::ServiceBase::SharedPtr> services;
    std::vector<rclcpp::client::ClientBase::SharedPtr> clients;
    for (auto &weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        has_invalid_weak_nodes = false;
        continue;
      }
      for (auto &weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        if (!group || group->can_be_taken_from_.load() == false)
        {
          continue;
        }
        for (auto &subscription : group->subscription_ptrs_)
        {
          subs.push_back(subscription);
        }
        for (auto &timer : group->timer_ptrs_)
        {
          timers.push_back(timer);
        }
        for (auto &service : group->service_ptrs_)
        {
          services.push_back(service);
        }
        for (auto &client : group->client_ptrs_)
        {
          clients.push_back(client);
        }
      }
    }
    // Clean up any invalid nodes, if they were detected
    if (has_invalid_weak_nodes)
    {
      weak_nodes_.erase(remove_if(weak_nodes_.begin(), weak_nodes_.end(),
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
    subscriber_handles.subscribers = static_cast<void **>(
      std::malloc(sizeof(void *) * number_of_subscriptions));
    if (subscriber_handles.subscribers == NULL)
    {
      // TODO(wjwwood): Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for subscriber pointers.");
    }
    // Then fill the SubscriberHandles with ready subscriptions
    size_t subscriber_handle_index = 0;
    for (auto &subscription : subs)
    {
      subscriber_handles.subscribers[subscriber_handle_index] = \
        subscription->subscription_handle_->data;
      subscriber_handle_index += 1;
    }

    // Use the number of services to allocate memory in the handles
    size_t number_of_services = services.size();
    rmw_services_t service_handles;
    service_handles.service_count = number_of_services;
    // TODO(esteve): Avoid redundant malloc's
    service_handles.services = static_cast<void **>(
      std::malloc(sizeof(void *) * number_of_services));
    if (service_handles.services == NULL)
    {
      // TODO(esteve): Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for service pointers.");
    }
    // Then fill the ServiceHandles with ready services
    size_t service_handle_index = 0;
    for (auto &service : services)
    {
      service_handles.services[service_handle_index] = \
        service->service_handle_->data;
      service_handle_index += 1;
    }

    // Use the number of clients to allocate memory in the handles
    size_t number_of_clients = clients.size();
    rmw_clients_t client_handles;
    client_handles.client_count = number_of_clients;
    // TODO: Avoid redundant malloc's
    client_handles.clients = static_cast<void **>(
      std::malloc(sizeof(void *) * number_of_clients));
    if (client_handles.clients == NULL)
    {
      // TODO: Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for client pointers.");
    }
    // Then fill the ServiceHandles with ready clients
    size_t client_handle_index = 0;
    for (auto &client : clients)
    {
      client_handles.clients[client_handle_index] = \
        client->client_handle_->data;
      client_handle_index += 1;
    }

    // Use the number of guard conditions to allocate memory in the handles
    // Add 2 to the number for the ctrl-c guard cond and the executor's
    size_t start_of_timer_guard_conds = 2;
    size_t number_of_guard_conds = timers.size() + start_of_timer_guard_conds;
    rmw_guard_conditions_t guard_condition_handles;
    guard_condition_handles.guard_condition_count = number_of_guard_conds;
    // TODO(wjwwood): Avoid redundant malloc's
    guard_condition_handles.guard_conditions = static_cast<void **>(
      std::malloc(sizeof(void *) * number_of_guard_conds));
    if (guard_condition_handles.guard_conditions == NULL)
    {
      // TODO(wjwwood): Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error(
        "Could not malloc for guard condition pointers.");
    }
    // Put the global ctrl-c guard condition in
    assert(guard_condition_handles.guard_condition_count > 1);
    guard_condition_handles.guard_conditions[0] = \
        rclcpp::utilities::get_global_sigint_guard_condition()->data;
    // Put the executor's guard condition in
    guard_condition_handles.guard_conditions[1] = \
        interrupt_guard_condition_->data;
    // Then fill the SubscriberHandles with ready subscriptions
    size_t guard_cond_handle_index = start_of_timer_guard_conds;
    for (auto &timer : timers)
    {
      guard_condition_handles.guard_conditions[guard_cond_handle_index] = \
        timer->guard_condition_->data;
      guard_cond_handle_index += 1;
    }

    // Now wait on the waitable subscriptions and timers
    rmw_wait(&subscriber_handles,
             &guard_condition_handles,
             &service_handles,
             &client_handles,
             nonblocking);
    // If ctrl-c guard condition, return directly
    if (guard_condition_handles.guard_conditions[0] != 0)
    {
      // Make sure to free memory
      // TODO(wjwwood): Remove theses when the "Avoid redundant malloc's"
      //                todo has been addressed.
      std::free(subscriber_handles.subscribers);
      std::free(service_handles.services);
      std::free(guard_condition_handles.guard_conditions);
      return;
    }
    // Add the new work to the class's list of things waiting to be executed
    // Starting with the subscribers
    for (size_t i = 0; i < number_of_subscriptions; ++i)
    {
      void *handle = subscriber_handles.subscribers[i];
      if (handle)
      {
        subscriber_handles_.push_back(handle);
      }
    }
    // Then the timers, start with start_of_timer_guard_conds
    for (size_t i = start_of_timer_guard_conds; i < number_of_guard_conds; ++i)
    {
      void *handle = guard_condition_handles.guard_conditions[i];
      if (handle)
      {
        guard_condition_handles_.push_back(handle);
      }
    }
    // Then the services
    for (size_t i = 0; i < number_of_services; ++i)
    {
      void *handle = service_handles.services[i];
      if (handle)
      {
        service_handles_.push_back(handle);
      }
    }
    // Then the clients
    for (size_t i = 0; i < number_of_clients; ++i)
    {
      void *handle = client_handles.clients[i];
      if (handle)
      {
        client_handles_.push_back(handle);
      }
    }

    // Make sure to free memory
    // TODO(wjwwood): Remove theses when the "Avoid redundant malloc's"
    //                todo has been addressed.
    std::free(subscriber_handles.subscribers);
    std::free(service_handles.services);
    std::free(guard_condition_handles.guard_conditions);
  }

/******************************/

  rclcpp::subscription::SubscriptionBase::SharedPtr
  get_subscription_by_handle(void * subscriber_handle)
  {
    for (auto weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        if (!group)
        {
          continue;
        }
        for (auto subscription : group->subscription_ptrs_)
        {
          if (subscription->subscription_handle_->data == subscriber_handle)
          {
            return subscription;
          }
        }
      }
    }
    return rclcpp::subscription::SubscriptionBase::SharedPtr();
  }

  rclcpp::timer::TimerBase::SharedPtr
  get_timer_by_handle(void * guard_condition_handle)
  {
    for (auto weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        if (!group)
        {
          continue;
        }
        for (auto timer : group->timer_ptrs_)
        {
          if (timer->guard_condition_->data == guard_condition_handle)
          {
            return timer;
          }
        }
      }
    }
    return rclcpp::timer::TimerBase::SharedPtr();
  }

  rclcpp::service::ServiceBase::SharedPtr
  get_service_by_handle(void * service_handle)
  {
    for (auto weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        if (!group)
        {
          continue;
        }
        for (auto service : group->service_ptrs_)
        {
          if (service->service_handle_->data == service_handle)
          {
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
    for (auto weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        if (!group)
        {
          continue;
        }
        for (auto client : group->client_ptrs_)
        {
          if (client->client_handle_->data == client_handle)
          {
            return client;
          }
        }
      }
    }
    return rclcpp::client::ClientBase::SharedPtr();
  }


  void
  remove_subscriber_handle_from_subscriber_handles(void *handle)
  {
    subscriber_handles_.remove(handle);
  }

  void
  remove_guard_condition_handle_from_guard_condition_handles(void *handle)
  {
    guard_condition_handles_.remove(handle);
  }

  void
  remove_service_handle_from_service_handles(void *handle)
  {
    service_handles_.remove(handle);
  }

  void
  remove_client_handle_from_client_handles(void *handle)
  {
    client_handles_.remove(handle);
  }

  rclcpp::node::Node::SharedPtr
  get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr &group)
  {
    if (!group)
    {
      return rclcpp::node::Node::SharedPtr();
    }
    for (auto &weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto &weak_group : node->callback_groups_)
      {
        auto callback_group = weak_group.lock();
        if (!callback_group)
        {
          continue;
        }
        if (callback_group == group)
        {
          return node;
        }
      }
    }
    return rclcpp::node::Node::SharedPtr();
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_timer(
    rclcpp::timer::TimerBase::SharedPtr &timer)
  {
    for (auto &weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto &weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        for (auto &t : group->timer_ptrs_)
        {
          if (t == timer)
          {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  void
  get_next_timer(std::shared_ptr<AnyExecutable> &any_exec)
  {
    for (auto handle : guard_condition_handles_)
    {
      auto timer = get_timer_by_handle(handle);
      if (timer)
      {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_timer(timer);
        if (!group)
        {
          // Group was not found, meaning the timer is not valid...
          // Remove it from the ready list and continue looking
          remove_guard_condition_handle_from_guard_condition_handles(handle);
          continue;
        }
        if (!group->can_be_taken_from_.load())
        {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->timer = timer;
        any_exec->callback_group = group;
        any_exec->node = get_node_by_group(group);
        remove_guard_condition_handle_from_guard_condition_handles(handle);
        return;
      }
      // Else, the timer is no longer valid, remove it and continue
      remove_guard_condition_handle_from_guard_condition_handles(handle);
    }
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr &subscription)
  {
    for (auto &weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto &weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        for (auto &sub : group->subscription_ptrs_)
        {
          if (sub == subscription)
          {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  void
  get_next_subscription(std::shared_ptr<AnyExecutable> &any_exec)
  {
    for (auto handle : subscriber_handles_)
    {
      auto subscription = get_subscription_by_handle(handle);
      if (subscription)
      {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_subscription(subscription);
        if (!group)
        {
          // Group was not found, meaning the subscription is not valid...
          // Remove it from the ready list and continue looking
          remove_subscriber_handle_from_subscriber_handles(handle);
          continue;
        }
        if (!group->can_be_taken_from_.load())
        {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->subscription = subscription;
        any_exec->callback_group = group;
        any_exec->node = get_node_by_group(group);
        remove_subscriber_handle_from_subscriber_handles(handle);
        return;
      }
      // Else, the subscription is no longer valid, remove it and continue
      remove_subscriber_handle_from_subscriber_handles(handle);
    }
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_service(
    rclcpp::service::ServiceBase::SharedPtr &service)
  {
    for (auto &weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto &weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        for (auto &serv : group->service_ptrs_)
        {
          if (serv == service)
          {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  void
  get_next_service(std::shared_ptr<AnyExecutable> &any_exec)
  {
    for (auto handle : service_handles_)
    {
      auto service = get_service_by_handle(handle);
      if (service)
      {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_service(service);
        if (!group)
        {
          // Group was not found, meaning the service is not valid...
          // Remove it from the ready list and continue looking
          remove_service_handle_from_service_handles(handle);
          continue;
        }
        if (!group->can_be_taken_from_.load())
        {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->service = service;
        any_exec->callback_group = group;
        any_exec->node = get_node_by_group(group);
        remove_service_handle_from_service_handles(handle);
        return;
      }
      // Else, the service is no longer valid, remove it and continue
      remove_service_handle_from_service_handles(handle);
    }
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_client(
    rclcpp::client::ClientBase::SharedPtr &client)
  {
    for (auto &weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto &weak_group : node->callback_groups_)
      {
        auto group = weak_group.lock();
        for (auto &cli : group->client_ptrs_)
        {
          if (cli == client)
          {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  void
  get_next_client(std::shared_ptr<AnyExecutable> &any_exec)
  {
    for (auto handle : client_handles_)
    {
      auto client = get_client_by_handle(handle);
      if (client)
      {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_client(client);
        if (!group)
        {
          // Group was not found, meaning the service is not valid...
          // Remove it from the ready list and continue looking
          remove_client_handle_from_client_handles(handle);
          continue;
        }
        if (!group->can_be_taken_from_.load())
        {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->client = client;
        any_exec->callback_group = group;
        any_exec->node = get_node_by_group(group);
        remove_client_handle_from_client_handles(handle);
        return;
      }
      // Else, the service is no longer valid, remove it and continue
      remove_client_handle_from_client_handles(handle);
    }
  }

  std::shared_ptr<AnyExecutable>
  get_next_ready_executable()
  {
    std::shared_ptr<AnyExecutable> any_exec(new AnyExecutable());
    // Check the timers to see if there are any that are ready, if so return
    get_next_timer(any_exec);
    if (any_exec->timer)
    {
      return any_exec;
    }
    // Check the subscriptions to see if there are any that are ready
    get_next_subscription(any_exec);
    if (any_exec->subscription)
    {
      return any_exec;
    }
    // Check the services to see if there are any that are ready
    get_next_service(any_exec);
    if (any_exec->service)
    {
      return any_exec;
    }
    // Check the clients to see if there are any that are ready
    get_next_client(any_exec);
    if (any_exec->client)
    {
      return any_exec;
    }
    // If there is neither a ready timer nor subscription, return a null ptr
    any_exec.reset();
    return any_exec;
  }

  std::shared_ptr<AnyExecutable>
  get_next_executable(bool nonblocking=false)
  {
    // Check to see if there are any subscriptions or timers needing service
    // TODO(wjwwood): improve run to run efficiency of this function
    auto any_exec = get_next_ready_executable();
    // If there are none
    if (!any_exec)
    {
      // Wait for subscriptions or timers to work on
      wait_for_work(nonblocking);
      // Try again
      any_exec = get_next_ready_executable();
    }
    // At this point any_exec should be valid with either a valid subscription
    // or a valid timer, or it should be a null shared_ptr
    if (any_exec)
    {
      // If it is valid, check to see if the group is mutually exclusive or
      // not, then mark it accordingly
      if (any_exec->callback_group->type_ == \
            callback_group::CallbackGroupType::MutuallyExclusive)
      {
        // It should not have been taken otherwise
        assert(any_exec->callback_group->can_be_taken_from_.load() == true);
        // Set to false to indicate something is being run from this group
        any_exec->callback_group->can_be_taken_from_.store(false);
      }
    }
    return any_exec;
  }

  rmw_guard_condition_t * interrupt_guard_condition_;

private:
  RCLCPP_DISABLE_COPY(Executor);

  std::vector<std::weak_ptr<rclcpp::node::Node>> weak_nodes_;
  typedef std::list<void*> SubscriberHandles;
  SubscriberHandles subscriber_handles_;
  typedef std::list<void*> GuardConditionHandles;
  GuardConditionHandles guard_condition_handles_;
  typedef std::list<void*> ServiceHandles;
  ServiceHandles service_handles_;
  typedef std::list<void*> ClientHandles;
  ClientHandles client_handles_;

};

} /* executor */
} /* rclcpp */

#endif /* RCLCPP_RCLCPP_EXECUTOR_HPP_ */
