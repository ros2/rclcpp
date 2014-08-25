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

  Executor()
  {
    subscriber_handles_.subscriber_count_ = 0;
    subscriber_handles_.subscribers_ = 0;
  }
  ~Executor() {}

  void add_node(rclcpp::node::Node::SharedPtr &node_ptr)
  {
    this->weak_nodes_.push_back(node_ptr);
  }

protected:
  static void execute_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr &subscription)
  {
    std::shared_ptr<void> message = subscription->create_message();
    bool taken = ros_middleware_interface::take(
      subscription->subscription_handle_,
      message.get());
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

  void reset_subscriber_handles()
  {
    if (subscriber_handles_.subscriber_count_ != 0)
    {
      subscriber_handles_.subscriber_count_ = 0;
      std::free(subscriber_handles_.subscribers_);
    }
  }

  void populate_subscriber_handles_with_node(rclcpp::node::Node &node)
  {
    // Use the number of subscriptions to pre allocate subscriber_handles
    subscriber_handles_.subscriber_count_ = node.number_of_subscriptions_;
    subscriber_handles_.subscribers_ = static_cast<void **>(
      std::malloc(sizeof(void *) * node.number_of_subscriptions_));
    // Add subscriptions from groups
    size_t handles_index = 0;
    for (auto group : node.callback_groups_)
    {
      for (auto subscription : group->subscription_ptrs_)
      {
        assert(handles_index < node.number_of_subscriptions_);
        subscriber_handles_.subscribers_[handles_index] = \
          subscription->subscription_handle_.data_;
        handles_index += 1;
      }
    }
    assert(handles_index == node.number_of_subscriptions_);
  }

  void populate_subscriber_handles()
  {
    // Calculate the number of subscriptions and create shared_ptrs
    size_t number_of_subscriptions = 0;
    std::vector<rclcpp::node::Node::SharedPtr> nodes;
    bool has_invalid_weak_nodes = false;
    for (auto &weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        has_invalid_weak_nodes = false;
        continue;
      }
      nodes.push_back(node);
      number_of_subscriptions += node->number_of_subscriptions_;
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
    // Use the number of subscriptions to pre allocate subscriber_handles
    subscriber_handles_.subscriber_count_ = number_of_subscriptions;
    subscriber_handles_.subscribers_ = static_cast<void **>(
      std::malloc(sizeof(void *) * number_of_subscriptions));
    if (subscriber_handles_.subscribers_ == NULL)
    {
      // TODO: Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for subscriber pointers.");
    }
    // Now fill the subscriber_handles with subscribers from the nodes
    size_t handles_index = 0;
    for (auto &node : nodes)
    {
      for (auto group : node->callback_groups_)
      {
        for (auto subscription : group->subscription_ptrs_)
        {
          assert(handles_index < number_of_subscriptions);
          subscriber_handles_.subscribers_[handles_index] = \
            subscription->subscription_handle_.data_;
          handles_index += 1;
        }
      }
    }
    assert(handles_index == number_of_subscriptions);
  }

  struct AnyExecutable
  {
    AnyExecutable() : subscription(0), guard_condition_handle(0) {}
    rclcpp::subscription::SubscriptionBase::SharedPtr subscription;
    ros_middleware_interface::GuardConditionHandle *guard_condition_handle;
  };

  std::list<void *> get_ready_subscriber_handles()
  {
    std::list<void *> ready_subscriber_handles;
    for (size_t i = 0; i < subscriber_handles_.subscriber_count_; ++i)
    {
      void *handle = subscriber_handles_.subscribers_[i];
      if (!handle)
      {
        continue;
      }
      ready_subscriber_handles.push_back(handle);
    }
    return ready_subscriber_handles;
  }

  rclcpp::subscription::SubscriptionBase::SharedPtr get_subscription_by_handle(
    void * subscriber_handle)
  {
    for (auto weak_node : weak_nodes_)
    {
      auto node = weak_node.lock();
      if (!node)
      {
        continue;
      }
      for (auto group : node->callback_groups_)
      {
        for (auto subscription : group->subscription_ptrs_)
        {
          if (subscription->subscription_handle_.data_ == subscriber_handle)
          {
            return subscription;
          }
        }
      }
    }
    return rclcpp::subscription::SubscriptionBase::SharedPtr();
  }

  void remove_subscriber_handle_from_subscriber_handles_(void *handle)
  {
    for (size_t i = 0; i < subscriber_handles_.subscriber_count_; ++i)
    {
      if (handle == subscriber_handles_.subscribers_[i])
      {
        // TODO: use nullptr here?
        subscriber_handles_.subscribers_[i] = NULL;
      }
    }
  }

  rclcpp::subscription::SubscriptionBase::SharedPtr get_next_subscription(
    std::list<void *> &ready_subscriber_handles)
  {
    if (ready_subscriber_handles.size() == 0)
    {
      return rclcpp::subscription::SubscriptionBase::SharedPtr();
    }
    while (ready_subscriber_handles.size() != 0)
    {
      void *handle = ready_subscriber_handles.front();
      ready_subscriber_handles.pop_front();
      remove_subscriber_handle_from_subscriber_handles_(handle);
      auto subscription = get_subscription_by_handle(handle);
      if (subscription) return subscription;
    }
    return rclcpp::subscription::SubscriptionBase::SharedPtr();
  }

  std::shared_ptr<AnyExecutable> get_next_executable(bool nonblocking=false)
  {
    namespace rmi = ros_middleware_interface;
    // Get any ready subscriber handles out of subscriber_handles_.
    // TODO: operate directly on subscriber_handles_ or
    // store ready_subscriber_handles to prevent rebuilding it every time
    auto ready_subscriber_handles = get_ready_subscriber_handles();
    // If there are none
    if (ready_subscriber_handles.size() == 0)
    {
      // Repopulate the subscriber handles and wait on them
      reset_subscriber_handles();
      populate_subscriber_handles();
      // TODO: populate the guard handles correctly
      rmi::GuardConditionHandles guard_condition_handles;
      guard_condition_handles.guard_condition_count_ = 1;
      guard_condition_handles.guard_conditions_ = static_cast<void **>(
        std::malloc(sizeof(void *) * 1));
      guard_condition_handles.guard_conditions_[0] = \
        rclcpp::utilities::get_global_sigint_guard_cond().data_;
      // Wait on the handles.
      rmi::wait(subscriber_handles_, guard_condition_handles, nonblocking);
      std::free(guard_condition_handles.guard_conditions_);
      ready_subscriber_handles = get_ready_subscriber_handles();
    }
    // At this point there should be something in either the subscriber handles
    // Or in the guard handles.
    // TODO: also check the timers here
    std::shared_ptr<AnyExecutable> any_exec(new AnyExecutable());
    // TODO: actually check the guard handles
    any_exec->guard_condition_handle = 0;
    // If any_exec.guard_condition_handle is valid, return now before checking
    // for valid subscriptions.
    if (any_exec->guard_condition_handle)
    {
      return any_exec;
    }
    // Check for subscriptions which are ready to be handled
    any_exec->subscription = get_next_subscription(ready_subscriber_handles);
    // If any_exec.subscription is valid, return now
    if (any_exec->subscription)
    {
      return any_exec;
    }
    // If there is neither a ready guard condition nor subscription, return an
    // empty any_exec anyways
    return any_exec;
  }

private:
  RCLCPP_DISABLE_COPY(Executor);

  std::vector<std::weak_ptr<rclcpp::node::Node>> weak_nodes_;
  ros_middleware_interface::SubscriberHandles subscriber_handles_;

};

} /* executor */
} /* rclcpp */

#endif /* RCLCPP_RCLCPP_EXECUTOR_HPP_ */
