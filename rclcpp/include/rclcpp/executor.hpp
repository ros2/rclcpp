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
    : interrupt_guard_condition_(
        ros_middleware_interface::create_guard_condition())
  {}
  virtual ~Executor() {}

  virtual void spin() = 0;

  virtual void
  add_node(rclcpp::node::Node::SharedPtr &node_ptr)
  {
    this->weak_nodes_.push_back(node_ptr);
  }

protected:
  struct AnyExecutable
  {
    AnyExecutable() : subscription(0), timer(0), callback_group(0), node(0) {}
    // Either the subscription or the timer will be set, but not both
    rclcpp::subscription::SubscriptionBase::SharedPtr subscription;
    rclcpp::timer::TimerBase::SharedPtr timer;
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
    // Reset the callback_group, regardless of type
    any_exec->callback_group->can_be_taken_from_.store(true);
    // Wake the wait, because it may need to be recalculated or work that
    // was previously blocked is now available.
    using ros_middleware_interface::trigger_guard_condition;
    trigger_guard_condition(interrupt_guard_condition_);
  }

  static void
  execute_subscription(
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

  static void
  execute_timer(
    rclcpp::timer::TimerBase::SharedPtr &timer)
  {
    timer->callback_();
  }

/*** Reseting class storage ***/

  void
  reset_all_handles()
  {
    reset_subscriber_handles();
    reset_guard_condition_handles();
  }

  void
  reset_subscriber_handles()
  {
    subscriber_handles_.clear();
  }

  void
  reset_guard_condition_handles()
  {
    guard_condition_handles_.clear();
  }

/******************************/

/*** Populating class storage from a single node ***/

  // TODO: pick a better name for this function
  void
  populate_all_handles_with_node(rclcpp::node::Node &node)
  {
    // TODO: reimplement
  }

/******************************/

/*** Populate class storage from stored weak node pointers and wait. ***/

  // TODO: pick a better name for this function
  void
  populate_all_handles(bool nonblocking)
  {
    // Collect the subscriptions and timers to be waited on
    bool has_invalid_weak_nodes = false;
    std::vector<rclcpp::subscription::SubscriptionBase::SharedPtr> subs;
    std::vector<rclcpp::timer::TimerBase::SharedPtr> timers;
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
    ros_middleware_interface::SubscriberHandles subscriber_handles;
    subscriber_handles.subscriber_count_ = number_of_subscriptions;
    // TODO: Avoid redundant malloc's
    subscriber_handles.subscribers_ = static_cast<void **>(
      std::malloc(sizeof(void *) * number_of_subscriptions));
    if (subscriber_handles.subscribers_ == NULL)
    {
      // TODO: Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error("Could not malloc for subscriber pointers.");
    }
    // Then fill the SubscriberHandles with ready subscriptions
    size_t subscriber_handle_index = 0;
    for (auto &subscription : subs)
    {
      subscriber_handles.subscribers_[subscriber_handle_index] = \
        subscription->subscription_handle_.data_;
      subscriber_handle_index += 1;
    }
    // Use the number of guard conditions to allocate memory in the handles
    // Add 2 to the number for the ctrl-c guard cond and the executor's
    size_t start_of_timer_guard_conds = 2;
    size_t number_of_guard_conds = timers.size() + start_of_timer_guard_conds;
    ros_middleware_interface::GuardConditionHandles guard_condition_handles;
    guard_condition_handles.guard_condition_count_ = number_of_guard_conds;
    // TODO: Avoid redundant malloc's
    guard_condition_handles.guard_conditions_ = static_cast<void **>(
      std::malloc(sizeof(void *) * number_of_guard_conds));
    if (guard_condition_handles.guard_conditions_ == NULL)
    {
      // TODO: Use a different error here? maybe std::bad_alloc?
      throw std::runtime_error(
        "Could not malloc for guard condition pointers.");
    }
    // Put the global ctrl-c guard condition in
    assert(guard_condition_handles.guard_condition_count_ > 1);
    guard_condition_handles.guard_conditions_[0] = \
        rclcpp::utilities::get_global_sigint_guard_condition().data_;
    // Put the executor's guard condition in
    guard_condition_handles.guard_conditions_[1] = \
        interrupt_guard_condition_.data_;
    // Then fill the SubscriberHandles with ready subscriptions
    size_t guard_cond_handle_index = start_of_timer_guard_conds;
    for (auto &timer : timers)
    {
      guard_condition_handles.guard_conditions_[guard_cond_handle_index] = \
        timer->guard_condition_.data_;
      guard_cond_handle_index += 1;
    }
    // Now wait on the waitable subscriptions and timers
    ros_middleware_interface::wait(subscriber_handles,
                                   guard_condition_handles,
                                   nonblocking);
    // Add the new work to the class's list of things waiting to be executed
    // Starting with the subscribers
    for (size_t i = 0; i < number_of_subscriptions; ++i)
    {
      void *handle = subscriber_handles.subscribers_[i];
      if (handle)
      {
        subscriber_handles_.push_back(handle);
      }
    }
    // Then the timers, start with start_of_timer_guard_conds
    for (size_t i = start_of_timer_guard_conds; i < number_of_guard_conds; ++i)
    {
      void *handle = guard_condition_handles.guard_conditions_[i];
      if (handle)
      {
        guard_condition_handles_.push_back(handle);
      }
    }
    // Make sure to free memory
    // TODO: Remove theses when "Avoid redundant malloc's" todo is addressed
    std::free(subscriber_handles.subscribers_);
    std::free(guard_condition_handles.guard_conditions_);
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
          if (subscription->subscription_handle_.data_ == subscriber_handle)
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
          if (timer->guard_condition_.data_ == guard_condition_handle)
          {
            return timer;
          }
        }
      }
    }
    return rclcpp::timer::TimerBase::SharedPtr();
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
    // If there is neither a ready timer nor subscription, return a null ptr
    any_exec.reset();
    return any_exec;
  }

  std::shared_ptr<AnyExecutable>
  get_next_executable(bool nonblocking=false)
  {
    namespace rmi = ros_middleware_interface;
    // Check to see if there are any subscriptions or timers needing service
    // TODO: improve run to run efficiency of this function
    auto any_exec = get_next_ready_executable();
    // If there are none
    if (!any_exec)
    {
      // Repopulate the subscriber handles and wait on them
      populate_all_handles(nonblocking);
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

  ros_middleware_interface::GuardConditionHandle interrupt_guard_condition_;
  std::vector<std::weak_ptr<rclcpp::node::Node>> weak_nodes_;
  typedef std::list<void*> SubscriberHandles;
  SubscriberHandles subscriber_handles_;
  typedef std::list<void*> GuardConditionHandles;
  GuardConditionHandles guard_condition_handles_;

private:
  RCLCPP_DISABLE_COPY(Executor);

};

} /* executor */
} /* rclcpp */

#endif /* RCLCPP_RCLCPP_EXECUTOR_HPP_ */
