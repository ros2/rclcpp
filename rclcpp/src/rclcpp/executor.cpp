// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <string>
#include <type_traits>

#include "rclcpp/executor.hpp"
#include "rclcpp/scope_exit.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

using rclcpp::executor::AnyExecutable;
using rclcpp::executor::Executor;
using rclcpp::executor::ExecutorArgs;
using rclcpp::executor::FutureReturnCode;

Executor::Executor(const ExecutorArgs & args)
: spinning(false),
  memory_strategy_(args.memory_strategy)
{
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  if (rcl_guard_condition_init(
    &interrupt_guard_condition_, guard_condition_options) != RCL_RET_OK)
  {
    throw std::runtime_error("Failed to create interrupt guard condition in Executor constructor");
  }

  // The number of guard conditions is always at least 2: 1 for the ctrl-c guard cond,
  // and one for the executor's guard cond (interrupt_guard_condition_)

  // These guard conditions are permanently attached to the waitset.
  // const size_t number_of_guard_conds = 2;
  //fixed_guard_conditions_.guard_condition_count = number_of_guard_conds;
  //fixed_guard_conditions_.guard_conditions = static_cast<void **>(guard_cond_handles_.data());

  // Put the global ctrl-c guard condition in
  memory_strategy_->add_guard_condition(rclcpp::utilities::get_global_sigint_guard_condition());

  // Put the executor's guard condition in
  memory_strategy_->add_guard_condition(interrupt_guard_condition_);

  if (rcl_wait_set_init(
    &waitset_, fixed_guard_conditions_.data(), 0, 0, 0, 0, 0, 0, rcl_get_default_allocator()) != RCL_RET_OK)
  {
    fprintf(stderr,
      "[rclcpp::error] failed to create waitset: %s\n", rmw_get_error_string_safe());
    if (rcl_guard_condition_fini(&interrupt_guard_condition_) != RCL_RET_OK) {
      fprintf(stderr,
        "[rclcpp::error] failed to destroy guard condition: %s\n", rmw_get_error_string_safe());
    }
    throw std::runtime_error("Failed to create waitset in Executor constructor");
  }
}

Executor::~Executor()
{
  // Finalize the waitset.
  if (rcl_wait_set_fini(&waitset_) != RMW_RET_OK) {
    fprintf(stderr,
      "[rclcpp::error] failed to destroy waitset: %s\n", rcl_get_error_string_safe());
  }
  // Finalize the interrupt guard condition.
  if (rcl_guard_condition_fini(&interrupt_guard_condition_) != RCL_RET_OK) {
    fprintf(stderr,
      "[rclcpp::error] failed to destroy guard condition: %s\n", rcl_get_error_string_safe());
  }
}

void
Executor::add_node(rclcpp::node::Node::SharedPtr node_ptr, bool notify)
{
  // If the node already has an executor
  if (node_ptr->has_executor.exchange(true)) {
    throw std::runtime_error("Node has already been added to an executor.");
  }
  // Check to ensure node not already added
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node == node_ptr) {
      // TODO(jacquelinekay): Use a different error here?
      throw std::runtime_error("Cannot add node to executor, node already added.");
    }
  }
  weak_nodes_.push_back(node_ptr);
  if (notify) {
    // Interrupt waiting to handle new node
    if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RMW_RET_OK) {
      throw std::runtime_error(rcl_get_error_string_safe());
    }
  }
  // Add the node's notify condition to the guard condition handles
  memory_strategy_->add_guard_condition(node_ptr->get_notify_guard_condition());
}

void
Executor::remove_node(rclcpp::node::Node::SharedPtr node_ptr, bool notify)
{
  bool node_removed = false;
  weak_nodes_.erase(
    std::remove_if(
      weak_nodes_.begin(), weak_nodes_.end(),
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      [&](std::weak_ptr<rclcpp::node::Node> & i)
      {
        bool matched = (i.lock() == node_ptr);
        node_removed |= matched;
        return matched;
      }
      // *INDENT-ON*
    )
  );
  node_ptr->has_executor.store(false);
  if (notify) {
    // If the node was matched and removed, interrupt waiting
    if (node_removed) {
      if (rcl_trigger_guard_condition(&interrupt_guard_condition_)!= RCL_RET_OK) {
        throw std::runtime_error(rcl_get_error_string_safe());
      }
    }
  }
  memory_strategy_->remove_guard_condition(node_ptr->get_notify_guard_condition());
}

void
Executor::spin_node_once_nanoseconds(
  rclcpp::node::Node::SharedPtr node,
  std::chrono::nanoseconds timeout)
{
  this->add_node(node, false);
  // non-blocking = true
  spin_once(timeout);
  this->remove_node(node, false);
}

void
Executor::spin_node_some(rclcpp::node::Node::SharedPtr node)
{
  this->add_node(node, false);
  spin_some();
  this->remove_node(node, false);
}

void
Executor::spin_some()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_some() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  AnyExecutable::SharedPtr any_exec;
  while ((any_exec = get_next_executable(std::chrono::milliseconds::zero())) && spinning.load()) {
    execute_any_executable(any_exec);
  }
}

void
Executor::spin_once(std::chrono::nanoseconds timeout)
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_once() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  auto any_exec = get_next_executable(timeout);
  if (any_exec) {
    execute_any_executable(any_exec);
  }
}

void
Executor::cancel()
{
  spinning.store(false);
  if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
    throw std::runtime_error(rcl_get_error_string_safe());
  }
}

void
Executor::set_memory_strategy(rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy)
{
  if (memory_strategy == nullptr) {
    throw std::runtime_error("Received NULL memory strategy in executor.");
  }
  memory_strategy_ = memory_strategy;
}

void
Executor::execute_any_executable(AnyExecutable::SharedPtr any_exec)
{
  if (!any_exec || !spinning.load()) {
    return;
  }
  if (any_exec->timer) {
    execute_timer(any_exec->timer);
  }
  if (any_exec->subscription) {
    execute_subscription(any_exec->subscription);
  }
  if (any_exec->subscription_intra_process) {
    execute_intra_process_subscription(any_exec->subscription_intra_process);
  }
  if (any_exec->service) {
    execute_service(any_exec->service);
  }
  if (any_exec->client) {
    execute_client(any_exec->client);
  }
  // Reset the callback_group, regardless of type
  any_exec->callback_group->can_be_taken_from().store(true);
  // Wake the wait, because it may need to be recalculated or work that
  // was previously blocked is now available.
  if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
    throw std::runtime_error(rcl_get_error_string_safe());
  }
}

void
Executor::execute_subscription(
  rclcpp::subscription::SubscriptionBase::SharedPtr subscription)
{
  std::shared_ptr<void> message = subscription->create_message();
  rmw_message_info_t message_info;

  auto ret = rcl_take(subscription->get_subscription_handle(),
      message.get(), &message_info);
  if (ret == RCL_RET_OK) {
    message_info.from_intra_process = false;
    subscription->handle_message(message, message_info);
  } else if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    fprintf(stderr,
      "[rclcpp::error] take failed for subscription on topic '%s': %s\n",
      subscription->get_topic_name().c_str(), rcl_get_error_string_safe());
  }
  subscription->return_message(message);
}

void
Executor::execute_intra_process_subscription(
  rclcpp::subscription::SubscriptionBase::SharedPtr subscription)
{
  rcl_interfaces::msg::IntraProcessMessage ipm;
  rmw_message_info_t message_info;
  rcl_ret_t status = rcl_take(
    subscription->get_intra_process_subscription_handle(),
    &ipm,
    &message_info);

  if (status == RCL_RET_OK) {
    message_info.from_intra_process = true;
    subscription->handle_intra_process_message(ipm, message_info);
  } else if (status != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    fprintf(stderr,
      "[rclcpp::error] take failed for intra process subscription on topic '%s': %s\n",
      subscription->get_topic_name().c_str(), rcl_get_error_string_safe());
  }
}

void
Executor::execute_timer(
  rclcpp::timer::TimerBase::SharedPtr timer)
{
  timer->execute_callback();
}

void
Executor::execute_service(
  rclcpp::service::ServiceBase::SharedPtr service)
{
  auto request_header = service->create_request_header();
  std::shared_ptr<void> request = service->create_request();
  rcl_ret_t status = rcl_take_request(
    service->get_service_handle(),
    request_header.get(),
    request.get());
  if (status != RCL_RET_SERVICE_TAKE_FAILED) {
    if (status == RMW_RET_OK) {
      service->handle_request(request_header, request);
    } else {
      fprintf(stderr,
        "[rclcpp::error] take request failed for server of service '%s': %s\n",
        service->get_service_name().c_str(), rcl_get_error_string_safe());
    }
  }
}

void
Executor::execute_client(
  rclcpp::client::ClientBase::SharedPtr client)
{
  auto request_header = client->create_request_header();
  std::shared_ptr<void> response = client->create_response();
  rcl_ret_t status = rcl_take_response(
    client->get_client_handle(),
    request_header.get(),
    response.get());
  if (status != RCL_RET_SERVICE_TAKE_FAILED) {
    if (status == RMW_RET_OK) {
      client->handle_response(request_header, response);
    } else {
      fprintf(stderr,
        "[rclcpp::error] take response failed for client of service '%s': %s\n",
        client->get_service_name().c_str(), rcl_get_error_string_safe());
    }
  }
}

void
Executor::wait_for_work(std::chrono::nanoseconds timeout)
{
  // Collect the subscriptions and timers to be waited on
  bool has_invalid_weak_nodes = memory_strategy_->collect_entities(weak_nodes_);

  // Clean up any invalid nodes, if they were detected
  if (has_invalid_weak_nodes) {
    weak_nodes_.erase(
      remove_if(
        weak_nodes_.begin(), weak_nodes_.end(),
        // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
        [](std::weak_ptr<rclcpp::node::Node> i)
        {
          return i.expired();
        }
        // *INDENT-ON*
      )
    );
  }

  memory_strategy_->clear_handles();
  if (waitset_.size_of_subscriptions < memory_strategy_->number_of_ready_subscriptions()) {
    rcl_wait_set_resize_subscriptions(
      &waitset_, memory_strategy_->number_of_ready_subscriptions());
  }

  if (waitset_.size_of_services < memory_strategy_->number_of_ready_services()) {
    rcl_wait_set_resize_services(
      &waitset_, memory_strategy_->number_of_ready_services());
  }

  if (waitset_.size_of_clients < memory_strategy_->number_of_ready_clients()) {
    rcl_wait_set_resize_clients(
      &waitset_, memory_strategy_->number_of_ready_clients());
  }

  // TODO Only add the soonest timer to the waitset

  memory_strategy_->add_handles_to_waitset(&waitset_);

  rcl_ret_t status = rcl_wait(&waitset_, timeout.count());
  if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT && status != RCL_RET_WAIT_SET_EMPTY) {
    throw std::runtime_error(rcl_get_error_string_safe());
  }

/*
  rmw_subscriptions_t subscriber_handles;
  subscriber_handles.subscriber_count =
    memory_strategy_->fill_subscriber_handles(subscriber_handles.subscribers);

  rmw_services_t service_handles;
  service_handles.service_count =
    memory_strategy_->fill_service_handles(service_handles.services);

  rmw_clients_t client_handles;
  client_handles.client_count =
    memory_strategy_->fill_client_handles(client_handles.clients);

  // construct a guard conditions struct
  rmw_guard_conditions_t guard_conditions;
  guard_conditions.guard_condition_count =
    memory_strategy_->fill_guard_condition_handles(guard_conditions.guard_conditions);

  rmw_time_t * wait_timeout = NULL;
  rmw_time_t rmw_timeout;

  auto next_timer_duration = get_earliest_timer();
  // If the next timer timeout must preempt the requested timeout
  // or if the requested timeout blocks forever, and there exists a valid timer,
  // replace the requested timeout with the next timeout.
  bool has_valid_timer = next_timer_duration >= std::chrono::nanoseconds::zero();
  if ((next_timer_duration < timeout ||
    timeout < std::chrono::nanoseconds::zero()) && has_valid_timer)
  {
    rmw_timeout.sec =
      std::chrono::duration_cast<std::chrono::seconds>(next_timer_duration).count();
    rmw_timeout.nsec = next_timer_duration.count() % (1000 * 1000 * 1000);
    wait_timeout = &rmw_timeout;
  } else if (timeout >= std::chrono::nanoseconds::zero()) {
    // Convert timeout representation to rmw_time
    rmw_timeout.sec = std::chrono::duration_cast<std::chrono::seconds>(timeout).count();
    rmw_timeout.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count() %
      (1000 * 1000 * 1000);
    wait_timeout = &rmw_timeout;
  }

  // Now wait on the waitable subscriptions and timers
  rmw_ret_t status = rmw_wait(
    &subscriber_handles,
    &guard_conditions,
    &service_handles,
    &client_handles,
    waitset_,
    wait_timeout);
  if (status != RMW_RET_OK && status != RMW_RET_TIMEOUT) {
    throw std::runtime_error(rmw_get_error_string_safe());
  }
*/

  // eh? I think rcl_wait_set does this for us!
  // memory_strategy_->remove_null_handles();
}

rclcpp::node::Node::SharedPtr
Executor::get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  if (!group) {
    return rclcpp::node::Node::SharedPtr();
  }
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto callback_group = weak_group.lock();
      if (callback_group == group) {
        return node;
      }
    }
  }
  return rclcpp::node::Node::SharedPtr();
}

rclcpp::callback_group::CallbackGroup::SharedPtr
Executor::get_group_by_timer(rclcpp::timer::TimerBase::SharedPtr timer)
{
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & weak_timer : group->get_timer_ptrs()) {
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
Executor::get_next_timer(AnyExecutable::SharedPtr any_exec)
{
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      for (auto & timer_ref : group->get_timer_ptrs()) {
        auto timer = timer_ref.lock();
        if (timer && timer->is_ready()) {
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
Executor::get_earliest_timer()
{
  std::chrono::nanoseconds latest = std::chrono::nanoseconds::max();
  bool timers_empty = true;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      for (auto & timer_ref : group->get_timer_ptrs()) {
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

AnyExecutable::SharedPtr
Executor::get_next_ready_executable()
{
  auto any_exec = memory_strategy_->instantiate_next_executable();
  // Check the timers to see if there are any that are ready, if so return
  get_next_timer(any_exec);
  if (any_exec->timer) {
    return any_exec;
  }
  // Check the subscriptions to see if there are any that are ready
  memory_strategy_->get_next_subscription(any_exec, weak_nodes_);
  if (any_exec->subscription || any_exec->subscription_intra_process) {
    return any_exec;
  }
  // Check the services to see if there are any that are ready
  memory_strategy_->get_next_service(any_exec, weak_nodes_);
  if (any_exec->service) {
    return any_exec;
  }
  // Check the clients to see if there are any that are ready
  memory_strategy_->get_next_client(any_exec, weak_nodes_);
  if (any_exec->client) {
    return any_exec;
  }
  // If there is no ready executable, return a null ptr
  return nullptr;
}

AnyExecutable::SharedPtr
Executor::get_next_executable(std::chrono::nanoseconds timeout)
{
  // Check to see if there are any subscriptions or timers needing service
  // TODO(wjwwood): improve run to run efficiency of this function
  auto any_exec = get_next_ready_executable();
  // If there are none
  if (!any_exec) {
    // Wait for subscriptions or timers to work on
    wait_for_work(timeout);
    if (!spinning.load()) {
      return nullptr;
    }
    // Try again
    any_exec = get_next_ready_executable();
  }
  // At this point any_exec should be valid with either a valid subscription
  // or a valid timer, or it should be a null shared_ptr
  if (any_exec) {
    // If it is valid, check to see if the group is mutually exclusive or
    // not, then mark it accordingly
    if (any_exec->callback_group && any_exec->callback_group->type() == \
      callback_group::CallbackGroupType::MutuallyExclusive)
    {
      // It should not have been taken otherwise
      assert(any_exec->callback_group->can_be_taken_from().load());
      // Set to false to indicate something is being run from this group
      // This is reset to true either when the any_exec is executed or when the
      // any_exec is destructued
      any_exec->callback_group->can_be_taken_from().store(false);
    }
  }
  return any_exec;
}

std::ostream &
rclcpp::executor::operator<<(std::ostream & os, const FutureReturnCode & future_return_code)
{
  return os << to_string(future_return_code);
}

std::string
rclcpp::executor::to_string(const FutureReturnCode & future_return_code)
{
  using enum_type = std::underlying_type<FutureReturnCode>::type;
  std::string prefix = "Unknown enum value (";
  std::string ret_as_string = std::to_string(static_cast<enum_type>(future_return_code));
  switch (future_return_code) {
    case FutureReturnCode::SUCCESS:
      prefix = "SUCCESS (";
      break;
    case FutureReturnCode::INTERRUPTED:
      prefix = "INTERRUPTED (";
      break;
    case FutureReturnCode::TIMEOUT:
      prefix = "TIMEOUT (";
      break;
  }
  return prefix + ret_as_string + ")";
}
