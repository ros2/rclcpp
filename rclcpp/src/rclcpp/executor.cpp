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

#include "rclcpp/executor.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "./scope_exit.hpp"

using rclcpp::executor::AnyExecutable;
using rclcpp::executor::Executor;

Executor::Executor(rclcpp::memory_strategy::MemoryStrategy::SharedPtr ms)
: spinning(false), interrupt_guard_condition_(rmw_create_guard_condition()),
  memory_strategy_(ms)
{
}

Executor::~Executor()
{
  // Try to deallocate the interrupt guard condition.
  if (interrupt_guard_condition_ != nullptr) {
    std::lock_guard<std::mutex> lock(trigger_guard_condition_mutex_);
    rmw_ret_t status = rmw_destroy_guard_condition(interrupt_guard_condition_);
    if (status != RMW_RET_OK) {
      fprintf(stderr,
        "[rclcpp::error] failed to destroy guard condition: %s\n", rmw_get_error_string_safe());
    }
  }
}

void
Executor::add_node(rclcpp::node::Node::SharedPtr node_ptr, bool notify)
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
  if (notify) {
    std::lock_guard<std::mutex> lock(trigger_guard_condition_mutex_);
    // Interrupt waiting to handle new node
    rmw_ret_t status = rmw_trigger_guard_condition(interrupt_guard_condition_);
    if (status != RMW_RET_OK) {
      throw std::runtime_error(rmw_get_error_string_safe());
    }
  }
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
  if (notify) {
    // If the node was matched and removed, interrupt waiting
    if (node_removed) {
      std::lock_guard<std::mutex> lock(trigger_guard_condition_mutex_);
      rmw_ret_t status = rmw_trigger_guard_condition(interrupt_guard_condition_);
      if (status != RMW_RET_OK) {
        throw std::runtime_error(rmw_get_error_string_safe());
      }
    }
  }
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
  while ((any_exec = get_next_executable(std::chrono::milliseconds(0))) && spinning.load()) {
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
  std::lock_guard<std::mutex> lock(trigger_guard_condition_mutex_);
  rmw_ret_t status = rmw_trigger_guard_condition(interrupt_guard_condition_);
  if (status != RMW_RET_OK) {
    throw std::runtime_error(rmw_get_error_string_safe());
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
Executor::execute_any_executable(const AnyExecutable::ConstSharedPtr & any_exec)
{
  if (!any_exec || !spinning.load()) {
    return;
  }

  if (timer::TimerBase::ConstSharedPtr timer = any_exec->get_timer()) {
    execute_timer(timer);
  } else if (subscription::SubscriptionBase::ConstSharedPtr subscription = any_exec->get_subscription()) {
    execute_subscription(subscription);
  } else if (subscription::SubscriptionBase::ConstSharedPtr subscription_intra_process = any_exec->get_subscription_intra_process()) {
    execute_intra_process_subscription(subscription_intra_process);
  } else if (any_exec->get_service()) {
    execute_service(any_exec->get_service());
  } else if (any_exec->get_client()) {
    execute_client(any_exec->get_client());
  }
  // Reset the callback_group, regardless of type
  any_exec->get_callback_group()->can_be_taken_from().store(true);
  // Wake the wait, because it may need to be recalculated or work that
  // was previously blocked is now available.
  {
    std::lock_guard<std::mutex> lock(trigger_guard_condition_mutex_);
    rmw_ret_t status = rmw_trigger_guard_condition(interrupt_guard_condition_);
    if (status != RMW_RET_OK) {
      throw std::runtime_error(rmw_get_error_string_safe());
    }
  }
  //assert(any_exec->is_one_field_set());
}

void
Executor::execute_subscription(
  const rclcpp::subscription::SubscriptionBase::ConstSharedPtr subscription)
{
  std::shared_ptr<void> message = subscription->create_message();
  bool taken = false;
  rmw_message_info_t message_info;
  auto ret =
    rmw_take_with_info(subscription->get_subscription_handle(),
      message.get(), &taken, &message_info);
  if (ret == RMW_RET_OK) {
    if (taken) {
      message_info.from_intra_process = false;
      subscription->handle_message(message, message_info);
    }
  } else {
    fprintf(stderr,
      "[rclcpp::error] take failed for subscription on topic '%s': %s\n",
      subscription->get_topic_name().c_str(), rmw_get_error_string_safe());
  }
  subscription->return_message(message);
}

void
Executor::execute_intra_process_subscription(
  const rclcpp::subscription::SubscriptionBase::ConstSharedPtr subscription)
{
  rcl_interfaces::msg::IntraProcessMessage ipm;
  bool taken = false;
  rmw_message_info_t message_info;
  rmw_ret_t status = rmw_take_with_info(
    subscription->get_intra_process_subscription_handle(),
    &ipm,
    &taken,
    &message_info);
  if (status == RMW_RET_OK) {
    if (taken) {
      message_info.from_intra_process = true;
      subscription->handle_intra_process_message(ipm, message_info);
    }
  } else {
    fprintf(stderr,
      "[rclcpp::error] take failed for intra process subscription on topic '%s': %s\n",
      subscription->get_topic_name().c_str(), rmw_get_error_string_safe());
  }
}

void
Executor::execute_timer(
  const rclcpp::timer::TimerBase::ConstSharedPtr timer)
{
  timer->execute_callback();
}

void
Executor::execute_service(
  rclcpp::service::ServiceBase::SharedPtr service)
{
  std::shared_ptr<void> request_header = service->create_request_header();
  std::shared_ptr<void> request = service->create_request();
  bool taken = false;
  rmw_ret_t status = rmw_take_request(
    service->get_service_handle(),
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

void
Executor::execute_client(
  rclcpp::client::ClientBase::SharedPtr client)
{
  std::shared_ptr<void> request_header = client->create_request_header();
  std::shared_ptr<void> response = client->create_response();
  bool taken = false;
  rmw_ret_t status = rmw_take_response(
    client->get_client_handle(),
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

void
Executor::wait_for_work(std::chrono::nanoseconds timeout)
{
  memory_strategy_->clear_active_entities();

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

  // Use the number of subscriptions to allocate memory in the handles
  rmw_subscriptions_t subscriber_handles;
  subscriber_handles.subscriber_count =
    memory_strategy_->fill_subscriber_handles(subscriber_handles.subscribers);

  rmw_services_t service_handles;
  service_handles.service_count =
    memory_strategy_->fill_service_handles(service_handles.services);

  rmw_clients_t client_handles;
  client_handles.client_count =
    memory_strategy_->fill_client_handles(client_handles.clients);

  // The number of guard conditions is fixed at 2: 1 for the ctrl-c guard cond,
  // and one for the executor's guard cond (interrupt_guard_condition_)
  size_t number_of_guard_conds = 2;
  rmw_guard_conditions_t guard_condition_handles;
  guard_condition_handles.guard_condition_count = number_of_guard_conds;
  guard_condition_handles.guard_conditions = static_cast<void **>(guard_cond_handles_.data());
  if (guard_condition_handles.guard_conditions == NULL &&
    number_of_guard_conds > 0)
  {
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
    memory_strategy_->clear_handles();
    return;
  }

  memory_strategy_->remove_null_handles();
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

bool
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
        if (timer && timer->check_and_trigger()) {
          any_exec->set_timer(timer);
          any_exec->set_callback_group(group);
          any_exec->set_node(get_node_by_group(group));
          return true;
        }
      }
    }
  }
  return false;
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
  assert(any_exec->is_one_field_set());

  // Check the timers to see if there are any that are ready, if so return
  if (get_next_timer(any_exec)) {
    return any_exec;
  }
  // Check the subscriptions to see if there are any that are ready
  if (memory_strategy_->get_next_subscription(any_exec, weak_nodes_)) {
    return any_exec;
  }
  // Check the services to see if there are any that are ready
  if (memory_strategy_->get_next_service(any_exec, weak_nodes_)) {
    return any_exec;
  }
  // Check the clients to see if there are any that are ready
  if (memory_strategy_->get_next_client(any_exec, weak_nodes_)) {
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
    if (any_exec->get_callback_group() && any_exec->get_callback_group()->type() == \
      callback_group::CallbackGroupType::MutuallyExclusive)
    {
      // It should not have been taken otherwise
      //assert(any_exec->callback_group->can_be_taken_from().load());
      // Set to false to indicate something is being run from this group
      // This is reset to true either when the any_exec is executed or when the
      // any_exec is destructued
      any_exec->get_callback_group()->can_be_taken_from().store(false);
    }
  }
  return any_exec;
}
