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
#include <memory>
#include <string>
#include <type_traits>

#include "rcl/allocator.h"
#include "rcl/error_handling.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/static_single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rclcpp/utilities.hpp"

#include "rcutils/logging_macros.h"

using rclcpp::exceptions::throw_from_rcl_error;
using rclcpp::AnyExecutable;
using rclcpp::Executor;
using rclcpp::ExecutorOptions;
using rclcpp::FutureReturnCode;

Executor::Executor(const rclcpp::ExecutorOptions & options)
: spinning(false),
  memory_strategy_(options.memory_strategy)
{
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_ret_t ret = rcl_guard_condition_init(
    &interrupt_guard_condition_, options.context->get_rcl_context().get(), guard_condition_options);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "Failed to create interrupt guard condition in Executor constructor");
  }

  // The number of guard conditions is always at least 2: 1 for the ctrl-c guard cond,
  // and one for the executor's guard cond (interrupt_guard_condition_)

  // Put the global ctrl-c guard condition in
  memory_strategy_->add_guard_condition(options.context->get_interrupt_guard_condition(&wait_set_));

  // Put the executor's guard condition in
  memory_strategy_->add_guard_condition(&interrupt_guard_condition_);
  rcl_allocator_t allocator = memory_strategy_->get_allocator();

  // Store the context for later use.
  context_ = options.context;

  ret = rcl_wait_set_init(
    &wait_set_,
    0, 2, 0, 0, 0, 0,
    context_->get_rcl_context().get(),
    allocator);
  if (RCL_RET_OK != ret) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to create wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
    if (rcl_guard_condition_fini(&interrupt_guard_condition_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "failed to destroy guard condition: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
    throw_from_rcl_error(ret, "Failed to create wait set in Executor constructor");
  }
}

Executor::~Executor()
{
  // Disassocate all nodes
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      std::atomic_bool & has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  weak_nodes_.clear();
  for (auto & guard_condition : guard_conditions_) {
    memory_strategy_->remove_guard_condition(guard_condition);
  }
  guard_conditions_.clear();

  // Finalize the wait set.
  if (rcl_wait_set_fini(&wait_set_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to destroy wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
  // Finalize the interrupt guard condition.
  if (rcl_guard_condition_fini(&interrupt_guard_condition_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to destroy guard condition: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
  // Remove and release the sigint guard condition
  memory_strategy_->remove_guard_condition(context_->get_interrupt_guard_condition(&wait_set_));
  context_->release_interrupt_guard_condition(&wait_set_, std::nothrow);
}

void
Executor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
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
  guard_conditions_.push_back(node_ptr->get_notify_guard_condition());
  if (notify) {
    // Interrupt waiting to handle new node
    if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
      throw std::runtime_error(rcl_get_error_string().str);
    }
  }
  // Add the node's notify condition to the guard condition handles
  std::unique_lock<std::mutex> lock(memory_strategy_mutex_);
  memory_strategy_->add_guard_condition(node_ptr->get_notify_guard_condition());
}

void
Executor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

void
Executor::remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  bool node_removed = false;
  {
    auto node_it = weak_nodes_.begin();
    auto gc_it = guard_conditions_.begin();
    while (node_it != weak_nodes_.end()) {
      bool matched = (node_it->lock() == node_ptr);
      if (matched) {
        node_it = weak_nodes_.erase(node_it);
        gc_it = guard_conditions_.erase(gc_it);
        node_removed = true;
      } else {
        ++node_it;
        ++gc_it;
      }
    }
  }
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
  if (notify) {
    // If the node was matched and removed, interrupt waiting
    if (node_removed) {
      if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
        throw std::runtime_error(rcl_get_error_string().str);
      }
    }
  }
  std::unique_lock<std::mutex> lock(memory_strategy_mutex_);
  memory_strategy_->remove_guard_condition(node_ptr->get_notify_guard_condition());
}

void
Executor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

void
Executor::spin_node_once_nanoseconds(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
  std::chrono::nanoseconds timeout)
{
  this->add_node(node, false);
  // non-blocking = true
  spin_once(timeout);
  this->remove_node(node, false);
}

void
Executor::spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
  this->add_node(node, false);
  spin_some();
  this->remove_node(node, false);
}

void
Executor::spin_node_some(std::shared_ptr<rclcpp::Node> node)
{
  this->spin_node_some(node->get_node_base_interface());
}

void
Executor::spin_some(std::chrono::nanoseconds max_duration)
{
  if (nullptr != dynamic_cast<executors::StaticSingleThreadedExecutor *>(this)) {
    throw rclcpp::exceptions::UnimplementedError(
            "spin_some is not implemented for StaticSingleThreadedExecutor, use spin or "
            "spin_until_future_complete");
  }

  auto start = std::chrono::steady_clock::now();
  auto max_duration_not_elapsed = [max_duration, start]() {
      if (std::chrono::nanoseconds(0) == max_duration) {
        // told to spin forever if need be
        return true;
      } else if (std::chrono::steady_clock::now() - start < max_duration) {
        // told to spin only for some maximum amount of time
        return true;
      }
      // spun too long
      return false;
    };

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_some() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  // non-blocking call to pre-load all available work
  wait_for_work(std::chrono::milliseconds::zero());
  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    AnyExecutable any_exec;
    if (get_next_ready_executable(any_exec)) {
      execute_any_executable(any_exec);
    } else {
      break;
    }
  }
}

void
Executor::spin_once_impl(std::chrono::nanoseconds timeout)
{
  AnyExecutable any_exec;
  if (get_next_executable(any_exec, timeout)) {
    execute_any_executable(any_exec);
  }
}

void
Executor::spin_once(std::chrono::nanoseconds timeout)
{
  if (nullptr != dynamic_cast<executors::StaticSingleThreadedExecutor *>(this)) {
    throw rclcpp::exceptions::UnimplementedError(
            "spin_once is not implemented for StaticSingleThreadedExecutor, use spin or "
            "spin_until_future_complete");
  }
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_once() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  spin_once_impl(timeout);
}

void
Executor::cancel()
{
  spinning.store(false);
  rcl_ret_t ret = rcl_trigger_guard_condition(&interrupt_guard_condition_);
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "Failed to trigger guard condition in cancel");
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
Executor::execute_any_executable(AnyExecutable & any_exec)
{
  if (!spinning.load()) {
    return;
  }
  if (any_exec.timer) {
    execute_timer(any_exec.timer);
  }
  if (any_exec.subscription) {
    execute_subscription(any_exec.subscription);
  }
  if (any_exec.service) {
    execute_service(any_exec.service);
  }
  if (any_exec.client) {
    execute_client(any_exec.client);
  }
  if (any_exec.waitable) {
    any_exec.waitable->execute();
  }
  // Reset the callback_group, regardless of type
  any_exec.callback_group->can_be_taken_from().store(true);
  // Wake the wait, because it may need to be recalculated or work that
  // was previously blocked is now available.
  rcl_ret_t ret = rcl_trigger_guard_condition(&interrupt_guard_condition_);
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "Failed to trigger guard condition from execute_any_executable");
  }
}

static
void
take_and_do_error_handling(
  const char * action_description,
  const char * topic_or_service_name,
  std::function<bool()> take_action,
  std::function<void()> handle_action)
{
  bool taken = false;
  try {
    taken = take_action();
  } catch (const rclcpp::exceptions::RCLError & rcl_error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"),
      "executor %s '%s' unexpectedly failed: %s",
      action_description,
      topic_or_service_name,
      rcl_error.what());
  }
  if (taken) {
    handle_action();
  } else {
    // Message or Service was not taken for some reason.
    // Note that this can be normal, if the underlying middleware needs to
    // interrupt wait spuriously it is allowed.
    // So in that case the executor cannot tell the difference in a
    // spurious wake up and an entity actually having data until trying
    // to take the data.
    RCLCPP_DEBUG(
      rclcpp::get_logger("rclcpp"),
      "executor %s '%s' failed to take anything",
      action_description,
      topic_or_service_name);
  }
}

void
Executor::execute_subscription(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::MessageInfo message_info;
  message_info.get_rmw_message_info().from_intra_process = false;

  if (subscription->is_serialized()) {
    // This is the case where a copy of the serialized message is taken from
    // the middleware via inter-process communication.
    std::shared_ptr<SerializedMessage> serialized_msg = subscription->create_serialized_message();
    take_and_do_error_handling(
      "taking a serialized message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_serialized(*serialized_msg.get(), message_info);},
      [&]()
      {
        auto void_serialized_msg = std::static_pointer_cast<void>(serialized_msg);
        subscription->handle_message(void_serialized_msg, message_info);
      });
    subscription->return_serialized_message(serialized_msg);
  } else if (subscription->can_loan_messages()) {
    // This is the case where a loaned message is taken from the middleware via
    // inter-process communication, given to the user for their callback,
    // and then returned.
    void * loaned_msg = nullptr;
    // TODO(wjwwood): refactor this into methods on subscription when LoanedMessage
    //   is extened to support subscriptions as well.
    take_and_do_error_handling(
      "taking a loaned message from topic",
      subscription->get_topic_name(),
      [&]()
      {
        rcl_ret_t ret = rcl_take_loaned_message(
          subscription->get_subscription_handle().get(),
          &loaned_msg,
          &message_info.get_rmw_message_info(),
          nullptr);
        if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
          return false;
        } else if (RCL_RET_OK != ret) {
          rclcpp::exceptions::throw_from_rcl_error(ret);
        }
        return true;
      },
      [&]() {subscription->handle_loaned_message(loaned_msg, message_info);});
    rcl_ret_t ret = rcl_return_loaned_message_from_subscription(
      subscription->get_subscription_handle().get(),
      loaned_msg);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "rcl_return_loaned_message_from_subscription() failed for subscription on topic '%s': %s",
        subscription->get_topic_name(), rcl_get_error_string().str);
    }
    loaned_msg = nullptr;
  } else {
    // This case is taking a copy of the message data from the middleware via
    // inter-process communication.
    std::shared_ptr<void> message = subscription->create_message();
    take_and_do_error_handling(
      "taking a message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_type_erased(message.get(), message_info);},
      [&]() {subscription->handle_message(message, message_info);});
    subscription->return_message(message);
  }
}

void
Executor::execute_timer(rclcpp::TimerBase::SharedPtr timer)
{
  timer->execute_callback();
}

void
Executor::execute_service(rclcpp::ServiceBase::SharedPtr service)
{
  auto request_header = service->create_request_header();
  std::shared_ptr<void> request = service->create_request();
  take_and_do_error_handling(
    "taking a service server request from service",
    service->get_service_name(),
    [&]() {return service->take_type_erased_request(request.get(), *request_header);},
    [&]() {service->handle_request(request_header, request);});
}

void
Executor::execute_client(
  rclcpp::ClientBase::SharedPtr client)
{
  auto request_header = client->create_request_header();
  std::shared_ptr<void> response = client->create_response();
  take_and_do_error_handling(
    "taking a service client response from service",
    client->get_service_name(),
    [&]() {return client->take_type_erased_response(response.get(), *request_header);},
    [&]() {client->handle_response(request_header, response);});
}

void
Executor::wait_for_work(std::chrono::nanoseconds timeout)
{
  {
    std::unique_lock<std::mutex> lock(memory_strategy_mutex_);

    // Collect the subscriptions and timers to be waited on
    memory_strategy_->clear_handles();
    bool has_invalid_weak_nodes = memory_strategy_->collect_entities(weak_nodes_);

    // Clean up any invalid nodes, if they were detected
    if (has_invalid_weak_nodes) {
      auto node_it = weak_nodes_.begin();
      auto gc_it = guard_conditions_.begin();
      while (node_it != weak_nodes_.end()) {
        if (node_it->expired()) {
          node_it = weak_nodes_.erase(node_it);
          memory_strategy_->remove_guard_condition(*gc_it);
          gc_it = guard_conditions_.erase(gc_it);
        } else {
          ++node_it;
          ++gc_it;
        }
      }
    }
    // clear wait set
    rcl_ret_t ret = rcl_wait_set_clear(&wait_set_);
    if (ret != RCL_RET_OK) {
      throw_from_rcl_error(ret, "Couldn't clear wait set");
    }

    // The size of waitables are accounted for in size of the other entities
    ret = rcl_wait_set_resize(
      &wait_set_, memory_strategy_->number_of_ready_subscriptions(),
      memory_strategy_->number_of_guard_conditions(), memory_strategy_->number_of_ready_timers(),
      memory_strategy_->number_of_ready_clients(), memory_strategy_->number_of_ready_services(),
      memory_strategy_->number_of_ready_events());
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "Couldn't resize the wait set");
    }

    if (!memory_strategy_->add_handles_to_wait_set(&wait_set_)) {
      throw std::runtime_error("Couldn't fill wait set");
    }
  }
  rcl_ret_t status =
    rcl_wait(&wait_set_, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());
  if (status == RCL_RET_WAIT_SET_EMPTY) {
    RCUTILS_LOG_WARN_NAMED(
      "rclcpp",
      "empty wait set received in rcl_wait(). This should never happen.");
  } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(status, "rcl_wait() failed");
  }

  // check the null handles in the wait set and remove them from the handles in memory strategy
  // for callback-based entities
  memory_strategy_->remove_null_handles(&wait_set_);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
Executor::get_node_by_group(rclcpp::CallbackGroup::SharedPtr group)
{
  if (!group) {
    return nullptr;
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
  return nullptr;
}

rclcpp::CallbackGroup::SharedPtr
Executor::get_group_by_timer(rclcpp::TimerBase::SharedPtr timer)
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
      auto timer_ref = group->find_timer_ptrs_if(
        [timer](const rclcpp::TimerBase::SharedPtr & timer_ptr) -> bool {
          return timer_ptr == timer;
        });
      if (timer_ref) {
        return group;
      }
    }
  }
  return rclcpp::CallbackGroup::SharedPtr();
}

bool
Executor::get_next_ready_executable(AnyExecutable & any_executable)
{
  bool success = false;
  // Check the timers to see if there are any that are ready
  memory_strategy_->get_next_timer(any_executable, weak_nodes_);
  if (any_executable.timer) {
    success = true;
  }
  if (!success) {
    // Check the subscriptions to see if there are any that are ready
    memory_strategy_->get_next_subscription(any_executable, weak_nodes_);
    if (any_executable.subscription) {
      success = true;
    }
  }
  if (!success) {
    // Check the services to see if there are any that are ready
    memory_strategy_->get_next_service(any_executable, weak_nodes_);
    if (any_executable.service) {
      success = true;
    }
  }
  if (!success) {
    // Check the clients to see if there are any that are ready
    memory_strategy_->get_next_client(any_executable, weak_nodes_);
    if (any_executable.client) {
      success = true;
    }
  }
  if (!success) {
    // Check the waitables to see if there are any that are ready
    memory_strategy_->get_next_waitable(any_executable, weak_nodes_);
    if (any_executable.waitable) {
      success = true;
    }
  }
  // At this point any_exec should be valid with either a valid subscription
  // or a valid timer, or it should be a null shared_ptr
  if (success) {
    // If it is valid, check to see if the group is mutually exclusive or
    // not, then mark it accordingly
    using callback_group::CallbackGroupType;
    if (
      any_executable.callback_group &&
      any_executable.callback_group->type() == CallbackGroupType::MutuallyExclusive)
    {
      // It should not have been taken otherwise
      assert(any_executable.callback_group->can_be_taken_from().load());
      // Set to false to indicate something is being run from this group
      // This is reset to true either when the any_exec is executed or when the
      // any_exec is destructued
      any_executable.callback_group->can_be_taken_from().store(false);
    }
  }
  // If there is no ready executable, return false
  return success;
}

bool
Executor::get_next_executable(AnyExecutable & any_executable, std::chrono::nanoseconds timeout)
{
  bool success = false;
  // Check to see if there are any subscriptions or timers needing service
  // TODO(wjwwood): improve run to run efficiency of this function
  success = get_next_ready_executable(any_executable);
  // If there are none
  if (!success) {
    // Wait for subscriptions or timers to work on
    wait_for_work(timeout);
    if (!spinning.load()) {
      return false;
    }
    // Try again
    success = get_next_ready_executable(any_executable);
  }
  return success;
}
