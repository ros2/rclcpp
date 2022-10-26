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
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rcl/allocator.h"
#include "rcl/error_handling.h"
#include "rcpputils/scope_exit.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include "rcutils/logging_macros.h"

#include "tracetools/tracetools.h"

using namespace std::chrono_literals;

using rclcpp::exceptions::throw_from_rcl_error;
using rclcpp::AnyExecutable;
using rclcpp::Executor;
using rclcpp::ExecutorOptions;
using rclcpp::FutureReturnCode;

Executor::Executor(const rclcpp::ExecutorOptions & options)
: spinning(false),
  interrupt_guard_condition_(options.context),
  shutdown_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context)),
  memory_strategy_(options.memory_strategy)
{
  // Store the context for later use.
  context_ = options.context;

  shutdown_callback_handle_ = context_->add_on_shutdown_callback(
    [weak_gc = std::weak_ptr<rclcpp::GuardCondition>{shutdown_guard_condition_}]() {
      auto strong_gc = weak_gc.lock();
      if (strong_gc) {
        strong_gc->trigger();
      }
    });

  // The number of guard conditions is always at least 2: 1 for the ctrl-c guard cond,
  // and one for the executor's guard cond (interrupt_guard_condition_)
  memory_strategy_->add_guard_condition(*shutdown_guard_condition_.get());

  // Put the executor's guard condition in
  memory_strategy_->add_guard_condition(interrupt_guard_condition_);
  rcl_allocator_t allocator = memory_strategy_->get_allocator();

  rcl_ret_t ret = rcl_wait_set_init(
    &wait_set_,
    0, 2, 0, 0, 0, 0,
    context_->get_rcl_context().get(),
    allocator);
  if (RCL_RET_OK != ret) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to create wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
    throw_from_rcl_error(ret, "Failed to create wait set in Executor constructor");
  }
}

Executor::~Executor()
{
  // Disassociate all callback groups.
  for (auto & pair : weak_groups_to_nodes_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool & has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  // Disassociate all nodes.
  std::for_each(
    weak_nodes_.begin(), weak_nodes_.end(), []
      (rclcpp::node_interfaces::NodeBaseInterface::WeakPtr weak_node_ptr) {
      auto shared_node_ptr = weak_node_ptr.lock();
      if (shared_node_ptr) {
        std::atomic_bool & has_executor = shared_node_ptr->get_associated_with_executor_atomic();
        has_executor.store(false);
      }
    });
  weak_nodes_.clear();
  weak_groups_associated_with_executor_to_nodes_.clear();
  weak_groups_to_nodes_associated_with_executor_.clear();
  weak_groups_to_nodes_.clear();
  for (const auto & pair : weak_groups_to_guard_conditions_) {
    auto guard_condition = pair.second;
    memory_strategy_->remove_guard_condition(guard_condition);
  }
  weak_groups_to_guard_conditions_.clear();

  // Finalize the wait set.
  if (rcl_wait_set_fini(&wait_set_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to destroy wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
  // Remove and release the sigint guard condition
  memory_strategy_->remove_guard_condition(shutdown_guard_condition_.get());
  memory_strategy_->remove_guard_condition(&interrupt_guard_condition_);

  // Remove shutdown callback handle registered to Context
  if (!context_->remove_on_shutdown_callback(shutdown_callback_handle_)) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to remove registered on_shutdown callback");
    rcl_reset_error();
  }
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
Executor::get_all_callback_groups()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  std::lock_guard<std::mutex> guard{mutex_};
  for (const auto & group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(group_node_ptr.first);
  }
  for (auto const & group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
Executor::get_manually_added_callback_groups()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  std::lock_guard<std::mutex> guard{mutex_};
  for (auto const & group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
Executor::get_automatically_added_callback_groups_from_nodes()
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  std::lock_guard<std::mutex> guard{mutex_};
  for (auto const & group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

void
Executor::add_callback_groups_from_nodes_associated_to_executor()
{
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      node->for_each_callback_group(
        [this, node](rclcpp::CallbackGroup::SharedPtr shared_group_ptr)
        {
          if (
            shared_group_ptr->automatically_add_to_executor_with_node() &&
            !shared_group_ptr->get_associated_with_executor_atomic().load())
          {
            this->add_callback_group_to_map(
              shared_group_ptr,
              node,
              weak_groups_to_nodes_associated_with_executor_,
              true);
          }
        });
    }
  }
}

void
Executor::add_callback_group_to_map(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap & weak_groups_to_nodes,
  bool notify)
{
  // If the callback_group already has an executor
  std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }

  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto insert_info =
    weak_groups_to_nodes.insert(std::make_pair(weak_group_ptr, node_ptr));
  bool was_inserted = insert_info.second;
  if (!was_inserted) {
    throw std::runtime_error("Callback group was already added to executor.");
  }
  // Also add to the map that contains all callback groups
  weak_groups_to_nodes_.insert(std::make_pair(weak_group_ptr, node_ptr));

  if (node_ptr->get_context()->is_valid()) {
    auto callback_group_guard_condition =
      group_ptr->get_notify_guard_condition(node_ptr->get_context());
    weak_groups_to_guard_conditions_[weak_group_ptr] = callback_group_guard_condition.get();
    // Add the callback_group's notify condition to the guard condition handles
    memory_strategy_->add_guard_condition(*callback_group_guard_condition);
  }

  if (notify) {
    // Interrupt waiting to handle new node
    try {
      interrupt_guard_condition_.trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on callback group add: ") + ex.what());
    }
  }
}

void
Executor::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  bool notify)
{
  std::lock_guard<std::mutex> guard{mutex_};
  this->add_callback_group_to_map(
    group_ptr,
    node_ptr,
    weak_groups_associated_with_executor_to_nodes_,
    notify);
}

void
Executor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error(
            std::string("Node '") + node_ptr->get_fully_qualified_name() +
            "' has already been added to an executor.");
  }
  std::lock_guard<std::mutex> guard{mutex_};
  node_ptr->for_each_callback_group(
    [this, node_ptr, notify](rclcpp::CallbackGroup::SharedPtr group_ptr)
    {
      if (!group_ptr->get_associated_with_executor_atomic().load() &&
      group_ptr->automatically_add_to_executor_with_node())
      {
        this->add_callback_group_to_map(
          group_ptr,
          node_ptr,
          weak_groups_to_nodes_associated_with_executor_,
          notify);
      }
    });

  weak_nodes_.push_back(node_ptr);
}

void
Executor::remove_callback_group_from_map(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap & weak_groups_to_nodes,
  bool notify)
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
    weak_groups_to_nodes_.erase(group_ptr);
    std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
    has_executor.store(false);
  } else {
    throw std::runtime_error("Callback group needs to be associated with executor.");
  }
  // If the node was matched and removed, interrupt waiting.
  if (!has_node(node_ptr, weak_groups_to_nodes_associated_with_executor_) &&
    !has_node(node_ptr, weak_groups_associated_with_executor_to_nodes_))
  {
    auto iter = weak_groups_to_guard_conditions_.find(weak_group_ptr);
    if (iter != weak_groups_to_guard_conditions_.end()) {
      memory_strategy_->remove_guard_condition(iter->second);
    }
    weak_groups_to_guard_conditions_.erase(weak_group_ptr);

    if (notify) {
      try {
        interrupt_guard_condition_.trigger();
      } catch (const rclcpp::exceptions::RCLError & ex) {
        throw std::runtime_error(
                std::string(
                  "Failed to trigger guard condition on callback group remove: ") + ex.what());
      }
    }
  }
}

void
Executor::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  bool notify)
{
  std::lock_guard<std::mutex> guard{mutex_};
  this->remove_callback_group_from_map(
    group_ptr,
    weak_groups_associated_with_executor_to_nodes_,
    notify);
}

void
Executor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

void
Executor::remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  if (!node_ptr->get_associated_with_executor_atomic().load()) {
    throw std::runtime_error("Node needs to be associated with an executor.");
  }

  std::lock_guard<std::mutex> guard{mutex_};
  bool found_node = false;
  auto node_it = weak_nodes_.begin();
  while (node_it != weak_nodes_.end()) {
    bool matched = (node_it->lock() == node_ptr);
    if (matched) {
      found_node = true;
      node_it = weak_nodes_.erase(node_it);
    } else {
      ++node_it;
    }
  }
  if (!found_node) {
    throw std::runtime_error("Node needs to be associated with this executor.");
  }

  for (auto it = weak_groups_to_nodes_associated_with_executor_.begin();
    it != weak_groups_to_nodes_associated_with_executor_.end(); )
  {
    auto weak_node_ptr = it->second;
    auto shared_node_ptr = weak_node_ptr.lock();
    auto group_ptr = it->first.lock();

    // Increment iterator before removing in case it's invalidated
    it++;
    if (shared_node_ptr == node_ptr) {
      remove_callback_group_from_map(
        group_ptr,
        weak_groups_to_nodes_associated_with_executor_,
        notify);
    }
  }

  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
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

void Executor::spin_some(std::chrono::nanoseconds max_duration)
{
  return this->spin_some_impl(max_duration, false);
}

void Executor::spin_all(std::chrono::nanoseconds max_duration)
{
  if (max_duration < 0ns) {
    throw std::invalid_argument("max_duration must be greater than or equal to 0");
  }
  return this->spin_some_impl(max_duration, true);
}

void
Executor::spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive)
{
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
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  bool work_available = false;
  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    AnyExecutable any_exec;
    if (!work_available) {
      wait_for_work(std::chrono::milliseconds::zero());
    }
    if (get_next_ready_executable(any_exec)) {
      execute_any_executable(any_exec);
      work_available = true;
    } else {
      if (!work_available || !exhaustive) {
        break;
      }
      work_available = false;
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
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_once() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  spin_once_impl(timeout);
}

void
Executor::cancel()
{
  spinning.store(false);
  try {
    interrupt_guard_condition_.trigger();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("Failed to trigger guard condition in cancel: ") + ex.what());
  }
}

void
Executor::set_memory_strategy(rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy)
{
  if (memory_strategy == nullptr) {
    throw std::runtime_error("Received NULL memory strategy in executor.");
  }
  std::lock_guard<std::mutex> guard{mutex_};
  memory_strategy_ = memory_strategy;
}

void
Executor::execute_any_executable(AnyExecutable & any_exec)
{
  if (!spinning.load()) {
    return;
  }
  if (any_exec.timer) {
    TRACEPOINT(
      rclcpp_executor_execute,
      static_cast<const void *>(any_exec.timer->get_timer_handle().get()));
    execute_timer(any_exec.timer);
  }
  if (any_exec.subscription) {
    TRACEPOINT(
      rclcpp_executor_execute,
      static_cast<const void *>(any_exec.subscription->get_subscription_handle().get()));
    execute_subscription(any_exec.subscription);
  }
  if (any_exec.service) {
    execute_service(any_exec.service);
  }
  if (any_exec.client) {
    execute_client(any_exec.client);
  }
  if (any_exec.waitable) {
    any_exec.waitable->execute(any_exec.data);
  }
  // Reset the callback_group, regardless of type
  any_exec.callback_group->can_be_taken_from().store(true);
  // Wake the wait, because it may need to be recalculated or work that
  // was previously blocked is now available.
  try {
    interrupt_guard_condition_.trigger();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string(
              "Failed to trigger guard condition from execute_any_executable: ") + ex.what());
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
        subscription->handle_serialized_message(serialized_msg, message_info);
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
    if (nullptr != loaned_msg) {
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
    }
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
  TRACEPOINT(rclcpp_executor_wait_for_work, timeout.count());
  {
    std::lock_guard<std::mutex> guard(mutex_);

    // Check weak_nodes_ to find any callback group that is not owned
    // by an executor and add it to the list of callbackgroups for
    // collect entities. Also exchange to false so it is not
    // allowed to add to another executor
    add_callback_groups_from_nodes_associated_to_executor();

    // Collect the subscriptions and timers to be waited on
    memory_strategy_->clear_handles();
    bool has_invalid_weak_groups_or_nodes =
      memory_strategy_->collect_entities(weak_groups_to_nodes_);

    if (has_invalid_weak_groups_or_nodes) {
      std::vector<rclcpp::CallbackGroup::WeakPtr> invalid_group_ptrs;
      for (auto pair : weak_groups_to_nodes_) {
        auto weak_group_ptr = pair.first;
        auto weak_node_ptr = pair.second;
        if (weak_group_ptr.expired() || weak_node_ptr.expired()) {
          invalid_group_ptrs.push_back(weak_group_ptr);
        }
      }
      std::for_each(
        invalid_group_ptrs.begin(), invalid_group_ptrs.end(),
        [this](rclcpp::CallbackGroup::WeakPtr group_ptr) {
          if (weak_groups_to_nodes_associated_with_executor_.find(group_ptr) !=
          weak_groups_to_nodes_associated_with_executor_.end())
          {
            weak_groups_to_nodes_associated_with_executor_.erase(group_ptr);
          }
          if (weak_groups_associated_with_executor_to_nodes_.find(group_ptr) !=
          weak_groups_associated_with_executor_to_nodes_.end())
          {
            weak_groups_associated_with_executor_to_nodes_.erase(group_ptr);
          }
          auto callback_guard_pair = weak_groups_to_guard_conditions_.find(group_ptr);
          if (callback_guard_pair != weak_groups_to_guard_conditions_.end()) {
            auto guard_condition = callback_guard_pair->second;
            weak_groups_to_guard_conditions_.erase(group_ptr);
            memory_strategy_->remove_guard_condition(guard_condition);
          }
          weak_groups_to_nodes_.erase(group_ptr);
        });
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
  std::lock_guard<std::mutex> guard(mutex_);
  memory_strategy_->remove_null_handles(&wait_set_);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
Executor::get_node_by_group(
  const rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &
  weak_groups_to_nodes,
  rclcpp::CallbackGroup::SharedPtr group)
{
  if (!group) {
    return nullptr;
  }
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr(group);
  const auto finder = weak_groups_to_nodes.find(weak_group_ptr);
  if (finder != weak_groups_to_nodes.end()) {
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr = finder->second.lock();
    return node_ptr;
  }
  return nullptr;
}

rclcpp::CallbackGroup::SharedPtr
Executor::get_group_by_timer(rclcpp::TimerBase::SharedPtr timer)
{
  std::lock_guard<std::mutex> guard{mutex_};
  for (const auto & pair : weak_groups_associated_with_executor_to_nodes_) {
    auto group = pair.first.lock();
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

  for (const auto & pair : weak_groups_to_nodes_associated_with_executor_) {
    auto group = pair.first.lock();
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
  return nullptr;
}

bool
Executor::get_next_ready_executable(AnyExecutable & any_executable)
{
  bool success = get_next_ready_executable_from_map(any_executable, weak_groups_to_nodes_);
  return success;
}

bool
Executor::get_next_ready_executable_from_map(
  AnyExecutable & any_executable,
  const rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &
  weak_groups_to_nodes)
{
  TRACEPOINT(rclcpp_executor_get_next_ready);
  bool success = false;
  std::lock_guard<std::mutex> guard{mutex_};
  // Check the timers to see if there are any that are ready
  memory_strategy_->get_next_timer(any_executable, weak_groups_to_nodes);
  if (any_executable.timer) {
    success = true;
  }
  if (!success) {
    // Check the subscriptions to see if there are any that are ready
    memory_strategy_->get_next_subscription(any_executable, weak_groups_to_nodes);
    if (any_executable.subscription) {
      success = true;
    }
  }
  if (!success) {
    // Check the services to see if there are any that are ready
    memory_strategy_->get_next_service(any_executable, weak_groups_to_nodes);
    if (any_executable.service) {
      success = true;
    }
  }
  if (!success) {
    // Check the clients to see if there are any that are ready
    memory_strategy_->get_next_client(any_executable, weak_groups_to_nodes);
    if (any_executable.client) {
      success = true;
    }
  }
  if (!success) {
    // Check the waitables to see if there are any that are ready
    memory_strategy_->get_next_waitable(any_executable, weak_groups_to_nodes);
    if (any_executable.waitable) {
      any_executable.data = any_executable.waitable->take_data();
      success = true;
    }
  }
  // At this point any_executable should be valid with either a valid subscription
  // or a valid timer, or it should be a null shared_ptr
  if (success) {
    rclcpp::CallbackGroup::WeakPtr weak_group_ptr = any_executable.callback_group;
    auto iter = weak_groups_to_nodes.find(weak_group_ptr);
    if (iter == weak_groups_to_nodes.end()) {
      success = false;
    }
  }

  if (success) {
    // If it is valid, check to see if the group is mutually exclusive or
    // not, then mark it accordingly ..Check if the callback_group belongs to this executor
    if (any_executable.callback_group && any_executable.callback_group->type() == \
      CallbackGroupType::MutuallyExclusive)
    {
      // It should not have been taken otherwise
      assert(any_executable.callback_group->can_be_taken_from().load());
      // Set to false to indicate something is being run from this group
      // This is reset to true either when the any_executable is executed or when the
      // any_executable is destructued
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

// Returns true iff the weak_groups_to_nodes map has node_ptr as the value in any of its entry.
bool
Executor::has_node(
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

bool
Executor::is_spinning()
{
  return spinning;
}
