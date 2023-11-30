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
#include <chrono>
#include <iterator>
#include <memory>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rcl/allocator.h"
#include "rcl/error_handling.h"
#include "rclcpp/executors/executor_notify_waitable.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rclcpp/dynamic_typesupport/dynamic_message.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include "rcutils/logging_macros.h"

#include "tracetools/tracetools.h"

using namespace std::chrono_literals;

using rclcpp::Executor;

/// Mask to indicate to the waitset to only add the subscription.
/// The events and intraprocess waitable are already added via the callback group.
static constexpr rclcpp::SubscriptionWaitSetMask kDefaultSubscriptionMask = {true, false, false};

class rclcpp::ExecutorImplementation {};

Executor::Executor(const rclcpp::ExecutorOptions & options)
: spinning(false),
  interrupt_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context)),
  shutdown_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context)),
  notify_waitable_(std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>(
      [this]() {
        this->entities_need_rebuild_.store(true);
      })),
  collector_(notify_waitable_),
  wait_set_({}, {}, {}, {}, {}, {}, options.context),
  current_notify_waitable_(notify_waitable_),
  impl_(std::make_unique<rclcpp::ExecutorImplementation>())
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

  notify_waitable_->add_guard_condition(interrupt_guard_condition_);
  notify_waitable_->add_guard_condition(shutdown_guard_condition_);
}

Executor::~Executor()
{
  std::lock_guard<std::mutex> guard(mutex_);

  notify_waitable_->remove_guard_condition(interrupt_guard_condition_);
  notify_waitable_->remove_guard_condition(shutdown_guard_condition_);
  current_collection_.timers.update(
    {}, {},
    [this](auto timer) {wait_set_.remove_timer(timer);});

  current_collection_.subscriptions.update(
    {}, {},
    [this](auto subscription) {
      wait_set_.remove_subscription(subscription, kDefaultSubscriptionMask);
    });

  current_collection_.clients.update(
    {}, {},
    [this](auto client) {wait_set_.remove_client(client);});

  current_collection_.services.update(
    {}, {},
    [this](auto service) {wait_set_.remove_service(service);});

  current_collection_.guard_conditions.update(
    {}, {},
    [this](auto guard_condition) {wait_set_.remove_guard_condition(guard_condition);});

  current_collection_.waitables.update(
    {}, {},
    [this](auto waitable) {wait_set_.remove_waitable(waitable);});

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
  this->collector_.update_collections();
  return this->collector_.get_all_callback_groups();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
Executor::get_manually_added_callback_groups()
{
  this->collector_.update_collections();
  return this->collector_.get_manually_added_callback_groups();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
Executor::get_automatically_added_callback_groups_from_nodes()
{
  this->collector_.update_collections();
  return this->collector_.get_automatically_added_callback_groups();
}

void
Executor::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  bool notify)
{
  (void) node_ptr;
  this->collector_.add_callback_group(group_ptr);

  if (!spinning.load()) {
    std::lock_guard<std::mutex> guard(mutex_);
    this->collect_entities();
  }

  if (notify) {
    try {
      interrupt_guard_condition_->trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on callback group add: ") + ex.what());
    }
  }
}

void
Executor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  this->collector_.add_node(node_ptr);

  if (!spinning.load()) {
    std::lock_guard<std::mutex> guard(mutex_);
    this->collect_entities();
  }

  if (notify) {
    try {
      interrupt_guard_condition_->trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on node add: ") + ex.what());
    }
  }
}

void
Executor::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  bool notify)
{
  this->collector_.remove_callback_group(group_ptr);

  if (!spinning.load()) {
    std::lock_guard<std::mutex> guard(mutex_);
    this->collect_entities();
  }
  if (notify) {
    try {
      interrupt_guard_condition_->trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on callback group remove: ") + ex.what());
    }
  }
}

void
Executor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

void
Executor::remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  this->collector_.remove_node(node_ptr);

  if (!spinning.load()) {
    std::lock_guard<std::mutex> guard(mutex_);
    this->collect_entities();
  }

  if (notify) {
    try {
      interrupt_guard_condition_->trigger();
    } catch (const rclcpp::exceptions::RCLError & ex) {
      throw std::runtime_error(
              std::string(
                "Failed to trigger guard condition on node remove: ") + ex.what());
    }
  }
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

void
Executor::spin_node_all(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
  std::chrono::nanoseconds max_duration)
{
  this->add_node(node, false);
  spin_all(max_duration);
  this->remove_node(node, false);
}

void
Executor::spin_node_all(std::shared_ptr<rclcpp::Node> node, std::chrono::nanoseconds max_duration)
{
  this->spin_node_all(node->get_node_base_interface(), max_duration);
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

  size_t work_in_queue = 0;
  bool has_waited = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    work_in_queue = ready_executables_.size();
  }
  // The logic below is to guarantee that we:
  // a) run all of the work in the queue before we spin the first time
  // b) spin at least once
  // c) run all of the work in the queue after we spin

  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    AnyExecutable any_exec;
    if (work_in_queue > 0) {
      // If there is work in the queue, then execute it
      // This covers the case that there are things left in the queue from a
      // previous spin.
      if (get_next_ready_executable(any_exec)) {
        execute_any_executable(any_exec);
      }
    } else if (!has_waited && !work_in_queue) {
      // Once the ready queue is empty, then we need to wait at least once.
      wait_for_work(std::chrono::milliseconds(0));
      has_waited = true;
    } else if (has_waited && !work_in_queue) {
      // Once we have emptied the ready queue, but have already waited:
      if (!exhaustive) {
        // In the case of spin some, then we can exit
        break;
      } else {
        // In the case of spin all, then we will allow ourselves to wait again.
        has_waited = false;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    work_in_queue = ready_executables_.size();
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
    interrupt_guard_condition_->trigger();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("Failed to trigger guard condition in cancel: ") + ex.what());
  }
}

void
Executor::execute_any_executable(AnyExecutable & any_exec)
{
  if (!spinning.load()) {
    return;
  }

  if (any_exec.timer) {
    TRACETOOLS_TRACEPOINT(
      rclcpp_executor_execute,
      static_cast<const void *>(any_exec.timer->get_timer_handle().get()));
    execute_timer(any_exec.timer);
  }
  if (any_exec.subscription) {
    TRACETOOLS_TRACEPOINT(
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
  if (any_exec.callback_group) {
    any_exec.callback_group->can_be_taken_from().store(true);
  }
}

template<typename Taker, typename Handler>
static
void
take_and_do_error_handling(
  const char * action_description,
  const char * topic_or_service_name,
  Taker take_action,
  Handler handle_action)
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
  using rclcpp::dynamic_typesupport::DynamicMessage;

  rclcpp::MessageInfo message_info;
  message_info.get_rmw_message_info().from_intra_process = false;

  switch (subscription->get_delivered_message_kind()) {
    // Deliver ROS message
    case rclcpp::DeliveredMessageKind::ROS_MESSAGE:
      {
        if (subscription->can_loan_messages()) {
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
              subscription->get_subscription_handle().get(), loaned_msg);
            if (RCL_RET_OK != ret) {
              RCLCPP_ERROR(
                rclcpp::get_logger("rclcpp"),
                "rcl_return_loaned_message_from_subscription() failed for subscription on topic "
                "'%s': %s",
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
          // TODO(clalancette): In the case that the user is using the MessageMemoryPool,
          // and they take a shared_ptr reference to the message in the callback, this can
          // inadvertently return the message to the pool when the user is still using it.
          // This is a bug that needs to be fixed in the pool, and we should probably have
          // a custom deleter for the message that actually does the return_message().
          subscription->return_message(message);
        }
        break;
      }

    // Deliver serialized message
    case rclcpp::DeliveredMessageKind::SERIALIZED_MESSAGE:
      {
        // This is the case where a copy of the serialized message is taken from
        // the middleware via inter-process communication.
        std::shared_ptr<SerializedMessage> serialized_msg =
          subscription->create_serialized_message();
        take_and_do_error_handling(
          "taking a serialized message from topic",
          subscription->get_topic_name(),
          [&]() {return subscription->take_serialized(*serialized_msg.get(), message_info);},
          [&]()
          {
            subscription->handle_serialized_message(serialized_msg, message_info);
          });
        subscription->return_serialized_message(serialized_msg);
        break;
      }

    // DYNAMIC SUBSCRIPTION ========================================================================
    // Deliver dynamic message
    case rclcpp::DeliveredMessageKind::DYNAMIC_MESSAGE:
      {
        throw std::runtime_error("Unimplemented");
      }

    default:
      {
        throw std::runtime_error("Delivered message kind is not supported");
      }
  }
  return;
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
Executor::collect_entities()
{
  // Get the current list of available waitables from the collector.
  rclcpp::executors::ExecutorEntitiesCollection collection;
  this->collector_.update_collections();
  auto callback_groups = this->collector_.get_all_callback_groups();
  rclcpp::executors::build_entities_collection(callback_groups, collection);

  // Make a copy of notify waitable so we can continue to mutate the original
  // one outside of the execute loop.
  // This prevents the collection of guard conditions in the waitable from changing
  // while we are waiting on it.
  if (notify_waitable_) {
    current_notify_waitable_ = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>(
      *notify_waitable_);
    auto notify_waitable = std::static_pointer_cast<rclcpp::Waitable>(current_notify_waitable_);
    collection.waitables.insert({notify_waitable.get(), {notify_waitable, {}}});
  }

  // We must remove expired entities here, so that we don't continue to use older entities.
  // See https://github.com/ros2/rclcpp/issues/2180 for more information.
  current_collection_.remove_expired_entities();

  // Update each of the groups of entities in the current collection, adding or removing
  // from the wait set as necessary.
  current_collection_.timers.update(
    collection.timers,
    [this](auto timer) {wait_set_.add_timer(timer);},
    [this](auto timer) {wait_set_.remove_timer(timer);});

  current_collection_.subscriptions.update(
    collection.subscriptions,
    [this](auto subscription) {
      wait_set_.add_subscription(subscription, kDefaultSubscriptionMask);
    },
    [this](auto subscription) {
      wait_set_.remove_subscription(subscription, kDefaultSubscriptionMask);
    });

  current_collection_.clients.update(
    collection.clients,
    [this](auto client) {wait_set_.add_client(client);},
    [this](auto client) {wait_set_.remove_client(client);});

  current_collection_.services.update(
    collection.services,
    [this](auto service) {wait_set_.add_service(service);},
    [this](auto service) {wait_set_.remove_service(service);});

  current_collection_.guard_conditions.update(
    collection.guard_conditions,
    [this](auto guard_condition) {wait_set_.add_guard_condition(guard_condition);},
    [this](auto guard_condition) {wait_set_.remove_guard_condition(guard_condition);});

  current_collection_.waitables.update(
    collection.waitables,
    [this](auto waitable) {
      wait_set_.add_waitable(waitable);
    },
    [this](auto waitable) {wait_set_.remove_waitable(waitable);});

  // In the case that an entity already has an expired weak pointer
  // before being removed from the waitset, additionally prune the waitset.
  this->wait_set_.prune_deleted_entities();
  this->entities_need_rebuild_.store(false);

  if (!this->ready_executables_.empty()) {
    std::unordered_set<rclcpp::CallbackGroup::SharedPtr> groups;
    for (const auto & weak_group : callback_groups) {
      auto group = weak_group.lock();
      if (group) {
        groups.insert(group);
      }
    }

    this->ready_executables_.erase(
      std::remove_if(
        this->ready_executables_.begin(),
        this->ready_executables_.end(),
        [groups](auto exec) {
          return groups.count(exec.callback_group) == 0;
        }),
      this->ready_executables_.end());
  }
}

void
Executor::wait_for_work(std::chrono::nanoseconds timeout)
{
  TRACETOOLS_TRACEPOINT(rclcpp_executor_wait_for_work, timeout.count());
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (current_collection_.empty() || this->entities_need_rebuild_.load()) {
      this->collect_entities();
    }
  }

  auto wait_result = wait_set_.wait(timeout);
  if (wait_result.kind() == WaitResultKind::Empty) {
    RCUTILS_LOG_WARN_NAMED(
      "rclcpp",
      "empty wait set received in wait(). This should never happen.");
  }
  rclcpp::executors::ready_executables(current_collection_, wait_result, ready_executables_);
}

bool
Executor::get_next_ready_executable(AnyExecutable & any_executable)
{
  TRACETOOLS_TRACEPOINT(rclcpp_executor_get_next_ready);

  std::lock_guard<std::mutex> guard(mutex_);
  if (ready_executables_.size() == 0) {
    return false;
  }

  any_executable = ready_executables_.front();
  ready_executables_.pop_front();

  if (any_executable.callback_group &&
    any_executable.callback_group->type() == CallbackGroupType::MutuallyExclusive)
  {
    assert(any_executable.callback_group->can_be_taken_from().load());
    any_executable.callback_group->can_be_taken_from().store(false);
  }

  return true;
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

bool
Executor::is_spinning()
{
  return spinning;
}
