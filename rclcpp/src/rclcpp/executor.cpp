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
#include <cassert>
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

Executor::Executor(const std::shared_ptr<rclcpp::Context> & context)
: spinning(false),
  entities_need_rebuild_(true),
  collector_(nullptr),
  wait_set_({}, {}, {}, {}, {}, {}, context)
{
}

Executor::Executor(const rclcpp::ExecutorOptions & options)
: spinning(false),
  interrupt_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context)),
  shutdown_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context)),
  context_(options.context),
  notify_waitable_(std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>(
      [this]() {
        this->entities_need_rebuild_.store(true);
      })),
  entities_need_rebuild_(true),
  collector_(notify_waitable_),
  wait_set_({}, {}, {}, {}, {}, {}, options.context),
  current_notify_waitable_(notify_waitable_),
  impl_(std::make_unique<rclcpp::ExecutorImplementation>())
{
  shutdown_callback_handle_ = context_->add_on_shutdown_callback(
    [weak_gc = std::weak_ptr<rclcpp::GuardCondition>{shutdown_guard_condition_}]() {
      auto strong_gc = weak_gc.lock();
      if (strong_gc) {
        strong_gc->trigger();
      }
    });

  notify_waitable_->add_guard_condition(interrupt_guard_condition_);
  notify_waitable_->add_guard_condition(shutdown_guard_condition_);

  // we need to initially rebuild the collection,
  // so that the notify_waitable_ is added
  collect_entities();
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

void
Executor::trigger_entity_recollect(bool notify)
{
  this->entities_need_rebuild_.store(true);

  if (!spinning.load() && entities_need_rebuild_.exchange(false)) {
    std::lock_guard<std::mutex> guard(mutex_);
    this->collect_entities();
  }

  if (notify) {
    interrupt_guard_condition_->trigger();
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

  try {
    this->trigger_entity_recollect(notify);
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string(
              "Failed to trigger guard condition on callback group add: ") + ex.what());
  }
}

void
Executor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  this->collector_.add_node(node_ptr);

  try {
    this->trigger_entity_recollect(notify);
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string(
              "Failed to trigger guard condition on node add: ") + ex.what());
  }
}

void
Executor::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  bool notify)
{
  this->collector_.remove_callback_group(group_ptr);

  try {
    this->trigger_entity_recollect(notify);
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string(
              "Failed to trigger guard condition on callback group remove: ") + ex.what());
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

  try {
    this->trigger_entity_recollect(notify);
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string(
              "Failed to trigger guard condition on node remove: ") + ex.what());
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

rclcpp::FutureReturnCode
Executor::spin_until_future_complete_impl(
  std::chrono::nanoseconds timeout,
  const std::function<std::future_status(std::chrono::nanoseconds wait_time)> & wait_for_future)
{
  // TODO(wjwwood): does not work recursively; can't call spin_node_until_future_complete
  // inside a callback executed by an executor.

  // Check the future before entering the while loop.
  // If the future is already complete, don't try to spin.
  std::future_status status = wait_for_future(std::chrono::seconds(0));
  if (status == std::future_status::ready) {
    return FutureReturnCode::SUCCESS;
  }

  auto end_time = std::chrono::steady_clock::now();
  std::chrono::nanoseconds timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    timeout);
  if (timeout_ns > std::chrono::nanoseconds::zero()) {
    end_time += timeout_ns;
  }
  std::chrono::nanoseconds timeout_left = timeout_ns;

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_until_future_complete() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(wait_result_.reset();this->spinning.store(false););
  while (rclcpp::ok(this->context_) && spinning.load()) {
    // Do one item of work.
    spin_once_impl(timeout_left);

    // Check if the future is set, return SUCCESS if it is.
    status = wait_for_future(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return FutureReturnCode::SUCCESS;
    }
    // If the original timeout is < 0, then this is blocking, never TIMEOUT.
    if (timeout_ns < std::chrono::nanoseconds::zero()) {
      continue;
    }
    // Otherwise check if we still have time to wait, return TIMEOUT if not.
    auto now = std::chrono::steady_clock::now();
    if (now >= end_time) {
      return FutureReturnCode::TIMEOUT;
    }
    // Subtract the elapsed time from the original timeout.
    timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
  }

  // The future did not complete before ok() returned false, return INTERRUPTED.
  return FutureReturnCode::INTERRUPTED;
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
  RCPPUTILS_SCOPE_EXIT(wait_result_.reset();this->spinning.store(false););

  // clear the wait result and wait for work without blocking to collect the work
  // for the first time
  // both spin_some and spin_all wait for work at the beginning
  wait_result_.reset();
  wait_for_work(std::chrono::milliseconds(0));
  bool entity_states_fully_polled = true;

  if (entities_need_rebuild_) {
    // if the last wait triggered a collection rebuild, we need to call
    // wait_for_work once more, in order to do a collection rebuild and collect
    // events from the just added entities
    entity_states_fully_polled = false;
  }

  // The logic of this while loop is as follows:
  //
  // - while not shutdown, and spinning (not canceled), and not max duration reached...
  // - try to get an executable item to execute, and execute it if available
  // - otherwise, reset the wait result, and ...
  // - if there was no work available just after waiting, break the loop unconditionally
  //   - this is appropriate for both spin_some and spin_all which use this function
  // - else if exhaustive = true, then wait for work again
  //   - this is only used for spin_all and not spin_some
  // - else break
  //   - this only occurs with spin_some
  //
  // The logic of this loop is subtle and should be carefully changed if at all.
  // See also:
  //   https://github.com/ros2/rclcpp/issues/2508
  //   https://github.com/ros2/rclcpp/pull/2517
  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    AnyExecutable any_exec;
    if (get_next_ready_executable(any_exec)) {
      execute_any_executable(any_exec);
      // during the execution some entity might got ready therefore we need
      // to repoll the states of all entities
      entity_states_fully_polled = false;
    } else {
      // if nothing is ready, reset the result to clear it
      wait_result_.reset();

      if (entity_states_fully_polled) {
        // there was no work after just waiting, always exit in this case
        // before the exhaustive condition can be checked
        break;
      }

      if (exhaustive) {
        // if exhaustive, wait for work again
        // this only happens for spin_all; spin_some only waits at the start
        wait_for_work(std::chrono::milliseconds(0));
        entity_states_fully_polled = true;
        if (entities_need_rebuild_) {
          // if the last wait triggered a collection rebuild, we need to call
          // wait_for_work once more, in order to do a collection rebuild and
          // collect events from the just added entities
          entity_states_fully_polled = false;
        }
      } else {
        break;
      }
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
  RCPPUTILS_SCOPE_EXIT(wait_result_.reset();this->spinning.store(false););
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

  assert(
    (void("cannot execute an AnyExecutable without a valid callback group"),
    any_exec.callback_group));

  if (any_exec.timer) {
    TRACETOOLS_TRACEPOINT(
      rclcpp_executor_execute,
      static_cast<const void *>(any_exec.timer->get_timer_handle().get()));
    execute_timer(any_exec.timer, any_exec.data);
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
    const std::shared_ptr<void> & const_data = any_exec.data;
    any_exec.waitable->execute(const_data);
  }

  // Reset the callback_group, regardless of type
  any_exec.callback_group->can_be_taken_from().store(true);
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
}

void
Executor::execute_timer(rclcpp::TimerBase::SharedPtr timer, const std::shared_ptr<void> & data_ptr)
{
  timer->execute_callback(data_ptr);
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
Executor::execute_client(rclcpp::ClientBase::SharedPtr client)
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
  // Updating the entity collection and waitset expires any active result
  this->wait_result_.reset();

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
    [this](auto waitable) {wait_set_.add_waitable(waitable);},
    [this](auto waitable) {wait_set_.remove_waitable(waitable);});

  // In the case that an entity already has an expired weak pointer
  // before being removed from the waitset, additionally prune the waitset.
  this->wait_set_.prune_deleted_entities();
}

void
Executor::wait_for_work(std::chrono::nanoseconds timeout)
{
  TRACETOOLS_TRACEPOINT(rclcpp_executor_wait_for_work, timeout.count());

  // Clear any previous wait result
  this->wait_result_.reset();

  // we need to make sure that callback groups don't get out of scope
  // during the wait. As in jazzy, they are not covered by the DynamicStorage,
  // we explicitly hold them here as a bugfix
  std::vector<rclcpp::CallbackGroup::SharedPtr> cbgs;

  {
    std::lock_guard<std::mutex> guard(mutex_);

    if (this->entities_need_rebuild_.exchange(false) || current_collection_.empty()) {
      this->collect_entities();
    }

    auto callback_groups = this->collector_.get_all_callback_groups();
    cbgs.resize(callback_groups.size());
    for(const auto & w_ptr : callback_groups) {
      auto shr_ptr = w_ptr.lock();
      if(shr_ptr) {
        cbgs.push_back(std::move(shr_ptr));
      }
    }
  }

  this->wait_result_.emplace(wait_set_.wait(timeout));

  // drop references to the callback groups, before trying to execute anything
  cbgs.clear();

  if (!this->wait_result_ || this->wait_result_->kind() == WaitResultKind::Empty) {
    RCUTILS_LOG_WARN_NAMED(
      "rclcpp",
      "empty wait set received in wait(). This should never happen.");
  } else {
    if (this->wait_result_->kind() == WaitResultKind::Ready && current_notify_waitable_) {
      auto & rcl_wait_set = this->wait_result_->get_wait_set().get_rcl_wait_set();
      if (current_notify_waitable_->is_ready(rcl_wait_set)) {
        current_notify_waitable_->execute(current_notify_waitable_->take_data());
      }
    }
  }
}

bool
Executor::get_next_ready_executable(AnyExecutable & any_executable)
{
  TRACETOOLS_TRACEPOINT(rclcpp_executor_get_next_ready);

  bool valid_executable = false;

  if (!wait_result_.has_value() || wait_result_->kind() != rclcpp::WaitResultKind::Ready) {
    return false;
  }

  if (!valid_executable) {
    size_t current_timer_index = 0;
    while (true) {
      auto [timer, timer_index] = wait_result_->peek_next_ready_timer(current_timer_index);
      if (nullptr == timer) {
        break;
      }
      current_timer_index = timer_index;
      auto entity_iter = current_collection_.timers.find(timer->get_timer_handle().get());
      if (entity_iter != current_collection_.timers.end()) {
        auto callback_group = entity_iter->second.callback_group.lock();
        if (!callback_group || !callback_group->can_be_taken_from()) {
          current_timer_index++;
          continue;
        }
        // At this point the timer is either ready for execution or was perhaps
        // it was canceled, based on the result of call(), but either way it
        // should not be checked again from peek_next_ready_timer(), so clear
        // it from the wait result.
        wait_result_->clear_timer_with_index(current_timer_index);
        // Check that the timer should be called still, i.e. it wasn't canceled.
        any_executable.data = timer->call();
        if (!any_executable.data) {
          current_timer_index++;
          continue;
        }
        any_executable.timer = timer;
        any_executable.callback_group = callback_group;
        valid_executable = true;
        break;
      }
      current_timer_index++;
    }
  }

  if (!valid_executable) {
    while (auto subscription = wait_result_->next_ready_subscription()) {
      auto entity_iter = current_collection_.subscriptions.find(
        subscription->get_subscription_handle().get());
      if (entity_iter != current_collection_.subscriptions.end()) {
        auto callback_group = entity_iter->second.callback_group.lock();
        if (!callback_group || !callback_group->can_be_taken_from()) {
          continue;
        }
        any_executable.subscription = subscription;
        any_executable.callback_group = callback_group;
        valid_executable = true;
        break;
      }
    }
  }

  if (!valid_executable) {
    while (auto service = wait_result_->next_ready_service()) {
      auto entity_iter = current_collection_.services.find(service->get_service_handle().get());
      if (entity_iter != current_collection_.services.end()) {
        auto callback_group = entity_iter->second.callback_group.lock();
        if (!callback_group || !callback_group->can_be_taken_from()) {
          continue;
        }
        any_executable.service = service;
        any_executable.callback_group = callback_group;
        valid_executable = true;
        break;
      }
    }
  }

  if (!valid_executable) {
    while (auto client = wait_result_->next_ready_client()) {
      auto entity_iter = current_collection_.clients.find(client->get_client_handle().get());
      if (entity_iter != current_collection_.clients.end()) {
        auto callback_group = entity_iter->second.callback_group.lock();
        if (!callback_group || !callback_group->can_be_taken_from()) {
          continue;
        }
        any_executable.client = client;
        any_executable.callback_group = callback_group;
        valid_executable = true;
        break;
      }
    }
  }

  if (!valid_executable) {
    while (auto waitable = wait_result_->next_ready_waitable()) {
      auto entity_iter = current_collection_.waitables.find(waitable.get());
      if (entity_iter != current_collection_.waitables.end()) {
        auto callback_group = entity_iter->second.callback_group.lock();
        if (!callback_group || !callback_group->can_be_taken_from()) {
          continue;
        }
        any_executable.waitable = waitable;
        any_executable.callback_group = callback_group;
        any_executable.data = waitable->take_data();
        valid_executable = true;
        break;
      }
    }
  }

  if (any_executable.callback_group) {
    if (any_executable.callback_group->type() == CallbackGroupType::MutuallyExclusive) {
      assert(any_executable.callback_group->can_be_taken_from().load());
      any_executable.callback_group->can_be_taken_from().store(false);
    }
  }


  return valid_executable;
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
