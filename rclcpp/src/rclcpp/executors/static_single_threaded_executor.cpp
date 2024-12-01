// Copyright 2019 Nobleo Technology
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

#include "rclcpp/executors/executor_entities_collection.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rclcpp/executors/static_single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"

using rclcpp::executors::StaticSingleThreadedExecutor;

StaticSingleThreadedExecutor::StaticSingleThreadedExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options)
{
}

StaticSingleThreadedExecutor::~StaticSingleThreadedExecutor() {}

void
StaticSingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );

  // This is essentially the contents of the rclcpp::Executor::wait_for_work method,
  // except we need to keep the wait result to reproduce the StaticSingleThreadedExecutor
  // behavior.
  while (rclcpp::ok(this->context_) && spinning.load()) {
    this->spin_once_impl(std::chrono::nanoseconds(-1));
  }
}

void
StaticSingleThreadedExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
  // In this context a 0 input max_duration means no duration limit
  if (std::chrono::nanoseconds(0) == max_duration) {
    max_duration = std::chrono::nanoseconds::max();
  }
  return this->spin_some_impl(max_duration, false);
}

void
StaticSingleThreadedExecutor::spin_all(std::chrono::nanoseconds max_duration)
{
  if (max_duration < std::chrono::nanoseconds(0)) {
    throw std::invalid_argument("max_duration must be greater than or equal to 0");
  }
  return this->spin_some_impl(max_duration, true);
}

void
StaticSingleThreadedExecutor::spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive)
{
  auto start = std::chrono::steady_clock::now();
  auto max_duration_not_elapsed = [max_duration, start]() {
      const auto spin_forever = std::chrono::nanoseconds(0) == max_duration;
      const auto cur_duration = std::chrono::steady_clock::now() - start;
      return spin_forever || (cur_duration < max_duration);
    };

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_some() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    // Get executables that are ready now
    std::lock_guard<std::mutex> guard(mutex_);

    auto wait_result = this->collect_and_wait(std::chrono::nanoseconds(0));
    if (wait_result.has_value()) {
      // Execute ready executables
      bool work_available = this->execute_ready_executables(
        current_collection_,
        wait_result.value(),
        false);
      if (!work_available || !exhaustive) {
        break;
      }
    }
  }
}

void
StaticSingleThreadedExecutor::spin_once_impl(std::chrono::nanoseconds timeout)
{
  if (rclcpp::ok(context_) && spinning.load()) {
    std::lock_guard<std::mutex> guard(mutex_);
    auto wait_result = this->collect_and_wait(timeout);
    if (wait_result.has_value()) {
      this->execute_ready_executables(current_collection_, wait_result.value(), true);
    }
  }
}

std::optional<rclcpp::WaitResult<rclcpp::WaitSet>>
StaticSingleThreadedExecutor::collect_and_wait(std::chrono::nanoseconds timeout)
{
  // we need to make sure that callback groups don't get out of scope
  // during the wait. As in jazzy, they are not covered by the DynamicStorage,
  // we explicitly hold them here as a bugfix
  std::vector<rclcpp::CallbackGroup::SharedPtr> cbgs;

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

  auto wait_result = wait_set_.wait(std::chrono::nanoseconds(timeout));

  // drop references to the callback groups, before trying to execute anything
  cbgs.clear();

  if (wait_result.kind() == WaitResultKind::Empty) {
    RCUTILS_LOG_WARN_NAMED(
      "rclcpp",
      "empty wait set received in wait(). This should never happen.");
    return {};
  } else {
    if (wait_result.kind() == WaitResultKind::Ready && current_notify_waitable_) {
      auto & rcl_wait_set = wait_result.get_wait_set().get_rcl_wait_set();
      if (current_notify_waitable_->is_ready(rcl_wait_set)) {
        current_notify_waitable_->execute(current_notify_waitable_->take_data());
      }
    }
  }
  return wait_result;
}

// This preserves the "scheduling semantics" of the StaticSingleThreadedExecutor
// from the original implementation.
bool StaticSingleThreadedExecutor::execute_ready_executables(
  const rclcpp::executors::ExecutorEntitiesCollection & collection,
  rclcpp::WaitResult<rclcpp::WaitSet> & wait_result,
  bool spin_once)
{
  bool any_ready_executable = false;
  if (wait_result.kind() != rclcpp::WaitResultKind::Ready) {
    return any_ready_executable;
  }

  while (auto subscription = wait_result.next_ready_subscription()) {
    auto entity_iter = collection.subscriptions.find(subscription->get_subscription_handle().get());
    if (entity_iter != collection.subscriptions.end()) {
      execute_subscription(subscription);
      any_ready_executable = true;
      if (spin_once) {return any_ready_executable;}
    }
  }

  size_t current_timer_index = 0;
  while (true) {
    auto [timer, timer_index] = wait_result.peek_next_ready_timer(current_timer_index);
    if (nullptr == timer) {
      break;
    }
    current_timer_index = timer_index;
    auto entity_iter = collection.timers.find(timer->get_timer_handle().get());
    if (entity_iter != collection.timers.end()) {
      wait_result.clear_timer_with_index(current_timer_index);
      auto data = timer->call();
      if (!data) {
        // someone canceled the timer between is_ready and call
        continue;
      }

      execute_timer(std::move(timer), data);
      any_ready_executable = true;
      if (spin_once) {return any_ready_executable;}
    }
  }

  while (auto client = wait_result.next_ready_client()) {
    auto entity_iter = collection.clients.find(client->get_client_handle().get());
    if (entity_iter != collection.clients.end()) {
      execute_client(client);
      any_ready_executable = true;
      if (spin_once) {return any_ready_executable;}
    }
  }

  while (auto service = wait_result.next_ready_service()) {
    auto entity_iter = collection.services.find(service->get_service_handle().get());
    if (entity_iter != collection.services.end()) {
      execute_service(service);
      any_ready_executable = true;
      if (spin_once) {return any_ready_executable;}
    }
  }

  while (auto waitable = wait_result.next_ready_waitable()) {
    auto entity_iter = collection.waitables.find(waitable.get());
    if (entity_iter != collection.waitables.end()) {
      const auto data = waitable->take_data();
      waitable->execute(data);
      any_ready_executable = true;
      if (spin_once) {return any_ready_executable;}
    }
  }
  return any_ready_executable;
}
