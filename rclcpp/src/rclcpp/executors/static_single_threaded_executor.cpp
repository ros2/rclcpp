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
#include "rclcpp/executors/executor_notify_waitable.hpp"
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
    std::deque<rclcpp::AnyExecutable> to_exec;

    std::lock_guard<std::mutex> guard(mutex_);
    if (current_collection_.empty() || this->entities_need_rebuild_.load()) {
      this->collect_entities();
    }

    auto wait_result = wait_set_.wait(std::chrono::nanoseconds(-1));
    if (wait_result.kind() == WaitResultKind::Empty) {
      RCUTILS_LOG_WARN_NAMED(
        "rclcpp",
        "empty wait set received in wait(). This should never happen.");
      continue;
    }
    execute_ready_executables(current_collection_, wait_result, false);
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

  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    // Get executables that are ready now
    std::lock_guard<std::mutex> guard(mutex_);
    if (current_collection_.empty() || this->entities_need_rebuild_.load()) {
      this->collect_entities();
    }

    auto wait_result = wait_set_.wait(std::chrono::nanoseconds(0));
    if (wait_result.kind() == WaitResultKind::Empty) {
      RCUTILS_LOG_WARN_NAMED(
        "rclcpp",
        "empty wait set received in wait(). This should never happen.");
      continue;
    }

    // Execute ready executables
    bool work_available = execute_ready_executables(current_collection_, wait_result, false);
    if (!work_available || !exhaustive) {
      break;
    }
  }
}

void
StaticSingleThreadedExecutor::spin_once_impl(std::chrono::nanoseconds timeout)
{
  if (rclcpp::ok(context_) && spinning.load()) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (current_collection_.empty() || this->entities_need_rebuild_.load()) {
      this->collect_entities();
    }

    auto wait_result = wait_set_.wait(std::chrono::nanoseconds(timeout));
    if (wait_result.kind() == WaitResultKind::Empty) {
      RCUTILS_LOG_WARN_NAMED(
        "rclcpp",
        "empty wait set received in wait(). This should never happen.");
      return;
    }

    // Execute ready executables
    execute_ready_executables(current_collection_, wait_result, true);
  }
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

  auto rcl_wait_set = wait_result.get_wait_set().get_rcl_wait_set();

  for (size_t ii = 0; ii < rcl_wait_set.size_of_timers; ++ii) {
    if (nullptr == rcl_wait_set.timers[ii]) {continue;}
    auto entity_iter = collection.timers.find(rcl_wait_set.timers[ii]);
    if (entity_iter != collection.timers.end()) {
      auto entity = entity_iter->second.entity.lock();
      if (!entity) {
        continue;
      }
      if (!entity->call()) {
        continue;
      }
      execute_timer(entity);
      if (spin_once) {
        return true;
      }
      any_ready_executable = true;
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_subscriptions; ++ii) {
    if (nullptr == rcl_wait_set.subscriptions[ii]) {continue;}
    auto entity_iter = collection.subscriptions.find(rcl_wait_set.subscriptions[ii]);
    if (entity_iter != collection.subscriptions.end()) {
      auto entity = entity_iter->second.entity.lock();
      if (!entity) {
        continue;
      }
      execute_subscription(entity);
      if (spin_once) {
        return true;
      }
      any_ready_executable = true;
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_services; ++ii) {
    if (nullptr == rcl_wait_set.services[ii]) {continue;}
    auto entity_iter = collection.services.find(rcl_wait_set.services[ii]);
    if (entity_iter != collection.services.end()) {
      auto entity = entity_iter->second.entity.lock();
      if (!entity) {
        continue;
      }
      execute_service(entity);
      if (spin_once) {
        return true;
      }
      any_ready_executable = true;
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_clients; ++ii) {
    if (nullptr == rcl_wait_set.clients[ii]) {continue;}
    auto entity_iter = collection.clients.find(rcl_wait_set.clients[ii]);
    if (entity_iter != collection.clients.end()) {
      auto entity = entity_iter->second.entity.lock();
      if (!entity) {
        continue;
      }
      execute_client(entity);
      if (spin_once) {
        return true;
      }
      any_ready_executable = true;
    }
  }

  for (auto & [handle, entry] : collection.waitables) {
    auto waitable = entry.entity.lock();
    if (!waitable) {
      continue;
    }
    if (!waitable->is_ready(&rcl_wait_set)) {
      continue;
    }

    auto data = waitable->take_data();
    waitable->execute(data);
    if (spin_once) {
      return true;
    }
    any_ready_executable = true;
  }
  return any_ready_executable;
}
