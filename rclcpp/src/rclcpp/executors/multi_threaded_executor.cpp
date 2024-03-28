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

#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "rcpputils/scope_exit.hpp"
#include "rcpputils/thread.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

using rclcpp::executors::MultiThreadedExecutor;

const char MultiThreadedExecutor::default_name[] = "RCLCPP_EXECUTOR_MULTI_THREADED";

static std::optional<rcpputils::ThreadAttribute>
default_thread_attr(const rclcpp::ExecutorOptions & options);

MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: MultiThreadedExecutor(
    options, number_of_threads, default_thread_attr(options),
    yield_before_execute, next_exec_timeout) {}

MultiThreadedExecutor::MultiThreadedExecutor(
  size_t number_of_threads,
  rcpputils::ThreadAttribute const & thread_attr,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: MultiThreadedExecutor(
    rclcpp::ExecutorOptions(), number_of_threads, std::optional(thread_attr),
    yield_before_execute, next_exec_timeout) {}

MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  rcpputils::ThreadAttribute const & thread_attr,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: MultiThreadedExecutor(
    options, number_of_threads, std::optional(thread_attr),
    yield_before_execute, next_exec_timeout) {}

MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  std::optional<rcpputils::ThreadAttribute> thread_attr,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: rclcpp::Executor(options),
  thread_attr_(std::move(thread_attr)),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_(next_exec_timeout)
{
  number_of_threads_ = number_of_threads > 0 ?
    number_of_threads :
    std::max(rcpputils::Thread::hardware_concurrency(), 2U);

  if (number_of_threads_ == 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "MultiThreadedExecutor is used with a single thread.\n"
      "Use the SingleThreadedExecutor instead.");
  }
}

MultiThreadedExecutor::~MultiThreadedExecutor() {}

void
MultiThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<rcpputils::Thread> threads;
  size_t thread_id = 0;

  if (thread_attr_) {
    std::lock_guard wait_lock{wait_mutex_};
    for (; thread_id < number_of_threads_; ++thread_id) {
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(thread_attr_.value(), func);
    }
  } else {
    {
      std::lock_guard wait_lock{wait_mutex_};
      for (; thread_id < number_of_threads_ - 1; ++thread_id) {
        auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
        threads.emplace_back(func);
      }
    }
    run(thread_id);
  }

  for (auto & thread : threads) {
    thread.join();
  }
}

size_t
MultiThreadedExecutor::get_number_of_threads()
{
  return number_of_threads_;
}

void
MultiThreadedExecutor::run(size_t this_thread_number)
{
  (void)this_thread_number;
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    {
      std::lock_guard wait_lock{wait_mutex_};
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      if (!get_next_executable(any_exec, next_exec_timeout_)) {
        continue;
      }
    }
    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    execute_any_executable(any_exec);

    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}

std::optional<rcpputils::ThreadAttribute>
default_thread_attr(rclcpp::ExecutorOptions const & options)
{
  const rcutils_thread_attrs_t * attrs = rcl_context_get_thread_attrs(
    options.context->get_rcl_context().get());
  if (!attrs) {
    return std::nullopt;
  }

  std::string name;
  bool name_specified = !options.name.empty();
  if (name_specified) {
    name = options.name;
  } else {
    name = MultiThreadedExecutor::default_name;
  }

  const rcutils_thread_attr_t * attrs_beg = attrs->attributes;
  const rcutils_thread_attr_t * attrs_end = attrs->attributes + attrs->num_attributes;
  const rcutils_thread_attr_t * attr = std::find_if(
    attrs_beg, attrs_end,
    [&](const auto & attr) {
      return attr.tag == name;
    });
  if (attr != attrs_end) {
    return rcpputils::ThreadAttribute(*attr);
  } else {
    if (name_specified) {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "MultiThreadedExecutor is named \"%s\", but not found corresponding thread attribute.",
        name.c_str());
    }
    return std::nullopt;
  }
}
