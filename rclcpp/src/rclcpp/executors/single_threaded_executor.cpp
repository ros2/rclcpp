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

#include "rcpputils/scope_exit.hpp"
#include "rcpputils/thread.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"

using rclcpp::executors::SingleThreadedExecutor;

const char SingleThreadedExecutor::default_name[] = "RCLCPP_EXECUTOR_SINGLE_THREADED";

static std::optional<rcpputils::ThreadAttribute>
default_thread_attr(const rclcpp::ExecutorOptions & options);

SingleThreadedExecutor::SingleThreadedExecutor(
  const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options),
  thread_attr_(default_thread_attr(options)) {}

SingleThreadedExecutor::SingleThreadedExecutor(
  const rclcpp::ExecutorOptions & options,
  const rcpputils::ThreadAttribute & thread_attr)
: rclcpp::Executor(options),
  thread_attr_(thread_attr) {}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin()
{
  if (thread_attr_) {
    rcpputils::Thread thread(thread_attr_.value(), &SingleThreadedExecutor::run, this);
    thread.join();
  } else {
    run();
  }
}

void
SingleThreadedExecutor::run()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable)) {
      execute_any_executable(any_executable);
    }
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
    name = SingleThreadedExecutor::default_name;
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
        "SingleThreadedExecutor is named \"%s\", but not found corresponding thread attribute.",
        name.c_str());
    }
    return std::nullopt;
  }
}
