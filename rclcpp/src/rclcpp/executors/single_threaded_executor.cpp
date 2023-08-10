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

#include "rcpputils/scope_exit.hpp"
#include "rcpputils/threads.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"

using rclcpp::executors::SingleThreadedExecutor;

SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options),
  thread_attributes_(nullptr)
{
  if (rcutils_thread_attrs_t * attrs = rcl_context_get_thread_attrs(
    options.context->get_rcl_context().get()))
  {
    thread_attributes_ = attrs;
  }

  if (thread_attributes_ && thread_attributes_->num_attributes != 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "Specified thread attributes contains multiple configurations.\n"
      "The executor runs only using first configuration.");
  }
}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin()
{
  if (thread_attributes_) {
    rcpputils::Thread::Attribute thread_attr;
    thread_attr.set_thread_attribute(
      thread_attributes_->attributes[0]);
    rcpputils::this_thread::run_with_thread_attribute(
      thread_attr, &SingleThreadedExecutor::run, this);
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
