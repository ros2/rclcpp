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

#include "rclcpp/executors/executor_entities_collection.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"

using rclcpp::executors::SingleThreadedExecutor;

SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options) {}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );

  while (rclcpp::ok(this->context_) && spinning.load()) {

    wait_for_work();

    std::deque<rclcpp::AnyExecutable> to_exec;

    {
      std::lock_guard<std::mutex> guard(mutex_);
      to_exec = this->ready_executables_;
      this->ready_executables_.clear();
    }

    for (auto & exec: to_exec)
    {
      if (exec.callback_group &&
        exec.callback_group->type() == CallbackGroupType::MutuallyExclusive)
      {
        assert(any_executable.callback_group->can_be_taken_from().load());
        exec.callback_group->can_be_taken_from().store(false);
      }
      execute_any_executable(exec);
    }

    FrameMark;
  }
}
