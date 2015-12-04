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

#include <chrono>
#include <functional>
#include <vector>

#include "rclcpp/utilities.hpp"
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor;

MultiThreadedExecutor::MultiThreadedExecutor(rclcpp::memory_strategy::MemoryStrategy::SharedPtr ms)
: executor::Executor(ms)
{
  number_of_threads_ = std::thread::hardware_concurrency();
  if (number_of_threads_ == 0) {
    number_of_threads_ = 1;
  }
}

MultiThreadedExecutor::~MultiThreadedExecutor() {}

void
MultiThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    std::lock_guard<std::mutex> wait_lock(wait_mutex_);
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
  }

  run(thread_id);
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
  thread_number_by_thread_id_[std::this_thread::get_id()] = this_thread_number;
  while (rclcpp::utilities::ok() && spinning.load()) {
    executor::AnyExecutable::SharedPtr any_exec;
    {
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      if (!rclcpp::utilities::ok() || !spinning.load()) {
        return;
      }
      any_exec = get_next_executable();
    }
    execute_any_executable(any_exec);
  }
}
