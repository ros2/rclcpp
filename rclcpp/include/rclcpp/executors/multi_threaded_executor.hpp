// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_

#include <mutex>
#include <thread>
#include <unordered_map>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executors
{
namespace multi_threaded_executor
{

class MultiThreadedExecutor : public executor::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MultiThreadedExecutor)

  RCLCPP_PUBLIC
  MultiThreadedExecutor(
    const executor::ExecutorArgs & args = rclcpp::executor::create_default_executor_arguments());

  RCLCPP_PUBLIC
  virtual ~MultiThreadedExecutor();

  RCLCPP_PUBLIC
  void
  spin();

  RCLCPP_PUBLIC
  size_t
  get_number_of_threads();

protected:
  RCLCPP_PUBLIC
  void
  run(size_t this_thread_number);

private:
  RCLCPP_DISABLE_COPY(MultiThreadedExecutor)

  std::mutex wait_mutex_;
  size_t number_of_threads_;
};

}  // namespace multi_threaded_executor
}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_
