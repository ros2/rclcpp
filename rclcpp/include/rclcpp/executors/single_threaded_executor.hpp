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

#ifndef RCLCPP_RCLCPP_EXECUTORS_SINGLE_THREADED_EXECUTOR_HPP_
#define RCLCPP_RCLCPP_EXECUTORS_SINGLE_THREADED_EXECUTOR_HPP_

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>

#include <rmw/rmw.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/memory_strategies.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/rate.hpp>

namespace rclcpp
{
namespace executors
{
namespace single_threaded_executor
{

class SingleThreadedExecutor : public executor::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SingleThreadedExecutor);

  SingleThreadedExecutor(memory_strategy::MemoryStrategy::SharedPtr ms =
    memory_strategy::create_default_strategy())
  : executor::Executor(ms) {}

  virtual ~SingleThreadedExecutor() {}

  void spin()
  {
    while (rclcpp::utilities::ok()) {
      auto any_exec = get_next_executable();
      execute_any_executable(any_exec);
    }
  }

private:
  RCLCPP_DISABLE_COPY(SingleThreadedExecutor);

};

} /* namespace single_threaded_executor */
} /* namespace executors */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_EXECUTORS_SINGLE_THREADED_EXECUTOR_HPP_ */
