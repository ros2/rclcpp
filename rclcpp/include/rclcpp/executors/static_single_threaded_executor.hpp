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

#ifndef RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_

#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace rclcpp
{
namespace executors
{

/// Static executor implementation
/**
 * This executor is a static version of the original single threaded executor.
 * It's static because it doesn't reconstruct the executable list for every iteration.
 * All nodes, callbackgroups, timers, subscriptions etc. are created before
 * spin() is called, and modified only when an entity is added/removed to/from a node.
 *
 * To run this executor instead of SingleThreadedExecutor replace:
 * rclcpp::executors::SingleThreadedExecutor exec;
 * by
 * rclcpp::executors::StaticSingleThreadedExecutor exec;
 * in your source code and spin node(s) in the following way:
 * exec.add_node(node);
 * exec.spin();
 * exec.remove_node(node);
 */

class StaticSingleThreadedExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SingleThreadedExecutor)

  /// Default constructor. See the default constructor for Executor.
  RCLCPP_PUBLIC
  explicit StaticSingleThreadedExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~StaticSingleThreadedExecutor();

  /// Single-threaded implementation of spin.
  RCLCPP_PUBLIC
  void
  spin() override;

private:
  RCLCPP_DISABLE_COPY(StaticSingleThreadedExecutor)
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_
