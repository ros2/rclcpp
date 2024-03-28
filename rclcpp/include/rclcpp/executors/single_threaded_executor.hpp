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

#ifndef RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_

#include <rmw/rmw.h>

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>
#include <optional>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/thread/thread_attribute.hpp"

namespace rclcpp
{
namespace executors
{

/// Single-threaded executor implementation.
/**
 * This is the default executor created by rclcpp::spin.
 */
class SingleThreadedExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SingleThreadedExecutor)

  /// Default constructor. See the default constructor for Executor.
  RCLCPP_PUBLIC
  explicit SingleThreadedExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  RCLCPP_PUBLIC
  explicit SingleThreadedExecutor(
    const rclcpp::ExecutorOptions & options,
    const rcpputils::ThreadAttribute & thread_attr);

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~SingleThreadedExecutor();

  /// Single-threaded implementation of spin.
  /**
   * This function will block until work comes in, execute it, and then repeat
   * the process until canceled.
   * It may be interrupt by a call to rclcpp::Executor::cancel() or by ctrl-c
   * if the associated context is configured to shutdown on SIGINT.
   * \throws std::runtime_error when spin() called while already spinning
   */
  RCLCPP_PUBLIC
  void
  spin() override;

  RCLCPP_PUBLIC
  bool has_thread_attribute() const
  {
    return thread_attr_.has_value();
  }

  RCLCPP_PUBLIC
  const std::optional<rcpputils::ThreadAttribute> &
  get_thread_attribute() const
  {
    return thread_attr_;
  }

  RCLCPP_PUBLIC
  static const char default_name[];

protected:
  RCLCPP_PUBLIC
  void
  run();

private:
  RCLCPP_DISABLE_COPY(SingleThreadedExecutor)

  std::optional<rcpputils::ThreadAttribute> thread_attr_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_
