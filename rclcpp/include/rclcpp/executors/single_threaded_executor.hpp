/* Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RCLCPP_RCLCPP_EXECUTORS_SINGLE_THREADED_EXECUTOR_HPP_
#define RCLCPP_RCLCPP_EXECUTORS_SINGLE_THREADED_EXECUTOR_HPP_

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>

#include <ros_middleware_interface/functions.h>
#include <ros_middleware_interface/handles.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/macros.hpp>
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
  RCLCPP_MAKE_SHARED_DEFINITIONS(SingleThreadedExecutor);

  SingleThreadedExecutor() {}

  ~SingleThreadedExecutor() {}

  void spin()
  {
    while (rclcpp::utilities::ok())
    {
      auto any_exec = get_next_executable();
      execute_any_executable(any_exec);
    }
  }

  void spin_node_some(rclcpp::node::Node &node)
  {
    reset_subscriber_handles();
    populate_all_handles_with_node(node);
    // non-blocking = true
    auto any_exec = get_next_executable(true);
    while (any_exec->subscription)
    {
      execute_subscription(any_exec->subscription);
      // non-blocking = true
      any_exec = get_next_executable(true);
    }
    reset_subscriber_handles();
  }

private:
  RCLCPP_DISABLE_COPY(SingleThreadedExecutor);

};

} /* namespace single_threaded_executor */
} /* namespace executors */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_EXECUTORS_SINGLE_THREADED_EXECUTOR_HPP_ */
