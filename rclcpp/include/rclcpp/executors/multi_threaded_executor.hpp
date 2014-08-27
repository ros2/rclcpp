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

#ifndef RCLCPP_RCLCPP_EXECUTORS_MULTI_THREADED_EXECUTOR_HPP_
#define RCLCPP_RCLCPP_EXECUTORS_MULTI_THREADED_EXECUTOR_HPP_

#include <cassert>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <vector>

#include <ros_middleware_interface/functions.h>
#include <ros_middleware_interface/handles.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

namespace rclcpp
{
namespace executors
{
namespace multi_threaded_executor
{

class MultiThreadedExecutor : public executor::Executor
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(MultiThreadedExecutor);

  MultiThreadedExecutor() {}

  ~MultiThreadedExecutor() {}

  void add_node(rclcpp::node::Node::SharedPtr &node_ptr)
  {
    this->weak_nodes_.push_back(node_ptr);
  }

  void spin()
  {
    std::vector<std::thread> threads;
    size_t number_of_threads = std::thread::hardware_concurrency();
    if (number_of_threads == 0)
    {
      number_of_threads = 1;
    }
    for (; number_of_threads > 0; --number_of_threads)
    {
      threads.emplace_back(std::thread(&MultiThreadedExecutor::run, this));
    }
    for (auto &thread : threads)
    {
      thread.join();
    }
  }

private:
  void run()
  {
    while (rclcpp::utilities::ok())
    {
      std::shared_ptr<AnyExecutable> any_exec;
      {
        std::lock_guard<std::mutex> wait_lock(wait_mutex_);
        if (!rclcpp::utilities::ok())
        {
          return;
        }
        any_exec = get_next_executable();
      }
      if (any_exec && any_exec->subscription)
      {
        // Do callback
        execute_subscription(any_exec->subscription);
      }
    }
  }

  RCLCPP_DISABLE_COPY(MultiThreadedExecutor);

  std::vector<std::weak_ptr<rclcpp::node::Node>> weak_nodes_;
  std::mutex wait_mutex_;

};

} /* namespace multi_threaded_executor */
} /* namespace executors */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_EXECUTORS_MULTI_THREADED_EXECUTOR_HPP_ */
