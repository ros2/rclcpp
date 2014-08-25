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

#ifndef RCLCPP_RCLCPP_RCLCPP_HPP_
#define RCLCPP_RCLCPP_RCLCPP_HPP_

#include <csignal>
#include <memory>
#include <mutex>

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

namespace
{
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> g_executor(0);
  std::mutex g_executor_mutex;
}

namespace rclcpp
{

using rclcpp::node::Node;
using rclcpp::rate::GenericRate;
using rclcpp::rate::WallRate;
using rclcpp::utilities::ok;
using rclcpp::utilities::init;

void spin_some(Node &node)
{
  std::lock_guard<std::mutex> lock(g_executor_mutex);
  if (!g_executor)
  {
    // Create an executor
    g_executor.reset(new rclcpp::executors::SingleThreadedExecutor());
  }
  g_executor->spin_node_some(node);
}

void spin_some(Node::SharedPtr &node_ptr)
{
  spin_some(*node_ptr);
}

void spin(Node::SharedPtr &node_ptr)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_ptr);
  executor.spin();
}

} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_RCLCPP_HPP_ */
