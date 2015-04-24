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

#ifndef RCLCPP_RCLCPP_RCLCPP_HPP_
#define RCLCPP_RCLCPP_RCLCPP_HPP_

#include <csignal>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

namespace rclcpp
{

const std::chrono::seconds operator"" _s(unsigned long long s)
{
  return std::chrono::seconds(s);
}
const std::chrono::nanoseconds operator"" _s(long double s)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<long double>(s));
}

const std::chrono::nanoseconds
operator"" _ns(unsigned long long ns)
{
  return std::chrono::nanoseconds(ns);
}
const std::chrono::nanoseconds
operator"" _ns(long double ns)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<long double, std::nano>(ns));
}

using rclcpp::node::Node;
using rclcpp::publisher::Publisher;
using rclcpp::subscription::SubscriptionBase;
using rclcpp::subscription::Subscription;
using rclcpp::rate::GenericRate;
using rclcpp::rate::WallRate;
using rclcpp::timer::GenericTimer;
using rclcpp::timer::TimerBase;
using rclcpp::timer::WallTimer;
typedef rclcpp::context::Context::SharedPtr ContextSharedPtr;
using rclcpp::utilities::ok;
using rclcpp::utilities::shutdown;
using rclcpp::utilities::init;
using rclcpp::utilities::sleep_for;

void spin_some(Node::SharedPtr & node_ptr)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.spin_node_some(node_ptr);
}

void spin(Node::SharedPtr & node_ptr)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_ptr);
  executor.spin();
}

template<typename FutureT>
std::shared_future<FutureT> &
spin_until_future_complete(
  Node::SharedPtr & node_ptr, std::shared_future<FutureT> & future)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::executors::spin_node_until_future_complete<FutureT>(
    executor, node_ptr, future);
  return future;
}

} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_RCLCPP_HPP_ */
