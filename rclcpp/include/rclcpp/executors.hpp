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

#ifndef RCLCPP__EXECUTORS_HPP_
#define RCLCPP__EXECUTORS_HPP_

#include <future>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Create a default single-threaded executor and execute any immediately available work.
// \param[in] node_ptr Shared pointer to the node to spin.
RCLCPP_PUBLIC
void
spin_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

/// Create a default single-threaded executor and spin the specified node.
// \param[in] node_ptr Shared pointer to the node to spin.
RCLCPP_PUBLIC
void
spin(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

namespace executors
{

using rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor;
using rclcpp::executors::single_threaded_executor::SingleThreadedExecutor;

/// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
/**
 * \param[in] executor The executor which will spin the node.
 * \param[in] node_ptr The node to spin.
 * \param[in] future The future to wait on. If SUCCESS, the future is safe to access after this function
 * \param[in] timeout Optional timeout parameter, which gets passed to Executor::spin_node_once.
   -1 is block forever, 0 is non-blocking.
   If the time spent inside the blocking loop exceeds this timeout, return a TIMEOUT return code.
 * \return The return code, one of SUCCESS, INTERRUPTED, or TIMEOUT.
 */
template<typename ResponseT, typename TimeT = std::milli>
rclcpp::executor::FutureReturnCode
spin_node_until_future_complete(
  rclcpp::executor::Executor & executor,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  std::shared_future<ResponseT> & future,
  std::chrono::duration<int64_t, TimeT> timeout = std::chrono::duration<int64_t, TimeT>(-1))
{
  // TODO(wjwwood): does not work recursively; can't call spin_node_until_future_complete
  // inside a callback executed by an executor.
  executor.add_node(node_ptr);
  auto retcode = executor.spin_until_future_complete(future, timeout);
  executor.remove_node(node_ptr);
  return retcode;
}

}  // namespace executors

template<typename FutureT, typename TimeT = std::milli>
rclcpp::executor::FutureReturnCode
spin_until_future_complete(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  std::shared_future<FutureT> & future,
  std::chrono::duration<int64_t, TimeT> timeout = std::chrono::duration<int64_t, TimeT>(-1))
{
  rclcpp::executors::SingleThreadedExecutor executor;
  return executors::spin_node_until_future_complete<FutureT>(executor, node_ptr, future, timeout);
}

}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS_HPP_
