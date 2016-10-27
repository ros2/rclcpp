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

#ifndef RCLCPP__LIFECYCLE_EXECUTOR_HPP_
#define RCLCPP__LIFECYCLE_EXECUTOR_HPP_

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/type_traits/is_manageable_node.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executors
{

class LifecycleExecutor : public single_threaded_executor::SingleThreadedExecutor
{
//  using Node = rclcpp::node::Node;

public:

  explicit LifecycleExecutor(const executor::ExecutorArgs & args = executor::create_default_executor_arguments())
    : SingleThreadedExecutor(args)
  {}

  //RCLCPP_PUBLIC
  template<class T, typename std::enable_if<is_manageable_node<T>::value>::type>
  void
  add_node(std::shared_ptr<T> node_ptr, bool notify = true);
};

} // namespace executor
} // namespace rclcpp

#endif
