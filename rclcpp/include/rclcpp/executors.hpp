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

#ifndef RCLCPP_RCLCPP_EXECUTORS_HPP_
#define RCLCPP_RCLCPP_EXECUTORS_HPP_

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace rclcpp
{
namespace executors
{

using rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor;
using rclcpp::executors::single_threaded_executor::SingleThreadedExecutor;

} // namespace executors
} // namespace rclcpp

#endif /* RCLCPP_RCLCPP_EXECUTORS_HPP_ */
