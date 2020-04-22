// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTOR_POLICIES__SCHEDULING_RESULT_HPP_
#define RCLCPP__EXECUTOR_POLICIES__SCHEDULING_RESULT_HPP_

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executor_policies
{

/// Represents the directions a SchedulingPolicy can give to an Executor.
enum RCLCPP_PUBLIC SchedulingResult
{
  ContinueExecuting,  //<! Indicates more work should be done before waiting.
  WaitForWork,  //<! Indicates that the executor should wait on the wait set again.
};

}  // namespace executor_policies
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_POLICIES__SCHEDULING_RESULT_HPP_
