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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__SYNCHRONIZATION_POLICY_COMMON_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__SYNCHRONIZATION_POLICY_COMMON_HPP_

#include <chrono>
#include <functional>

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

/// Common structure for synchronization policies.
class SynchronizationPolicyCommon
{
protected:
  SynchronizationPolicyCommon() = default;
  ~SynchronizationPolicyCommon() = default;

  std::function<bool()>
  create_loop_predicate(
    std::chrono::nanoseconds time_to_wait_ns,
    std::chrono::steady_clock::time_point start)
  {
    if (time_to_wait_ns >= std::chrono::nanoseconds(0)) {
      // If time_to_wait_ns is >= 0 schedule against a deadline.
      auto deadline = start + time_to_wait_ns;
      return [deadline]() -> bool {return std::chrono::steady_clock::now() < deadline;};
    } else {
      // In the case of time_to_wait_ns < 0, just always return true to loop forever.
      return []() -> bool {return true;};
    }
  }

  std::chrono::nanoseconds
  calculate_time_left_to_wait(
    std::chrono::nanoseconds original_time_to_wait_ns,
    std::chrono::steady_clock::time_point start)
  {
    std::chrono::nanoseconds time_left_to_wait;
    if (original_time_to_wait_ns < std::chrono::nanoseconds(0)) {
      time_left_to_wait = original_time_to_wait_ns;
    } else {
      time_left_to_wait = original_time_to_wait_ns - (std::chrono::steady_clock::now() - start);
      if (time_left_to_wait < std::chrono::nanoseconds(0)) {
        time_left_to_wait = std::chrono::nanoseconds(0);
      }
    }
    return time_left_to_wait;
  }
};

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__SYNCHRONIZATION_POLICY_COMMON_HPP_
