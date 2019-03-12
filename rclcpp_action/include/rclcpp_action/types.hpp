// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_ACTION__TYPES_HPP_
#define RCLCPP_ACTION__TYPES_HPP_

#include <rcl_action/types.h>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_info.hpp>

#include <climits>
#include <functional>
#include <string>

#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{

using GoalUUID = std::array<uint8_t, UUID_SIZE>;
using GoalStatus = action_msgs::msg::GoalStatus;
using GoalInfo = action_msgs::msg::GoalInfo;

/// Convert a goal id to a human readable string.
RCLCPP_ACTION_PUBLIC
std::string
to_string(const GoalUUID & goal_id);

// Convert C++ GoalID to rcl_action_goal_info_t
RCLCPP_ACTION_PUBLIC
void
convert(const GoalUUID & goal_id, rcl_action_goal_info_t * info);

// Convert rcl_action_goal_info_t to C++ GoalID
RCLCPP_ACTION_PUBLIC
void
convert(const rcl_action_goal_info_t & info, GoalUUID * goal_id);
}  // namespace rclcpp_action

namespace std
{
template<>
struct less<rclcpp_action::GoalUUID>
{
  bool operator()(
    const rclcpp_action::GoalUUID & lhs,
    const rclcpp_action::GoalUUID & rhs) const
  {
    return lhs < rhs;
  }
};

/// Hash a goal id so it can be used as a key in std::unordered_map
template<>
struct hash<rclcpp_action::GoalUUID>
{
  size_t operator()(const rclcpp_action::GoalUUID & uuid) const noexcept
  {
    // TODO(sloretz) Use someone else's hash function and cite it
    size_t result = 0;
    for (size_t i = 0; i < uuid.size(); ++i) {
      for (size_t b = 0; b < sizeof(size_t); ++b) {
        size_t part = uuid[i];
        part <<= CHAR_BIT * b;
        result ^= part;
      }
    }
    return result;
  }
};
}  // namespace std
#endif  // RCLCPP_ACTION__TYPES_HPP_
