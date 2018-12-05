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

#include <functional>


namespace rclcpp_action
{

using GoalID = std::array<uint8_t, UUID_SIZE>;
using GoalStatus = action_msgs::msg::GoalStatus;
using GoalInfo = action_msgs::msg::GoalInfo;

}  // namespace rclcpp_action

namespace std
{
template<>
struct less<rclcpp_action::GoalID>
{
  bool operator()(
    const rclcpp_action::GoalID & lhs,
    const rclcpp_action::GoalID & rhs) const
  {
    return lhs < rhs;
  }
};
}  // namespace std
#endif  // RCLCPP_ACTION__TYPES_HPP_
