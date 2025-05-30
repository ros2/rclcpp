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

#include "rclcpp_action/types.hpp"

#include <string>

namespace rclcpp_action
{
std::string
to_string(const GoalUUID & goal_id)
{
  constexpr char HEX[] = "0123456789abcdef";
  std::string result;
  result.resize(36);
  size_t i = 0;
  for (uint8_t byte : goal_id) {
    result[i++] = HEX[byte >> 4];
    result[i++] = HEX[byte & 0x0f];
    // A RFC-4122 compliant UUID looks like:
    // 00000000-0000-0000-0000-000000000000
    // That means that there is a '-' at offset 8, 13, 18, and 23
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      result[i++] = '-';
    }
  }
  return result;
}

void
convert(const GoalUUID & goal_id, rcl_action_goal_info_t * info)
{
  if (info == nullptr) {
    throw std::invalid_argument("info is nullptr");
  }
  for (size_t i = 0; i < UUID_SIZE; ++i) {
    info->goal_id.uuid[i] = goal_id[i];
  }
}

void
convert(const rcl_action_goal_info_t & info, GoalUUID * goal_id)
{
  if (goal_id == nullptr) {
    throw std::invalid_argument("goal_id is nullptr");
  }
  for (size_t i = 0; i < UUID_SIZE; ++i) {
    (*goal_id)[i] = info.goal_id.uuid[i];
  }
}
}  // namespace rclcpp_action
