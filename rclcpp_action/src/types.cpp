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
#include <sstream>

namespace rclcpp_action
{
std::string
to_string(const GoalUUID & goal_id)
{
  std::stringstream stream;
  stream << std::hex;
  for (const auto & element : goal_id) {
    stream << static_cast<int>(element);
  }
  return stream.str();
}

void
convert(const GoalUUID & goal_id, rcl_action_goal_info_t * info)
{
  for (size_t i = 0; i < 16; ++i) {
    info->goal_id.uuid[i] = goal_id[i];
  }
}

void
convert(const rcl_action_goal_info_t & info, GoalUUID * goal_id)
{
  for (size_t i = 0; i < 16; ++i) {
    (*goal_id)[i] = info.goal_id.uuid[i];
  }
}
}  // namespace rclcpp_action
