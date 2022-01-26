// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "rclcpp_lifecycle/managed_entity.hpp"

namespace rclcpp_lifecycle
{

void SimpleManagedEntity::on_activate()
{
  activated_.store(true);
}

void SimpleManagedEntity::on_deactivate()
{
  activated_.store(false);
}

bool SimpleManagedEntity::is_activated() const
{
  return activated_.load();
}

}  // namespace rclcpp_lifecycle
