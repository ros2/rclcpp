// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef LIFECYCLE_NODE_ENTITIES_MANAGER_HPP_
#define LIFECYCLE_NODE_ENTITIES_MANAGER_HPP_

#include <vector>
#include <memory>
#include "rclcpp_lifecycle/managed_entity.hpp"
#include <rclcpp/timer.hpp>

namespace rclcpp_lifecycle
{

class LifecycleNodeEntitiesManager
{
public:
  void
  on_activate() const;

  void
  on_deactivate() const;

  void
  add_managed_entity(std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity);

  void
  add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer);

private:
  std::vector<std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface>> weak_managed_entities_;
  std::vector<std::weak_ptr<rclcpp::TimerBase>> weak_timers_;
};

}  // namespace rclcpp_lifecycle
#endif  // LIFECYCLE_NODE_ENTITIES_MANAGER_HPP_
