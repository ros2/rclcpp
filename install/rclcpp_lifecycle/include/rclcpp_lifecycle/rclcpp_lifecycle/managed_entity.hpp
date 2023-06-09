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

#ifndef RCLCPP_LIFECYCLE__MANAGED_ENTITY_HPP_
#define RCLCPP_LIFECYCLE__MANAGED_ENTITY_HPP_

#include <atomic>

#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_lifecycle
{

///  Base class for lifecycle entities, like `LifecyclePublisher`.
class ManagedEntityInterface
{
public:
  RCLCPP_LIFECYCLE_PUBLIC
  virtual
  ~ManagedEntityInterface() {}

  RCLCPP_LIFECYCLE_PUBLIC
  virtual
  void
  on_activate() = 0;

  RCLCPP_LIFECYCLE_PUBLIC
  virtual
  void
  on_deactivate() = 0;
};

/// A simple implementation of `ManagedEntityInterface`, which toogles a flag.
class SimpleManagedEntity : public ManagedEntityInterface
{
public:
  RCLCPP_LIFECYCLE_PUBLIC
  ~SimpleManagedEntity() override = default;

  RCLCPP_LIFECYCLE_PUBLIC
  void
  on_activate() override;

  RCLCPP_LIFECYCLE_PUBLIC
  void
  on_deactivate() override;

  RCLCPP_LIFECYCLE_PUBLIC
  bool
  is_activated() const;

private:
  std::atomic<bool> activated_ = false;
};

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__MANAGED_ENTITY_HPP_
