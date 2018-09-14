// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rcl_lifecycle/rcl_lifecycle.h"

namespace rclcpp_lifecycle
{
namespace node_interfaces
{

LifecycleNodeInterface::CallbackReturn
LifecycleNodeInterface::on_configure(const State &)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
LifecycleNodeInterface::on_cleanup(const State &)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
LifecycleNodeInterface::on_shutdown(const State &)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
LifecycleNodeInterface::on_activate(const State &)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
LifecycleNodeInterface::on_deactivate(const State &)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
LifecycleNodeInterface::on_error(const State &)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace node_interfaces
}  // namespace rclcpp_lifecycle
