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

namespace rclcpp_lifecycle
{
namespace node_interfaces
{

bool
LifecycleNodeInterface::on_configure()
{
  return true;
}

bool
LifecycleNodeInterface::on_cleanup()
{
  return true;
}

bool
LifecycleNodeInterface::on_shutdown()
{
  return true;
}

bool
LifecycleNodeInterface::on_activate()
{
  return true;
}

bool
LifecycleNodeInterface::on_deactivate()
{
  return true;
}

bool
LifecycleNodeInterface::on_error()
{
  return false;
}

}  // namespace node_interfaces
}  // namespace rclcpp_lifecycle
