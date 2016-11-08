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

#include <rclcpp_lifecycle/lifecycle_manager.hpp>

#include "lifecycle_manager_impl.hpp"  // implementation

namespace rclcpp
{
namespace lifecycle
{

using NodeInterfacePtr = std::shared_ptr<rclcpp::node::lifecycle::LifecycleNodeInterface>;

LifecycleManager::LifecycleManager():
  impl_(new LifecycleManagerImpl())
{}


LifecycleManager::~LifecycleManager() = default;

void
LifecycleManager::add_node_interface(const NodeInterfacePtr& node_interface)
{
  impl_->add_node_interface(node_interface);
}

void
LifecycleManager::add_node_interface(const NodeInterfacePtr& node_interface, rcl_state_machine_t custom_state_machine)
{
  impl_->add_node_interface(node_interface, custom_state_machine);
}

bool
LifecycleManager::register_on_configure(const std::string& node_name, std::function<bool(void)>& fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::CONFIGURING>(node_name, fcn);
}

bool
LifecycleManager::configure(const std::string& node_name)
{
  return impl_->change_state<lifecycle::LifecycleTransitionsT::CONFIGURING>(node_name);
}

bool
LifecycleManager::register_on_cleanup(const std::string& node_name, std::function<bool(void)>& fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::CLEANINGUP>(node_name, fcn);
}

bool
LifecycleManager::cleanup(const std::string& node_name)
{
  return impl_->change_state<lifecycle::LifecycleTransitionsT::CLEANINGUP>(node_name);
}

bool
LifecycleManager::register_on_shutdown(const std::string& node_name, std::function<bool(void)>& fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::SHUTTINGDOWN>(node_name, fcn);
}

bool
LifecycleManager::shutdown(const std::string& node_name)
{
  return impl_->change_state<lifecycle::LifecycleTransitionsT::SHUTTINGDOWN>(node_name);
}

bool
LifecycleManager::register_on_activate(const std::string& node_name, std::function<bool(void)>& fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::ACTIVATING>(node_name, fcn);
}

bool
LifecycleManager::activate(const std::string& node_name)
{
  return impl_->change_state<lifecycle::LifecycleTransitionsT::ACTIVATING>(node_name);
}

bool
LifecycleManager::register_on_deactivate(const std::string& node_name, std::function<bool(void)>& fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::DEACTIVATING>(node_name, fcn);
}

bool
LifecycleManager::deactivate(const std::string& node_name)
{
  return impl_->change_state<lifecycle::LifecycleTransitionsT::DEACTIVATING>(node_name);
}

}  // namespace lifecycle
}  // namespace rclcpp
