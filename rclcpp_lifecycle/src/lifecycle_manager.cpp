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

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_manager.hpp"

#include "lifecycle_manager_impl.hpp"  // implementation

namespace rclcpp
{
namespace lifecycle
{

LifecycleManager::LifecycleManager()
: impl_(new LifecycleManagerImpl())
{
  impl_->init();
}

LifecycleManager::~LifecycleManager() = default;

std::shared_ptr<rclcpp::node::Node>
LifecycleManager::get_node_base_interface()
{
  return impl_->node_base_handle_;
}

void
LifecycleManager::add_node_interface(const NodePtr & node)
{
  add_node_interface(node->get_base_interface()->get_name(), node);
}

void
LifecycleManager::add_node_interface(const std::string & node_name,
  const NodeInterfacePtr & node_interface)
{
  impl_->add_node_interface(node_name, node_interface);
}

bool
LifecycleManager::register_on_configure(const std::string & node_name,
  std::function<bool(void)> & fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::CONFIGURING>(node_name, fcn);
}

bool
LifecycleManager::configure(const std::string & node_name)
{
  return impl_->change_state(node_name, lifecycle::LifecycleTransitionsT::CONFIGURING);
}

bool
LifecycleManager::register_on_cleanup(const std::string & node_name,
  std::function<bool(void)> & fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::CLEANINGUP>(node_name, fcn);
}

bool
LifecycleManager::cleanup(const std::string & node_name)
{
  return impl_->change_state(node_name, lifecycle::LifecycleTransitionsT::CLEANINGUP);
}

bool
LifecycleManager::register_on_shutdown(const std::string & node_name,
  std::function<bool(void)> & fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::SHUTTINGDOWN>(node_name, fcn);
}

bool
LifecycleManager::shutdown(const std::string & node_name)
{
  return impl_->change_state(node_name, lifecycle::LifecycleTransitionsT::SHUTTINGDOWN);
}

bool
LifecycleManager::register_on_activate(const std::string & node_name,
  std::function<bool(void)> & fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::ACTIVATING>(node_name, fcn);
}

bool
LifecycleManager::activate(const std::string & node_name)
{
  return impl_->change_state(node_name, lifecycle::LifecycleTransitionsT::ACTIVATING);
}

bool
LifecycleManager::register_on_deactivate(const std::string & node_name,
  std::function<bool(void)> & fcn)
{
  return impl_->register_callback<lifecycle::LifecycleTransitionsT::DEACTIVATING>(node_name, fcn);
}

bool
LifecycleManager::deactivate(const std::string & node_name)
{
  return impl_->change_state(node_name, lifecycle::LifecycleTransitionsT::DEACTIVATING);
}

}  // namespace lifecycle
}  // namespace rclcpp
