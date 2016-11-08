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

#ifndef RCLCPP__LIFECYCLE_MANAGER_IMPL_H_
#define RCLCPP__LIFECYCLE_MANAGER_IMPL_H_

#include <memory>
#include <vector>
#include <map>

#include <rcl_lifecycle/lifecycle_state.h>
#include <rcl_lifecycle/default_state_machine.h>

namespace rclcpp
{
namespace lifecycle
{

using NodeInterfacePtr = std::shared_ptr<node::lifecycle::LifecycleNodeInterface>;
using NodeInterfaceWeakPtr = std::weak_ptr<node::lifecycle::LifecycleNodeInterface>;

struct NodeStateMachine
{
  NodeInterfaceWeakPtr weak_node_handle;
  rcl_state_machine_t state_machine;
  std::map<LifecycleTransitionsT, std::function<bool(void)>> cb_map;
};

class LifecycleManager::LifecycleManagerImpl
{
public:
  LifecycleManagerImpl() = default;
  ~LifecycleManagerImpl() = default;

  LIFECYCLE_EXPORT
  void
  add_node_interface(const NodeInterfacePtr& node_interface)
  {
    rcl_state_machine_t state_machine = rcl_get_default_state_machine();
    add_node_interface(node_interface, state_machine);
  }

  LIFECYCLE_EXPORT
  void
  add_node_interface(const NodeInterfacePtr& node_interface, rcl_state_machine_t custom_state_machine)
  {
    NodeStateMachine node_state_machine;
    node_state_machine.weak_node_handle = node_interface;
    // TODO(karsten1987): Find a way to make this generic to an enduser
    node_state_machine.state_machine = custom_state_machine;

    // register default callbacks
    // maybe optional
    using NodeInterface = node::lifecycle::LifecycleNodeInterface;
    std::function<bool(void)> cb_configuring = std::bind(&NodeInterface::on_configure, node_interface);
    std::function<bool(void)> cb_cleaningup = std::bind(&NodeInterface::on_cleanup, node_interface);
    std::function<bool(void)> cb_shuttingdown = std::bind(&NodeInterface::on_shutdown, node_interface);
    std::function<bool(void)> cb_activating = std::bind(&NodeInterface::on_activate, node_interface);
    std::function<bool(void)> cb_deactivating = std::bind(&NodeInterface::on_deactivate, node_interface);
    std::function<bool(void)> cb_error = std::bind(&NodeInterface::on_error, node_interface);
    node_state_machine.cb_map[LifecycleTransitionsT::CONFIGURING] = cb_configuring;
    node_state_machine.cb_map[LifecycleTransitionsT::CLEANINGUP] = cb_cleaningup;
    node_state_machine.cb_map[LifecycleTransitionsT::SHUTTINGDOWN] = cb_shuttingdown;
    node_state_machine.cb_map[LifecycleTransitionsT::ACTIVATING] = cb_activating;
    node_state_machine.cb_map[LifecycleTransitionsT::DEACTIVATING] = cb_deactivating;
    node_state_machine.cb_map[LifecycleTransitionsT::ERRORPROCESSING] = cb_error;

    // TODO(karsten1987): clarify what do if node already exists
    auto node_name = node_interface->get_name();
    node_handle_map_[node_name] = node_state_machine;
  }

  template<LifecycleTransitionsT lifecycle_transition>
  bool
  register_callback(const std::string& node_name, std::function<bool(void)>& cb)
  {
    if (node_name.empty())
      return false;

    auto node_handle_iter = node_handle_map_.find(node_name);
    if (node_handle_iter == node_handle_map_.end())
    {
      fprintf(stderr, "Node with name %s is not registered\n", node_name.c_str());
    }
    node_handle_iter->second.cb_map[lifecycle_transition] = cb;
    return true;
  }

  template<LifecycleTransitionsT lifecycle_transition>
  bool
  change_state(const std::string& node_name = "")
  {
    if (node_name.empty())
      return false;

    auto node_handle_iter = node_handle_map_.find(node_name);
    if (node_handle_iter == node_handle_map_.end())
    {
      fprintf(stderr, "Node with name %s is not registered\n", node_name.c_str());
      return false;
    }

    auto node_handle = node_handle_iter->second.weak_node_handle.lock();
    if (!node_handle)
      return false;

    // ask RCL if this is a valid state
    const rcl_state_transition_t* transition
     = rcl_is_valid_transition_by_index(&node_handle_iter->second.state_machine, static_cast<std::uint8_t>(lifecycle_transition));
    if (transition == NULL)
      return false;

    std::function<bool(void)> callback = node_handle_iter->second.cb_map[lifecycle_transition];
    if(!callback())
    {
      node_handle->on_error();
      node_handle_iter->second.state_machine.current_state = &rcl_state_errorprocessing;
      return false;
    }
    node_handle_iter->second.state_machine.current_state = transition->goal;
    return true;
  }

private:
  std::map<std::string, NodeStateMachine> node_handle_map_;
};

}  // namespace lifecycle
}  // namespace rclcpp
#endif
