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

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/transition_description.hpp"
#include "lifecycle_msgs/msg/transition_event.h"  // for getting the c-typesupport
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rcl/error_handling.h"
#include "rcl/node.h"

#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rcl_lifecycle/transition_map.h"

#include "rcutils/logging_macros.h"

#include "rmw/types.h"

#include "lifecycle_node_interface_impl.hpp"

namespace rclcpp_lifecycle
{

LifecycleNode::LifecycleNodeInterfaceImpl::LifecycleNodeInterfaceImpl(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface)
: node_base_interface_(node_base_interface),
  node_services_interface_(node_services_interface)
{
}

LifecycleNode::LifecycleNodeInterfaceImpl::~LifecycleNodeInterfaceImpl()
{
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::init(bool enable_communication_interface)
{
  state_manager_hdl_ = std::make_shared<LifecycleNodeStateManager>();
  state_manager_hdl_->init(
    node_base_interface_,
    enable_communication_interface
  );
  if (enable_communication_interface) {
    state_services_manager_hdl_ = std::make_unique<LifecycleNodeStateServicesManager>(
      node_base_interface_,
      node_services_interface_,
      state_manager_hdl_
    );
  }
  managed_entities_manager_hdl_ = std::make_unique<LifecycleNodeEntitiesManager>();
}

bool
LifecycleNode::LifecycleNodeInterfaceImpl::register_callback(
  std::uint8_t lifecycle_transition,
  std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> & cb)
{
  return state_manager_hdl_->register_callback(lifecycle_transition, cb);
}

const State &
LifecycleNode::LifecycleNodeInterfaceImpl::get_current_state() const
{
  return state_manager_hdl_->get_current_state();
}

std::vector<State>
LifecycleNode::LifecycleNodeInterfaceImpl::get_available_states() const
{
  return state_manager_hdl_->get_available_states();
}

std::vector<Transition>
LifecycleNode::LifecycleNodeInterfaceImpl::get_available_transitions() const
{
  return state_manager_hdl_->get_available_transitions();
}

std::vector<Transition>
LifecycleNode::LifecycleNodeInterfaceImpl::get_transition_graph() const
{
  return state_manager_hdl_->get_transition_graph();
}

rcl_ret_t
LifecycleNode::LifecycleNodeInterfaceImpl::change_state(
  std::uint8_t transition_id,
  node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  return state_manager_hdl_->change_state(transition_id, cb_return_code);
}

const State & LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
  const char * transition_label)
{
  node_interfaces::LifecycleNodeInterface::CallbackReturn error;
  return trigger_transition(transition_label, error);
}

const State & LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
  const char * transition_label,
  node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  const rcl_lifecycle_transition_t * transition =
    state_manager_hdl_->get_transition_by_label(transition_label);

  if (transition) {
    state_manager_hdl_->change_state(static_cast<uint8_t>(transition->id), cb_return_code);
  }
  return get_current_state();
}

const State &
LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(uint8_t transition_id)
{
  node_interfaces::LifecycleNodeInterface::CallbackReturn error;
  state_manager_hdl_->change_state(transition_id, error);
  (void) error;
  return get_current_state();
}

const State &
LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
  uint8_t transition_id,
  node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  state_manager_hdl_->change_state(transition_id, cb_return_code);
  return get_current_state();
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::add_managed_entity(
  std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity)
{
  managed_entities_manager_hdl_->add_managed_entity(managed_entity);
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::add_timer_handle(
  std::shared_ptr<rclcpp::TimerBase> timer)
{
  managed_entities_manager_hdl_->add_timer_handle(timer);
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_activate() const
{
  managed_entities_manager_hdl_->on_activate();
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_deactivate() const
{
  managed_entities_manager_hdl_->on_deactivate();
}

}  // namespace rclcpp_lifecycle
