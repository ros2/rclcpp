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

#ifndef LIFECYCLE_NODE_INTERFACE_IMPL_HPP_
#define LIFECYCLE_NODE_INTERFACE_IMPL_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rmw/types.h"

#include "lifecycle_node_state_manager.hpp"
#include "lifecycle_node_state_services_manager.hpp"
#include "lifecycle_node_entities_manager.hpp"

namespace rclcpp_lifecycle
{

class LifecycleNode::LifecycleNodeInterfaceImpl final
{
public:
  LifecycleNodeInterfaceImpl(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface);

  ~LifecycleNodeInterfaceImpl();

  void
  init(bool enable_communication_interface = true);

  bool
  register_callback(
    std::uint8_t lifecycle_transition,
    std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> & cb);

  const State &
  get_current_state() const;

  std::vector<State>
  get_available_states() const;

  std::vector<Transition>
  get_available_transitions() const;

  std::vector<Transition>
  get_transition_graph() const;

  rcl_ret_t
  change_state(
    std::uint8_t transition_id,
    node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code);

  const State &
  trigger_transition(uint8_t transition_id);

  const State &
  trigger_transition(
    uint8_t transition_id,
    node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code);

  const State & trigger_transition(const char * transition_label);

  const State & trigger_transition(
    const char * transition_label,
    node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code);

  void
  on_activate() const;

  void
  on_deactivate() const;

  void
  add_managed_entity(std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity);

  void
  add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer);

private:
  RCLCPP_DISABLE_COPY(LifecycleNodeInterfaceImpl)

  using NodeBasePtr = std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>;
  using NodeServicesPtr = std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>;
  using NodeTimersPtr = std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>;

  NodeBasePtr node_base_interface_;
  NodeServicesPtr node_services_interface_;
  NodeTimersPtr node_timers_interface_;

  std::shared_ptr<LifecycleNodeStateManager> state_manager_hdl_;
  std::unique_ptr<LifecycleNodeStateServicesManager> state_services_manager_hdl_;
  std::unique_ptr<LifecycleNodeEntitiesManager> managed_entities_manager_hdl_;
};

}  // namespace rclcpp_lifecycle
#endif  // LIFECYCLE_NODE_INTERFACE_IMPL_HPP_
