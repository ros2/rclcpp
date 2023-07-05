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

#ifndef LIFECYCLE_NODE_STATE_SERVICES_MANAGER_HPP_
#define LIFECYCLE_NODE_STATE_SERVICES_MANAGER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rclcpp_lifecycle/transition.hpp"
#include "lifecycle_node_state_manager.hpp"

namespace rclcpp_lifecycle
{
class LifecycleNodeStateServicesManager
{
  using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
  using GetStateSrv = lifecycle_msgs::srv::GetState;
  using GetAvailableStatesSrv = lifecycle_msgs::srv::GetAvailableStates;
  using GetAvailableTransitionsSrv = lifecycle_msgs::srv::GetAvailableTransitions;
  using TransitionEventMsg = lifecycle_msgs::msg::TransitionEvent;

public:
  LifecycleNodeStateServicesManager(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    const std::weak_ptr<LifecycleNodeStateManager> state_manager_hdl);

  void send_change_state_resp(
    bool success,
    const std::shared_ptr<rmw_request_id_t> header) const;

private:
  void
  on_change_state(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<ChangeStateSrv::Request> req);

  void
  on_get_state(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetStateSrv::Request> req,
    std::shared_ptr<GetStateSrv::Response> resp) const;

  void
  on_get_available_states(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableStatesSrv::Request> req,
    std::shared_ptr<GetAvailableStatesSrv::Response> resp) const;

  void
  on_get_available_transitions(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
    std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const;

  void
  on_get_transition_graph(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
    std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const;

  void
  copy_transitions_vector_to_resp(
    const std::vector<Transition> transition_vec,
    std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const;

  using ChangeStateSrvPtr = std::shared_ptr<rclcpp::Service<ChangeStateSrv>>;
  using GetStateSrvPtr = std::shared_ptr<rclcpp::Service<GetStateSrv>>;
  using GetAvailableStatesSrvPtr =
    std::shared_ptr<rclcpp::Service<GetAvailableStatesSrv>>;
  using GetAvailableTransitionsSrvPtr =
    std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>;
  using GetTransitionGraphSrvPtr =
    std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>;

  ChangeStateSrvPtr srv_change_state_;
  GetStateSrvPtr srv_get_state_;
  GetAvailableStatesSrvPtr srv_get_available_states_;
  GetAvailableTransitionsSrvPtr srv_get_available_transitions_;
  GetTransitionGraphSrvPtr srv_get_transition_graph_;

  std::weak_ptr<LifecycleNodeStateManager> state_manager_hdl_;
};

}  // namespace rclcpp_lifecycle
#endif  // LIFECYCLE_NODE_STATE_SERVICES_MANAGER_HPP_
