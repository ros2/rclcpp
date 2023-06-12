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

#include <vector>
#include <string>
#include <utility>

#include "lifecycle_node_state_services_manager.hpp"
#include "lifecycle_node_state_manager.hpp"

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace rclcpp_lifecycle
{

LifecycleNodeStateServicesManager::LifecycleNodeStateServicesManager(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
  const std::weak_ptr<LifecycleNodeStateManager> state_manager_hdl
)
: state_manager_hdl_(state_manager_hdl)
{
  if (auto state_manager_hdl = state_manager_hdl_.lock()) {
    rcl_lifecycle_com_interface_t & state_machine_com_interface =
      state_manager_hdl->get_rcl_com_interface();
    { // change_state
      auto cb = std::bind(
        &LifecycleNodeStateServicesManager::on_change_state, this,
        std::placeholders::_1, std::placeholders::_2);
      rclcpp::AnyServiceCallback<ChangeStateSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_change_state_ = std::make_shared<rclcpp::Service<ChangeStateSrv>>(
        node_base_interface->get_shared_rcl_node_handle(),
        &state_machine_com_interface.srv_change_state,
        any_cb);
      node_services_interface->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_change_state_),
        nullptr);
    }

    { // get_state
      auto cb = std::bind(
        &LifecycleNodeStateServicesManager::on_get_state, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetStateSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_state_ = std::make_shared<rclcpp::Service<GetStateSrv>>(
        node_base_interface->get_shared_rcl_node_handle(),
        &state_machine_com_interface.srv_get_state,
        any_cb);
      node_services_interface->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_state_),
        nullptr);
    }

    { // get_available_states
      auto cb = std::bind(
        &LifecycleNodeStateServicesManager::on_get_available_states, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableStatesSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_available_states_ = std::make_shared<rclcpp::Service<GetAvailableStatesSrv>>(
        node_base_interface->get_shared_rcl_node_handle(),
        &state_machine_com_interface.srv_get_available_states,
        any_cb);
      node_services_interface->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_states_),
        nullptr);
    }

    { // get_available_transitions
      auto cb = std::bind(
        &LifecycleNodeStateServicesManager::on_get_available_transitions, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_available_transitions_ =
        std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
        node_base_interface->get_shared_rcl_node_handle(),
        &state_machine_com_interface.srv_get_available_transitions,
        any_cb);
      node_services_interface->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_transitions_),
        nullptr);
    }

    { // get_transition_graph
      auto cb = std::bind(
        &LifecycleNodeStateServicesManager::on_get_transition_graph, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_transition_graph_ =
        std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
        node_base_interface->get_shared_rcl_node_handle(),
        &state_machine_com_interface.srv_get_transition_graph,
        any_cb);
      node_services_interface->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_transition_graph_),
        nullptr);
    }

    { // cancel_transition
      auto cb = std::bind(
        &LifecycleNodeStateServicesManager::on_cancel_transition, this,
        std::placeholders::_1, std::placeholders::_2);
      rclcpp::AnyServiceCallback<CancelTransitionSrv> any_cb;
      any_cb.set(std::move(cb));

      rcl_service_options_t cancel_srv_options = rcl_service_get_default_options();

      srv_cancel_transition_ =
        std::make_shared<rclcpp::Service<CancelTransitionSrv>>(
        node_base_interface->get_shared_rcl_node_handle(),
        std::string(node_base_interface->get_fully_qualified_name()) + "/cancel_transition",
        any_cb,
        cancel_srv_options);

      node_services_interface->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_cancel_transition_),
        nullptr);
    }
  }
}

void
LifecycleNodeStateServicesManager::send_change_state_resp(
  bool success,
  const std::shared_ptr<rmw_request_id_t> header) const
{
  auto resp = std::make_unique<ChangeStateSrv::Response>();
  resp->success = success;
  srv_change_state_->send_response(*header, *resp);
}

void
LifecycleNodeStateServicesManager::send_cancel_transition_resp(
  const std::string & error_msg,
  bool success,
  const std::shared_ptr<rmw_request_id_t> header) const
{
  auto resp = std::make_unique<CancelTransitionSrv::Response>();
  resp->success = success;
  resp->error_msg = error_msg;
  srv_cancel_transition_->send_response(*header, *resp);
}

void
LifecycleNodeStateServicesManager::on_change_state(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<ChangeStateSrv::Request> req)
{
  auto state_hdl = state_manager_hdl_.lock();
  if (state_hdl) {
    state_hdl->throw_runtime_error_on_uninitialized_state_machine("change state");
    int transition_id = state_hdl->get_transition_id_from_request(req);
    if (transition_id < 0) {
      send_change_state_resp(false, header);
    } else {
      state_hdl->change_state(
        transition_id,
        std::bind(
          &LifecycleNodeStateServicesManager::send_change_state_resp,
          this, std::placeholders::_1, std::placeholders::_2),
        header);
    }
  } else { /*Unable to lock StateManager*/
    send_change_state_resp(false, header);
  }
}

void
LifecycleNodeStateServicesManager::on_get_state(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<GetStateSrv::Request> req,
  std::shared_ptr<GetStateSrv::Response> resp) const
{
  (void)header;
  (void)req;
  auto state_hdl = state_manager_hdl_.lock();
  if (state_hdl) {
    state_hdl->throw_runtime_error_on_uninitialized_state_machine("get state");
    const State & current_state = state_hdl->get_current_state();
    resp->current_state.id = current_state.id();
    resp->current_state.label = current_state.label();
  }
}

void
LifecycleNodeStateServicesManager::on_get_available_states(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<GetAvailableStatesSrv::Request> req,
  std::shared_ptr<GetAvailableStatesSrv::Response> resp) const
{
  (void)header;
  (void)req;
  auto state_hdl = state_manager_hdl_.lock();
  if (state_hdl) {
    state_hdl->throw_runtime_error_on_uninitialized_state_machine("get available states");
    std::vector<State> available_states = state_hdl->get_available_states();
    resp->available_states.resize(available_states.size());
    for (unsigned int i = 0; i < available_states.size(); ++i) {
      resp->available_states[i].id = available_states[i].id();
      resp->available_states[i].label = available_states[i].label();
    }
  }
}

void
LifecycleNodeStateServicesManager::on_get_available_transitions(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
  std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const
{
  (void)header;
  (void)req;
  auto state_hdl = state_manager_hdl_.lock();
  if (state_hdl) {
    state_hdl->throw_runtime_error_on_uninitialized_state_machine("get available transitions");
    std::vector<Transition> available_transitions = state_hdl->get_available_transitions();
    copy_transitions_vector_to_resp(available_transitions, resp);
  }
}

void
LifecycleNodeStateServicesManager::on_get_transition_graph(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
  std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const
{
  (void)header;
  (void)req;
  auto state_hdl = state_manager_hdl_.lock();
  if (state_hdl) {
    state_hdl->throw_runtime_error_on_uninitialized_state_machine("get transition graph");
    std::vector<Transition> available_transitions = state_hdl->get_transition_graph();
    copy_transitions_vector_to_resp(available_transitions, resp);
  }
}

void
LifecycleNodeStateServicesManager::on_cancel_transition(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<CancelTransitionSrv::Request> req) const
{
  (void) req;
  auto state_hdl = state_manager_hdl_.lock();
  if (state_hdl) {
    state_hdl->throw_runtime_error_on_uninitialized_state_machine("cancel transition");
    state_hdl->cancel_transition(
      std::bind(
        &LifecycleNodeStateServicesManager::send_cancel_transition_resp, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      header);
  } else {
    send_cancel_transition_resp("LifecycleNodeStateManager is not available.", false, header);
  }
}

void
LifecycleNodeStateServicesManager::copy_transitions_vector_to_resp(
  const std::vector<Transition> transition_vec,
  std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const
{
  resp->available_transitions.resize(transition_vec.size());
  for (unsigned int i = 0; i < transition_vec.size(); ++i) {
    lifecycle_msgs::msg::TransitionDescription & trans_desc = resp->available_transitions[i];

    Transition transition = transition_vec[i];
    trans_desc.transition.id = transition.id();
    trans_desc.transition.label = transition.label();
    trans_desc.start_state.id = transition.start_state().id();
    trans_desc.start_state.label = transition.start_state().label();
    trans_desc.goal_state.id = transition.goal_state().id();
    trans_desc.goal_state.label = transition.goal_state().label();
  }
}

}  // namespace rclcpp_lifecycle
