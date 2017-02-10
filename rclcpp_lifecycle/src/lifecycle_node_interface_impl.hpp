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
#include <string>
#include <vector>
#include <utility>

#include "lifecycle_msgs/msg/transition_description.hpp"
#include "lifecycle_msgs/msg/transition_event.h"  // for getting the c-typesupport
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"

#include "rcl/error_handling.h"

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

namespace rclcpp_lifecycle
{

class LifecycleNode::LifecycleNodeInterfaceImpl
{
  using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
  using GetStateSrv = lifecycle_msgs::srv::GetState;
  using GetAvailableStatesSrv = lifecycle_msgs::srv::GetAvailableStates;
  using GetAvailableTransitionsSrv = lifecycle_msgs::srv::GetAvailableTransitions;
  using TransitionEventMsg = lifecycle_msgs::msg::TransitionEvent;

public:
  LifecycleNodeInterfaceImpl(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface)
  : node_base_interface_(node_base_interface),
    node_services_interface_(node_services_interface)
  {}

  ~LifecycleNodeInterfaceImpl()
  {
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      fprintf(stderr, "%s:%u, FATAL: rcl_state_machine got destroyed externally.\n",
        __FILE__, __LINE__);
    } else {
      rcl_lifecycle_state_machine_fini(&state_machine_,
        node_base_interface_->get_rcl_node_handle());
    }
  }

  void
  init()
  {
    state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();
    // The call to initialize the state machine takes
    // currently five different typesupports for all publishers/services
    // created within the RCL_LIFECYCLE structure.
    // The publisher takes a C-Typesupport since the publishing (i.e. creating
    // the message) is done fully in RCL.
    // Services are handled in C++, so that it needs a C++ typesupport structure.
    rcl_ret_t ret = rcl_lifecycle_state_machine_init(
      &state_machine_, node_base_interface_->get_rcl_node_handle(),
      ROSIDL_GET_MSG_TYPE_SUPPORT(lifecycle_msgs, msg, TransitionEvent),
      rosidl_typesupport_cpp::get_service_type_support_handle<ChangeStateSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetStateSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableStatesSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableTransitionsSrv>(),
      true);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error(
              std::string(
                "Couldn't initialize state machine for node ") + node_base_interface_->get_name());
    }

    {  // change_state
      auto cb = std::bind(&LifecycleNodeInterfaceImpl::on_change_state, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::any_service_callback::AnyServiceCallback<ChangeStateSrv> any_cb;
      any_cb.set(cb);

      srv_change_state_ = std::make_shared<rclcpp::service::Service<ChangeStateSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_change_state,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::service::ServiceBase>(srv_change_state_),
        nullptr);
    }

    {  // get_state
      auto cb = std::bind(&LifecycleNodeInterfaceImpl::on_get_state, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::any_service_callback::AnyServiceCallback<GetStateSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_state_ = std::make_shared<rclcpp::service::Service<GetStateSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_get_state,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::service::ServiceBase>(srv_get_state_),
        nullptr);
    }

    {  // get_available_states
      auto cb = std::bind(&LifecycleNodeInterfaceImpl::on_get_available_states, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::any_service_callback::AnyServiceCallback<GetAvailableStatesSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_available_states_ = std::make_shared<rclcpp::service::Service<GetAvailableStatesSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_get_available_states,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::service::ServiceBase>(srv_get_available_states_),
        nullptr);
    }

    {  // get_available_transitions
      auto cb = std::bind(&LifecycleNodeInterfaceImpl::on_get_available_transitions, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::any_service_callback::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_available_transitions_ =
        std::make_shared<rclcpp::service::Service<GetAvailableTransitionsSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_get_available_transitions,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::service::ServiceBase>(srv_get_available_transitions_),
        nullptr);
    }
  }

  bool
  register_callback(std::uint8_t lifecycle_transition, std::function<rcl_lifecycle_ret_t(
      const State &)> & cb)
  {
    cb_map_[lifecycle_transition] = cb;
    return true;
  }

  void
  on_change_state(const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<ChangeStateSrv::Request> req,
    std::shared_ptr<ChangeStateSrv::Response> resp)
  {
    (void)header;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error(
              "Can't get state. State machine is not initialized.");
    }
    resp->success = change_state(req->transition.id);
  }

  void
  on_get_state(const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetStateSrv::Request> req,
    std::shared_ptr<GetStateSrv::Response> resp)
  {
    (void)header;
    (void)req;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error(
              "Can't get state. State machine is not initialized.");
    }
    resp->current_state.id = static_cast<uint8_t>(state_machine_.current_state->id);
    resp->current_state.label = state_machine_.current_state->label;
  }

  void
  on_get_available_states(const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableStatesSrv::Request> req,
    std::shared_ptr<GetAvailableStatesSrv::Response> resp)
  {
    (void)header;
    (void)req;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error(
              "Can't get available states. State machine is not initialized.");
    }
    for (uint8_t i = 0; i < state_machine_.transition_map.states_size; ++i) {
      lifecycle_msgs::msg::State state;
      state.id = static_cast<uint8_t>(state_machine_.transition_map.states[i].id);
      state.label = static_cast<std::string>(state_machine_.transition_map.states[i].label);
      resp->available_states.push_back(state);
    }
  }

  void
  on_get_available_transitions(const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
    std::shared_ptr<GetAvailableTransitionsSrv::Response> resp)
  {
    (void)header;
    (void)req;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error(
              "Can't get available transitions. State machine is not initialized.");
      return;
    }

    for (uint8_t i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
      rcl_lifecycle_transition_t & rcl_transition = state_machine_.transition_map.transitions[i];
      lifecycle_msgs::msg::TransitionDescription trans_desc;
      trans_desc.transition.id = rcl_transition.id;
      trans_desc.transition.label = rcl_transition.label;
      trans_desc.start_state.id = rcl_transition.start->id;
      trans_desc.start_state.label = rcl_transition.start->label;
      trans_desc.goal_state.id = rcl_transition.goal->id;
      trans_desc.goal_state.label = rcl_transition.goal->label;
      resp->available_transitions.push_back(trans_desc);
    }
  }

  const State &
  get_current_state()
  {
    current_state_ = State(state_machine_.current_state);
    return current_state_;
  }

  std::vector<State>
  get_available_states()
  {
    std::vector<State> states;
    for (uint8_t i = 0; i < state_machine_.transition_map.states_size; ++i) {
      State state(&state_machine_.transition_map.states[i]);
      states.push_back(state);
    }
    return states;
  }

  std::vector<Transition>
  get_available_transitions()
  {
    std::vector<Transition> transitions;

    for (uint8_t i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
      Transition transition(
        &state_machine_.transition_map.transitions[i]);
      transitions.push_back(transition);
    }
    return transitions;
  }

  bool
  change_state(std::uint8_t lifecycle_transition)
  {
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      fprintf(stderr, "%s:%d, Unable to change state for state machine for %s: %s \n",
        __FILE__, __LINE__, node_base_interface_->get_name(), rcl_get_error_string_safe());
      return false;
    }

    // keep the initial state to pass to a transition callback
    State initial_state(state_machine_.current_state);

    uint8_t transition_id = lifecycle_transition;
    if (rcl_lifecycle_trigger_transition(&state_machine_, transition_id, true) != RCL_RET_OK) {
      fprintf(stderr, "%s:%d, Unable to start transition %u from current state %s: %s\n",
        __FILE__, __LINE__, transition_id,
        state_machine_.current_state->label, rcl_get_error_string_safe());
      return false;
    }

    rcl_lifecycle_ret_t cb_success = execute_callback(
      state_machine_.current_state->id, initial_state);

    if (rcl_lifecycle_trigger_transition(
        &state_machine_, cb_success, true) != RCL_RET_OK)
    {
      fprintf(stderr, "Failed to finish transition %u. Current state is now: %s\n",
        transition_id, state_machine_.current_state->label);
      return false;
    }

    // error handling ?!
    // TODO(karsten1987): iterate over possible ret value
    if (cb_success == RCL_LIFECYCLE_RET_ERROR) {
      rcl_lifecycle_ret_t error_resolved = execute_callback(state_machine_.current_state->id,
          initial_state);
      if (error_resolved == RCL_RET_OK) {
        fprintf(stderr, "Exception handling was successful\n");
        // We call cleanup on the error state
        rcl_lifecycle_trigger_transition(
          &state_machine_, error_resolved, true);
        fprintf(stderr, "current state after error callback%s\n",
          state_machine_.current_state->label);
      } else {
        // We call shutdown on the error state
        rcl_lifecycle_trigger_transition(
          &state_machine_, error_resolved, true);
      }
    }
    // This true holds in both cases where the actual callback
    // was successful or not, since at this point we have a valid transistion
    // to either a new primary state or error state
    return true;
  }

  rcl_lifecycle_ret_t
  execute_callback(unsigned int cb_id, const State & previous_state)
  {
    // in case no callback was attached, we forward directly
    auto cb_success = RCL_LIFECYCLE_RET_OK;

    auto it = cb_map_.find(cb_id);
    if (it != cb_map_.end()) {
      auto callback = it->second;
      try {
        cb_success = callback(State(previous_state));
      } catch (const std::exception &) {
        // TODO(karsten1987): Windows CI doens't let me print the msg here
        // the todo is to forward the exception to the on_error callback
        // fprintf(stderr, "Caught exception in callback for transition %d\n",
        //  it->first);
        // fprintf(stderr, "Original error msg: %s\n", e.what());
        // maybe directly go for error handling here
        // and pass exception along with it
        cb_success = RCL_LIFECYCLE_RET_ERROR;
      }
    }
    return cb_success;
  }

  const State &
  trigger_transition(uint8_t transition_id)
  {
    change_state(transition_id);
    return get_current_state();
  }

  void
  add_publisher_handle(std::shared_ptr<rclcpp_lifecycle::LifecyclePublisherInterface> pub)
  {
    weak_pubs_.push_back(pub);
  }

  void
  add_timer_handle(std::shared_ptr<rclcpp::timer::TimerBase> timer)
  {
    weak_timers_.push_back(timer);
  }

  rcl_lifecycle_state_machine_t state_machine_;
  State current_state_;
  std::map<
    std::uint8_t,
    std::function<rcl_lifecycle_ret_t(const State &)>> cb_map_;

  using NodeBasePtr = std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>;
  using NodeServicesPtr = std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>;
  using ChangeStateSrvPtr = std::shared_ptr<rclcpp::service::Service<ChangeStateSrv>>;
  using GetStateSrvPtr = std::shared_ptr<rclcpp::service::Service<GetStateSrv>>;
  using GetAvailableStatesSrvPtr =
      std::shared_ptr<rclcpp::service::Service<GetAvailableStatesSrv>>;
  using GetAvailableTransitionsSrvPtr =
      std::shared_ptr<rclcpp::service::Service<GetAvailableTransitionsSrv>>;

  NodeBasePtr node_base_interface_;
  NodeServicesPtr node_services_interface_;
  ChangeStateSrvPtr srv_change_state_;
  GetStateSrvPtr srv_get_state_;
  GetAvailableStatesSrvPtr srv_get_available_states_;
  GetAvailableTransitionsSrvPtr srv_get_available_transitions_;

  // to controllable things
  std::vector<std::weak_ptr<rclcpp_lifecycle::LifecyclePublisherInterface>> weak_pubs_;
  std::vector<std::weak_ptr<rclcpp::timer::TimerBase>> weak_timers_;
};

}  // namespace rclcpp_lifecycle
#endif  // LIFECYCLE_NODE_INTERFACE_IMPL_HPP_
