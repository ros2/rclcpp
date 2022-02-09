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
#include "rcl_lifecycle/transition_map.h"

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

#include "rcutils/logging_macros.h"

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
    rcl_node_t * node_handle = node_base_interface_->get_rcl_node_handle();
    auto ret = rcl_lifecycle_state_machine_fini(&state_machine_, node_handle);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_FATAL_NAMED(
        "rclcpp_lifecycle",
        "failed to destroy rcl_state_machine");
    }
  }

  void
  init(bool enable_communication_interface = true)
  {
    rcl_node_t * node_handle = node_base_interface_->get_rcl_node_handle();
    const rcl_node_options_t * node_options =
      rcl_node_get_options(node_base_interface_->get_rcl_node_handle());
    state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();
    auto state_machine_options = rcl_lifecycle_get_default_state_machine_options();
    state_machine_options.enable_com_interface = enable_communication_interface;
    state_machine_options.allocator = node_options->allocator;

    // The call to initialize the state machine takes
    // currently five different typesupports for all publishers/services
    // created within the RCL_LIFECYCLE structure.
    // The publisher takes a C-Typesupport since the publishing (i.e. creating
    // the message) is done fully in RCL.
    // Services are handled in C++, so that it needs a C++ typesupport structure.
    rcl_ret_t ret = rcl_lifecycle_state_machine_init(
      &state_machine_,
      node_handle,
      ROSIDL_GET_MSG_TYPE_SUPPORT(lifecycle_msgs, msg, TransitionEvent),
      rosidl_typesupport_cpp::get_service_type_support_handle<ChangeStateSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetStateSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableStatesSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableTransitionsSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableTransitionsSrv>(),
      &state_machine_options);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error(
              std::string("Couldn't initialize state machine for node ") +
              node_base_interface_->get_name());
    }

    if (enable_communication_interface) {
      { // change_state
        auto cb = std::bind(
          &LifecycleNodeInterfaceImpl::on_change_state, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        rclcpp::AnyServiceCallback<ChangeStateSrv> any_cb;
        any_cb.set(std::move(cb));

        srv_change_state_ = std::make_shared<rclcpp::Service<ChangeStateSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_change_state,
          any_cb);
        node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_change_state_),
          nullptr);
      }

      { // get_state
        auto cb = std::bind(
          &LifecycleNodeInterfaceImpl::on_get_state, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        rclcpp::AnyServiceCallback<GetStateSrv> any_cb;
        any_cb.set(std::move(cb));

        srv_get_state_ = std::make_shared<rclcpp::Service<GetStateSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_get_state,
          any_cb);
        node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_state_),
          nullptr);
      }

      { // get_available_states
        auto cb = std::bind(
          &LifecycleNodeInterfaceImpl::on_get_available_states, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        rclcpp::AnyServiceCallback<GetAvailableStatesSrv> any_cb;
        any_cb.set(std::move(cb));

        srv_get_available_states_ = std::make_shared<rclcpp::Service<GetAvailableStatesSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_get_available_states,
          any_cb);
        node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_states_),
          nullptr);
      }

      { // get_available_transitions
        auto cb = std::bind(
          &LifecycleNodeInterfaceImpl::on_get_available_transitions, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
        any_cb.set(std::move(cb));

        srv_get_available_transitions_ =
          std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_get_available_transitions,
          any_cb);
        node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_transitions_),
          nullptr);
      }

      { // get_transition_graph
        auto cb = std::bind(
          &LifecycleNodeInterfaceImpl::on_get_transition_graph, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
        any_cb.set(std::move(cb));

        srv_get_transition_graph_ =
          std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_get_transition_graph,
          any_cb);
        node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_transition_graph_),
          nullptr);
      }
    }
  }

  bool
  register_callback(
    std::uint8_t lifecycle_transition,
    std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> & cb)
  {
    cb_map_[lifecycle_transition] = cb;
    return true;
  }

  void
  on_change_state(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<ChangeStateSrv::Request> req,
    std::shared_ptr<ChangeStateSrv::Response> resp)
  {
    (void)header;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error(
              "Can't get state. State machine is not initialized.");
    }

    auto transition_id = req->transition.id;
    // if there's a label attached to the request,
    // we check the transition attached to this label.
    // we further can't compare the id of the looked up transition
    // because ros2 service call defaults all intergers to zero.
    // that means if we call ros2 service call ... {transition: {label: shutdown}}
    // the id of the request is 0 (zero) whereas the id from the lookup up transition
    // can be different.
    // the result of this is that the label takes presedence of the id.
    if (req->transition.label.size() != 0) {
      auto rcl_transition = rcl_lifecycle_get_transition_by_label(
        state_machine_.current_state, req->transition.label.c_str());
      if (rcl_transition == nullptr) {
        resp->success = false;
        return;
      }
      transition_id = static_cast<std::uint8_t>(rcl_transition->id);
    }

    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code;
    auto ret = change_state(transition_id, cb_return_code);
    (void) ret;
    // TODO(karsten1987): Lifecycle msgs have to be extended to keep both returns
    // 1. return is the actual transition
    // 2. return is whether an error occurred or not
    resp->success =
      (cb_return_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  }

  void
  on_get_state(
    const std::shared_ptr<rmw_request_id_t> header,
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
  on_get_available_states(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableStatesSrv::Request> req,
    std::shared_ptr<GetAvailableStatesSrv::Response> resp)
  {
    (void)header;
    (void)req;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error(
              "Can't get available states. State machine is not initialized.");
    }

    resp->available_states.resize(state_machine_.transition_map.states_size);
    for (unsigned int i = 0; i < state_machine_.transition_map.states_size; ++i) {
      resp->available_states[i].id =
        static_cast<uint8_t>(state_machine_.transition_map.states[i].id);
      resp->available_states[i].label =
        static_cast<std::string>(state_machine_.transition_map.states[i].label);
    }
  }

  void
  on_get_available_transitions(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
    std::shared_ptr<GetAvailableTransitionsSrv::Response> resp)
  {
    (void)header;
    (void)req;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error(
              "Can't get available transitions. State machine is not initialized.");
    }

    resp->available_transitions.resize(state_machine_.current_state->valid_transition_size);
    for (unsigned int i = 0; i < state_machine_.current_state->valid_transition_size; ++i) {
      lifecycle_msgs::msg::TransitionDescription & trans_desc = resp->available_transitions[i];

      auto rcl_transition = state_machine_.current_state->valid_transitions[i];
      trans_desc.transition.id = static_cast<uint8_t>(rcl_transition.id);
      trans_desc.transition.label = rcl_transition.label;
      trans_desc.start_state.id = static_cast<uint8_t>(rcl_transition.start->id);
      trans_desc.start_state.label = rcl_transition.start->label;
      trans_desc.goal_state.id = static_cast<uint8_t>(rcl_transition.goal->id);
      trans_desc.goal_state.label = rcl_transition.goal->label;
    }
  }

  void
  on_get_transition_graph(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
    std::shared_ptr<GetAvailableTransitionsSrv::Response> resp)
  {
    (void)header;
    (void)req;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error(
              "Can't get available transitions. State machine is not initialized.");
    }

    resp->available_transitions.resize(state_machine_.transition_map.transitions_size);
    for (unsigned int i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
      lifecycle_msgs::msg::TransitionDescription & trans_desc = resp->available_transitions[i];

      auto rcl_transition = state_machine_.transition_map.transitions[i];
      trans_desc.transition.id = static_cast<uint8_t>(rcl_transition.id);
      trans_desc.transition.label = rcl_transition.label;
      trans_desc.start_state.id = static_cast<uint8_t>(rcl_transition.start->id);
      trans_desc.start_state.label = rcl_transition.start->label;
      trans_desc.goal_state.id = static_cast<uint8_t>(rcl_transition.goal->id);
      trans_desc.goal_state.label = rcl_transition.goal->label;
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
    states.reserve(state_machine_.transition_map.states_size);

    for (unsigned int i = 0; i < state_machine_.transition_map.states_size; ++i) {
      states.emplace_back(&state_machine_.transition_map.states[i]);
    }
    return states;
  }

  std::vector<Transition>
  get_available_transitions()
  {
    std::vector<Transition> transitions;
    transitions.reserve(state_machine_.current_state->valid_transition_size);

    for (unsigned int i = 0; i < state_machine_.current_state->valid_transition_size; ++i) {
      transitions.emplace_back(&state_machine_.current_state->valid_transitions[i]);
    }
    return transitions;
  }

  std::vector<Transition>
  get_transition_graph()
  {
    std::vector<Transition> transitions;
    transitions.reserve(state_machine_.transition_map.transitions_size);

    for (unsigned int i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
      transitions.emplace_back(&state_machine_.transition_map.transitions[i]);
    }
    return transitions;
  }

  rcl_ret_t
  change_state(std::uint8_t transition_id, LifecycleNodeInterface::CallbackReturn & cb_return_code)
  {
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR(
        "Unable to change state for state machine for %s: %s",
        node_base_interface_->get_name(), rcl_get_error_string().str);
      return RCL_RET_ERROR;
    }

    constexpr bool publish_update = true;
    // keep the initial state to pass to a transition callback
    State initial_state(state_machine_.current_state);

    if (
      rcl_lifecycle_trigger_transition_by_id(
        &state_machine_, transition_id, publish_update) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR(
        "Unable to start transition %u from current state %s: %s",
        transition_id, state_machine_.current_state->label, rcl_get_error_string().str);
      rcutils_reset_error();
      return RCL_RET_ERROR;
    }

    auto get_label_for_return_code =
      [](node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code) -> const char *{
        auto cb_id = static_cast<uint8_t>(cb_return_code);
        if (cb_id == lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS) {
          return rcl_lifecycle_transition_success_label;
        } else if (cb_id == lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE) {
          return rcl_lifecycle_transition_failure_label;
        }
        return rcl_lifecycle_transition_error_label;
      };

    cb_return_code = execute_callback(state_machine_.current_state->id, initial_state);
    auto transition_label = get_label_for_return_code(cb_return_code);

    if (
      rcl_lifecycle_trigger_transition_by_label(
        &state_machine_, transition_label, publish_update) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR(
        "Failed to finish transition %u. Current state is now: %s (%s)",
        transition_id, state_machine_.current_state->label, rcl_get_error_string().str);
      rcutils_reset_error();
      return RCL_RET_ERROR;
    }

    // error handling ?!
    // TODO(karsten1987): iterate over possible ret value
    if (cb_return_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR) {
      RCUTILS_LOG_WARN("Error occurred while doing error handling.");

      auto error_cb_code = execute_callback(state_machine_.current_state->id, initial_state);
      auto error_cb_label = get_label_for_return_code(error_cb_code);
      if (
        rcl_lifecycle_trigger_transition_by_label(
          &state_machine_, error_cb_label, publish_update) != RCL_RET_OK)
      {
        RCUTILS_LOG_ERROR("Failed to call cleanup on error state: %s", rcl_get_error_string().str);
        rcutils_reset_error();
        return RCL_RET_ERROR;
      }
    }
    // This true holds in both cases where the actual callback
    // was successful or not, since at this point we have a valid transistion
    // to either a new primary state or error state
    return RCL_RET_OK;
  }

  LifecycleNodeInterface::CallbackReturn
  execute_callback(unsigned int cb_id, const State & previous_state)
  {
    // in case no callback was attached, we forward directly
    auto cb_success = node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    auto it = cb_map_.find(static_cast<uint8_t>(cb_id));
    if (it != cb_map_.end()) {
      auto callback = it->second;
      try {
        cb_success = callback(State(previous_state));
      } catch (const std::exception & e) {
        RCUTILS_LOG_ERROR("Caught exception in callback for transition %d", it->first);
        RCUTILS_LOG_ERROR("Original error: %s", e.what());
        cb_success = node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
      }
    }
    return cb_success;
  }

  const State & trigger_transition(const char * transition_label)
  {
    LifecycleNodeInterface::CallbackReturn error;
    return trigger_transition(transition_label, error);
  }

  const State & trigger_transition(
    const char * transition_label, LifecycleNodeInterface::CallbackReturn & cb_return_code)
  {
    auto transition =
      rcl_lifecycle_get_transition_by_label(state_machine_.current_state, transition_label);
    if (transition) {
      change_state(static_cast<uint8_t>(transition->id), cb_return_code);
    }
    return get_current_state();
  }

  const State &
  trigger_transition(uint8_t transition_id)
  {
    LifecycleNodeInterface::CallbackReturn error;
    change_state(transition_id, error);
    (void) error;
    return get_current_state();
  }

  const State &
  trigger_transition(uint8_t transition_id, LifecycleNodeInterface::CallbackReturn & cb_return_code)
  {
    change_state(transition_id, cb_return_code);
    return get_current_state();
  }

  void
  add_managed_entity(std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity)
  {
    weak_managed_entities_.push_back(managed_entity);
  }

  void
  add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer)
  {
    weak_timers_.push_back(timer);
  }

  void
  on_activate()
  {
    for (const auto & weak_entity : weak_managed_entities_) {
      auto entity = weak_entity.lock();
      if (entity) {
        entity->on_activate();
      }
    }
  }

  void
  on_deactivate()
  {
    for (const auto & weak_entity : weak_managed_entities_) {
      auto entity = weak_entity.lock();
      if (entity) {
        entity->on_deactivate();
      }
    }
  }

  rcl_lifecycle_state_machine_t state_machine_;
  State current_state_;
  std::map<
    std::uint8_t,
    std::function<LifecycleNodeInterface::CallbackReturn(const State &)>> cb_map_;

  using NodeBasePtr = std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>;
  using NodeServicesPtr = std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>;
  using ChangeStateSrvPtr = std::shared_ptr<rclcpp::Service<ChangeStateSrv>>;
  using GetStateSrvPtr = std::shared_ptr<rclcpp::Service<GetStateSrv>>;
  using GetAvailableStatesSrvPtr =
    std::shared_ptr<rclcpp::Service<GetAvailableStatesSrv>>;
  using GetAvailableTransitionsSrvPtr =
    std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>;
  using GetTransitionGraphSrvPtr =
    std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>;

  NodeBasePtr node_base_interface_;
  NodeServicesPtr node_services_interface_;
  ChangeStateSrvPtr srv_change_state_;
  GetStateSrvPtr srv_get_state_;
  GetAvailableStatesSrvPtr srv_get_available_states_;
  GetAvailableTransitionsSrvPtr srv_get_available_transitions_;
  GetTransitionGraphSrvPtr srv_get_transition_graph_;

  // to controllable things
  std::vector<std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface>> weak_managed_entities_;
  std::vector<std::weak_ptr<rclcpp::TimerBase>> weak_timers_;
};

}  // namespace rclcpp_lifecycle
#endif  // LIFECYCLE_NODE_INTERFACE_IMPL_HPP_
