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

#include <cassert>
#include <memory>
#include <chrono>

#include "lifecycle_node_state_services_manager.hpp"
#include "lifecycle_node_state_manager.hpp"

namespace rclcpp_lifecycle
{
void
LifecycleNodeStateManager::init(
  const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
  bool enable_communication_interface)
{
  using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
  using GetStateSrv = lifecycle_msgs::srv::GetState;
  using GetAvailableStatesSrv = lifecycle_msgs::srv::GetAvailableStates;
  using GetAvailableTransitionsSrv = lifecycle_msgs::srv::GetAvailableTransitions;

  node_base_interface_ = node_base_interface;

  rcl_node_t * node_handle = node_base_interface_->get_rcl_node_handle();
  const rcl_node_options_t * node_options =
    rcl_node_get_options(node_base_interface_->get_rcl_node_handle());
  auto state_machine_options = rcl_lifecycle_get_default_state_machine_options();
  state_machine_options.enable_com_interface = enable_communication_interface;
  state_machine_options.allocator = node_options->allocator;

  // The call to initialize the state machine takes
  // currently five different typesupports for all publishers/services
  // created within the RCL_LIFECYCLE structure.
  // The publisher takes a C-Typesupport since the publishing (i.e. creating
  // the message) is done fully in RCL.
  // Services are handled in C++, so that it needs a C++ typesupport structure.
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();
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

  update_current_state_();
}

void
LifecycleNodeStateManager::throw_runtime_error_on_uninitialized_state_machine(
  const std::string & attempted_action) const
{
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error(
            "Can't " + attempted_action + ". State machine is not initialized.");
  }
}

bool
LifecycleNodeStateManager::register_callback(
  std::uint8_t lifecycle_transition,
  std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> & cb)
{
  cb_map_[lifecycle_transition] = cb;
  return true;
}

const State &
LifecycleNodeStateManager::get_current_state() const
{
  return current_state_;
}

std::vector<State>
LifecycleNodeStateManager::get_available_states() const
{
  std::vector<State> states;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  states.reserve(state_machine_.transition_map.states_size);

  for (unsigned int i = 0; i < state_machine_.transition_map.states_size; ++i) {
    states.emplace_back(&state_machine_.transition_map.states[i]);
  }
  return states;
}

std::vector<Transition>
LifecycleNodeStateManager::get_available_transitions() const
{
  std::vector<Transition> transitions;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  transitions.reserve(state_machine_.current_state->valid_transition_size);

  for (unsigned int i = 0; i < state_machine_.current_state->valid_transition_size; ++i) {
    transitions.emplace_back(&state_machine_.current_state->valid_transitions[i]);
  }
  return transitions;
}

std::vector<Transition>
LifecycleNodeStateManager::get_transition_graph() const
{
  std::vector<Transition> transitions;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  transitions.reserve(state_machine_.transition_map.transitions_size);

  for (unsigned int i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
    transitions.emplace_back(&state_machine_.transition_map.transitions[i]);
  }
  return transitions;
}

bool
LifecycleNodeStateManager::is_transitioning() const
{
  return is_transitioning_.load();
}

rcl_ret_t
LifecycleNodeStateManager::change_state(
  uint8_t transition_id,
  node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code
)
{
  rcl_ret_t ret = change_state(transition_id);
  cb_return_code = cb_return_code_;
  return ret;
}

rcl_ret_t
LifecycleNodeStateManager::change_state(
  uint8_t transition_id,
  std::function<void(bool, std::shared_ptr<rmw_request_id_t>)> callback /*= nullptr*/,
  const std::shared_ptr<rmw_request_id_t> header /* = nullptr*/)
{
  if (is_transitioning()) {
    RCUTILS_LOG_ERROR(
      "%s currently in transition, failing requested transition id %d.",
      node_base_interface_->get_name(),
      transition_id);
    if (callback) {
      callback(false, header);
    }
    return RCL_RET_ERROR;
  }

  is_transitioning_.store(true);
  send_change_state_resp_cb_ = callback;
  change_state_header_ = header;
  transition_id_ = transition_id;
  rcl_ret_ = RCL_RET_OK;
  transition_cb_completed_ = false;
  on_error_cb_completed_ = false;

  unsigned int current_state_id;
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    constexpr bool publish_update = true;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR(
        "Unable to change state for state machine for %s: %s",
        node_base_interface_->get_name(), rcl_get_error_string().str);
      rcl_ret_error();
      return rcl_ret_;
    }

    pre_transition_primary_state_ = State(state_machine_.current_state);

    if (
      rcl_lifecycle_trigger_transition_by_id(
        &state_machine_, transition_id_, publish_update) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR(
        "Unable to start transition %u from current state %s: %s",
        transition_id_, state_machine_.current_state->label,
        rcl_get_error_string().str);
      rcutils_reset_error();
      rcl_ret_error();
      return rcl_ret_;
    }
    current_state_id = get_current_state_id();
    update_current_state_();
  }

  cb_return_code_ = execute_callback(
    current_state_id,
    pre_transition_primary_state_);
  process_callback_resp(cb_return_code_);

  return rcl_ret_;
}

void
LifecycleNodeStateManager::process_callback_resp(
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
{
  uint8_t current_state_id = get_current_state_id();
  if (in_non_error_transition_state(current_state_id)) {
    if (transition_cb_completed_) {
      RCUTILS_LOG_ERROR(
        "process_callback_resp recursively running user"
        " transition function with id%d",
        current_state_id);
      rcl_ret_error();
      return;
    }
    transition_cb_completed_ = true;
    process_user_callback_resp(cb_return_code);
  } else if (in_error_transition_state(current_state_id)) {
    if (on_error_cb_completed_) {
      RCUTILS_LOG_ERROR(
        "process_callback_resp recursively running user"
        " transition function with id%d",
        current_state_id);
      rcl_ret_error();
      return;
    }
    on_error_cb_completed_ = true;
    process_on_error_resp(cb_return_code);
  } else {
    RCUTILS_LOG_ERROR(
      "process_callback_resp failed for %s: not in a transition state",
      node_base_interface_->get_name());
    rcl_ret_error();
  }
}

int
LifecycleNodeStateManager::get_transition_id_from_request(
  const ChangeStateSrv::Request::SharedPtr req)
{
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

  // Use transition.label over transition.id if transition.label exits
  // label has higher precedence to the id due to ROS 2 defaulting integers to 0
  // e.g.: srv call of {transition: {label: configure}}
  //       transition.id    = 0           -> would be equiv to "create"
  //       transition.label = "configure" -> id is 1, use this
  if (req->transition.label.size() != 0) {
    auto rcl_transition = rcl_lifecycle_get_transition_by_label(
      state_machine_.current_state, req->transition.label.c_str());
    if (rcl_transition == nullptr) {
      // send fail response printing out the requested label
      RCUTILS_LOG_ERROR(
        "ChangeState srv request failed: Transition label '%s'"
        " is not available in the current state '%s'",
        req->transition.label.c_str(), state_machine_.current_state->label);
      return -1;
    }
    return static_cast<int>(rcl_transition->id);
  }
  return req->transition.id;
}

const rcl_lifecycle_transition_t *
LifecycleNodeStateManager::get_transition_by_label(const char * label) const
{
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  return
    rcl_lifecycle_get_transition_by_label(state_machine_.current_state, label);
}

rcl_lifecycle_com_interface_t &
LifecycleNodeStateManager::get_rcl_com_interface()
{
  return state_machine_.com_interface;
}

void
LifecycleNodeStateManager::process_user_callback_resp(
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
{
  constexpr bool publish_update = true;
  unsigned int current_state_id;

  cb_return_code_ = cb_return_code;
  auto transition_label = get_label_for_return_code(cb_return_code);
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    if (
      rcl_lifecycle_trigger_transition_by_label(
        &state_machine_, transition_label, publish_update) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR(
        "Failed to finish transition %u: Current state is now: %s (%s)",
        transition_id_,
        state_machine_.current_state->label,
        rcl_get_error_string().str);
      rcutils_reset_error();
      rcl_ret_error();
      return;
    }
    current_state_id = get_current_state_id();

    update_current_state_();
  }

  // error handling ?!
  // TODO(karsten1987): iterate over possible ret value
  if (cb_return_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR) {
    RCUTILS_LOG_WARN("Error occurred while calling transition function, calling on_error.");
    auto error_cb_code = execute_callback(
      current_state_id,
      pre_transition_primary_state_);
    process_callback_resp(error_cb_code);
  } else {
    finalize_change_state(
      cb_return_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  }
}

void
LifecycleNodeStateManager::process_on_error_resp(
  node_interfaces::LifecycleNodeInterface::CallbackReturn error_cb_code)
{
  constexpr bool publish_update = true;
  auto error_cb_label = get_label_for_return_code(error_cb_code);
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  if (
    rcl_lifecycle_trigger_transition_by_label(
      &state_machine_, error_cb_label, publish_update) != RCL_RET_OK)
  {
    RCUTILS_LOG_ERROR("Failed to call cleanup on error state: %s", rcl_get_error_string().str);
    rcutils_reset_error();
    rcl_ret_error();
    return;
  }
  finalize_change_state(
    error_cb_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

void
LifecycleNodeStateManager::finalize_change_state(bool success)
{
  // TODO(karsten1987): Lifecycle msgs have to be extended to keep both returns
  // 1. return is the actual transition
  // 2. return is whether an error occurred or not
  rcl_ret_ = success ? RCL_RET_OK : RCL_RET_ERROR;
  update_current_state_();

  if (send_change_state_resp_cb_) {
    send_change_state_resp_cb_(success, change_state_header_);
    change_state_header_.reset();
    send_change_state_resp_cb_ = nullptr;
  }
  is_transitioning_.store(false);
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleNodeStateManager::execute_callback(
  unsigned int cb_id, const State & previous_state) const
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

const char *
LifecycleNodeStateManager::get_label_for_return_code(
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
{
  auto cb_id = static_cast<uint8_t>(cb_return_code);
  if (cb_id == lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS) {
    return rcl_lifecycle_transition_success_label;
  } else if (cb_id == lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE) {
    return rcl_lifecycle_transition_failure_label;
  }
  return rcl_lifecycle_transition_error_label;
}

void
LifecycleNodeStateManager::rcl_ret_error()
{
  finalize_change_state(false);
}

void
LifecycleNodeStateManager::update_current_state_()
{
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  current_state_ = State(state_machine_.current_state);
}

uint8_t
LifecycleNodeStateManager::get_current_state_id() const
{
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  return state_machine_.current_state->id;
}

bool
LifecycleNodeStateManager::in_non_error_transition_state(
  uint8_t current_state_id) const
{
  return current_state_id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING ||
         current_state_id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP ||
         current_state_id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN ||
         current_state_id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING ||
         current_state_id == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING;
}

bool
LifecycleNodeStateManager::in_error_transition_state(
  uint8_t current_state_id) const
{
  return current_state_id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING;
}

LifecycleNodeStateManager::~LifecycleNodeStateManager()
{
  rcl_node_t * node_handle = node_base_interface_->get_rcl_node_handle();
  rcl_ret_t ret;
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    ret = rcl_lifecycle_state_machine_fini(&state_machine_, node_handle);
  }
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_FATAL_NAMED(
      "rclcpp_lifecycle",
      "failed to destroy rcl_state_machine");
  }
  send_change_state_resp_cb_ = nullptr;
  change_state_header_.reset();
  node_base_interface_.reset();
}

}  // namespace rclcpp_lifecycle
