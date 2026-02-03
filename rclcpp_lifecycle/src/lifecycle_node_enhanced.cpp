// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "rclcpp_lifecycle/lifecycle_node_enhanced.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

namespace rclcpp_lifecycle
{

LifecycleNodeEnhanced::LifecycleNodeEnhanced(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, namespace_, options),
  state_history_(std::make_unique<StateHistory>(100))
{
  // Record initial state
  record_transition(get_current_state(), "initial");
}

LifecycleNodeEnhanced::LifecycleNodeEnhanced(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: LifecycleNodeEnhanced(node_name, "", options)
{
}

LifecycleNodeEnhanced::~LifecycleNodeEnhanced() = default;

// ============================================================================
// State Query Methods
// ============================================================================

bool LifecycleNodeEnhanced::is_active() const noexcept
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool LifecycleNodeEnhanced::is_inactive() const noexcept
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool LifecycleNodeEnhanced::is_configured() const noexcept
{
  auto state_id = get_current_state().id();
  return state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
         state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool LifecycleNodeEnhanced::is_unconfigured() const noexcept
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
}

bool LifecycleNodeEnhanced::is_finalized() const noexcept
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
}

// ============================================================================
// State History Methods
// ============================================================================

std::optional<State> LifecycleNodeEnhanced::get_previous_state() const noexcept
{
  return state_history_->get_previous_state();
}

std::vector<StateHistoryEntry> LifecycleNodeEnhanced::get_state_history(size_t max_entries) const
{
  return state_history_->get_entries(max_entries);
}

std::chrono::steady_clock::duration LifecycleNodeEnhanced::get_time_in_current_state()
  const noexcept
{
  return state_history_->get_time_in_current_state();
}

// ============================================================================
// Safe Transition Methods
// ============================================================================

TransitionResult LifecycleNodeEnhanced::safe_transition(uint8_t transition_id)
{
  auto start_time = std::chrono::steady_clock::now();
  State from_state = get_current_state();

  try {
    // Attempt the transition
    State result_state = trigger_transition(transition_id);
    auto duration = std::chrono::steady_clock::now() - start_time;

    // Check if transition succeeded by comparing states
    // A successful transition moves to a new primary state
    bool success =
      result_state.id() != from_state.id() &&
      result_state.id() != lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING;

    if (success) {
      record_transition(result_state, "safe_transition");
      return TransitionResult::make_success(result_state, duration);
    }

    // Transition failed - attempt rollback is not needed as the node
    // should still be in its original state or error processing state
    std::string error_msg = "Transition failed from '" + from_state.label() +
      "' with transition ID " + std::to_string(transition_id);

    return TransitionResult::make_failure(
      error_msg,
      result_state,
      "Check node logs for callback failure details",
      false);

  } catch (const std::exception & e) {
    auto duration = std::chrono::steady_clock::now() - start_time;
    State current = get_current_state();

    return TransitionResult::make_failure(
      std::string("Exception during transition: ") + e.what(),
      current,
      "Review the exception and ensure callbacks don't throw",
      current.id() == from_state.id());
  }
}

TransitionResult LifecycleNodeEnhanced::safe_configure()
{
  if (!is_unconfigured()) {
    return TransitionResult::make_failure(
      "Cannot configure: node is not in unconfigured state",
      get_current_state(),
      "Call cleanup() first to return to unconfigured state");
  }
  return safe_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

TransitionResult LifecycleNodeEnhanced::safe_activate()
{
  if (!is_inactive()) {
    return TransitionResult::make_failure(
      "Cannot activate: node is not in inactive state",
      get_current_state(),
      "Call configure() first if unconfigured, or deactivate() first if active");
  }
  return safe_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

TransitionResult LifecycleNodeEnhanced::safe_deactivate()
{
  if (!is_active()) {
    return TransitionResult::make_failure(
      "Cannot deactivate: node is not in active state",
      get_current_state(),
      "Node must be active to deactivate");
  }
  return safe_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

TransitionResult LifecycleNodeEnhanced::safe_cleanup()
{
  if (!is_inactive()) {
    return TransitionResult::make_failure(
      "Cannot cleanup: node is not in inactive state",
      get_current_state(),
      "Call deactivate() first if active");
  }
  return safe_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

// ============================================================================
// Transition Validation Methods
// ============================================================================

bool LifecycleNodeEnhanced::can_transition_to(uint8_t target_state_id) const noexcept
{
  auto current_id = get_current_state().id();

  // Direct transitions allowed from current state
  switch (current_id) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      return target_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
             target_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      return target_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
             target_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED ||
             target_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      return target_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
             target_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      return false;  // No transitions from finalized state

    default:
      return false;
  }
}

// ============================================================================
// Transition Callback Methods
// ============================================================================

TransitionCallbackHandle LifecycleNodeEnhanced::register_on_transition_callback(
  std::function<void(const TransitionContext &)> callback)
{
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  TransitionCallbackHandle handle = next_callback_handle_++;
  transition_callbacks_[handle] = std::move(callback);
  return handle;
}

bool LifecycleNodeEnhanced::unregister_transition_callback(TransitionCallbackHandle handle)
{
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  auto it = transition_callbacks_.find(handle);
  if (it != transition_callbacks_.end()) {
    transition_callbacks_.erase(it);
    return true;
  }
  return false;
}

void LifecycleNodeEnhanced::notify_transition_callbacks(
  const State & from_state,
  const State & to_state,
  const Transition & transition,
  bool success)
{
  std::lock_guard<std::mutex> lock(callbacks_mutex_);

  TransitionContext context;
  context.from_state = from_state;
  context.to_state = to_state;
  context.transition = transition;
  context.timestamp = std::chrono::steady_clock::now();
  context.success = success;

  for (const auto & [handle, callback] : transition_callbacks_) {
    try {
      callback(context);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(),
        "Exception in transition callback (handle %lu): %s",
        handle,
        e.what());
    }
  }
}

// ============================================================================
// Private Methods
// ============================================================================

void LifecycleNodeEnhanced::record_transition(const State & state, const std::string & trigger)
{
  state_history_->record_state(state, trigger);
}

}  // namespace rclcpp_lifecycle
