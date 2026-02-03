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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_ENHANCED_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_NODE_ENHANCED_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state_history.hpp"
#include "rclcpp_lifecycle/lifecycle_exceptions.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_lifecycle
{

/**
 * @brief Result structure for transition operations
 *
 * Contains comprehensive information about the outcome of a transition attempt,
 * including success status, error details, and rollback information.
 */
struct TransitionResult
{
  /// Whether the transition succeeded
  bool success = false;

  /// Human-readable error message (empty if success)
  std::string error_message;

  /// Suggested recovery action (empty if success)
  std::string recovery_suggestion;

  /// Whether automatic rollback succeeded (for safe_* methods)
  bool rollback_succeeded = false;

  /// The state after the transition (or attempted transition)
  State resulting_state;

  /// Time taken for the transition
  std::chrono::steady_clock::duration duration = std::chrono::steady_clock::duration::zero();

  /**
   * @brief Construct a successful TransitionResult
   */
  static TransitionResult make_success(
    const State & resulting_state,
    std::chrono::steady_clock::duration duration = std::chrono::steady_clock::duration::zero())
  {
    TransitionResult result;
    result.success = true;
    result.resulting_state = resulting_state;
    result.duration = duration;
    return result;
  }

  /**
   * @brief Construct a failed TransitionResult
   */
  static TransitionResult make_failure(
    const std::string & error_message,
    const State & resulting_state,
    const std::string & recovery_suggestion = "",
    bool rollback_succeeded = false)
  {
    TransitionResult result;
    result.success = false;
    result.error_message = error_message;
    result.resulting_state = resulting_state;
    result.recovery_suggestion = recovery_suggestion;
    result.rollback_succeeded = rollback_succeeded;
    return result;
  }
};

/**
 * @brief Context information for transition callbacks
 *
 * Provides detailed information about a transition for observers.
 */
struct TransitionContext
{
  /// The state being transitioned from
  State from_state;

  /// The state being transitioned to
  State to_state;

  /// The transition being performed
  Transition transition;

  /// When the transition occurred
  std::chrono::steady_clock::time_point timestamp;

  /// Whether the transition succeeded
  bool success = false;
};

/**
 * @brief Handle for registered transition callbacks
 */
using TransitionCallbackHandle = uint64_t;

/**
 * @class LifecycleNodeEnhanced
 * @brief Enhanced lifecycle node with improved state management and error handling
 *
 * This class extends LifecycleNode to provide additional features commonly
 * requested by the ROS2 community:
 *
 * - **State Query Methods**: Convenient methods like is_active(), is_configured()
 * - **State History**: Track and query the history of state transitions
 * - **Safe Transitions**: Automatic rollback on transition failure
 * - **Transition Callbacks**: Register observers for any state transition
 * - **Improved Error Handling**: Detailed error information and recovery suggestions
 *
 * The class is designed to be fully backward compatible with existing
 * LifecycleNode code.
 *
 * Example usage:
 * @code{.cpp}
 * class MyNode : public rclcpp_lifecycle::LifecycleNodeEnhanced
 * {
 * public:
 *   MyNode() : LifecycleNodeEnhanced("my_node") {}
 *
 *   void do_work()
 *   {
 *     if (is_active()) {
 *       publisher_->publish(msg);
 *     }
 *   }
 *
 *   CallbackReturn on_configure(const State & previous_state) override
 *   {
 *     // Use state history for debugging
 *     auto prev = get_previous_state();
 *     if (prev.has_value()) {
 *       RCLCPP_INFO(get_logger(), "Previous state: %s", prev->label().c_str());
 *     }
 *     return CallbackReturn::SUCCESS;
 *   }
 * };
 * @endcode
 */
class LifecycleNodeEnhanced : public LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleNodeEnhanced)

  /**
   * @brief Construct a LifecycleNodeEnhanced
   *
   * @param node_name Name of the node
   * @param namespace_ Namespace of the node
   * @param options Node options
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit LifecycleNodeEnhanced(
    const std::string & node_name,
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Construct a LifecycleNodeEnhanced with explicit namespace
   *
   * @param node_name Name of the node
   * @param options Node options
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit LifecycleNodeEnhanced(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);

  /**
   * @brief Virtual destructor
   */
  RCLCPP_LIFECYCLE_PUBLIC
  ~LifecycleNodeEnhanced() override;

  // ============================================================================
  // State Query Methods
  // ============================================================================

  /**
   * @brief Check if the node is currently in the Active state
   * @return true if the node is in the Active state, false otherwise
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  bool is_active() const noexcept;

  /**
   * @brief Check if the node is currently in the Inactive state
   * @return true if the node is in the Inactive state, false otherwise
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  bool is_inactive() const noexcept;

  /**
   * @brief Check if the node has been configured (inactive or active state)
   * @return true if the node is in Inactive or Active state, false otherwise
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  bool is_configured() const noexcept;

  /**
   * @brief Check if the node is in the Unconfigured state
   * @return true if the node is in the Unconfigured state, false otherwise
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  bool is_unconfigured() const noexcept;

  /**
   * @brief Check if the node is in the Finalized state
   * @return true if the node is in the Finalized state, false otherwise
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  bool is_finalized() const noexcept;

  // ============================================================================
  // State History Methods
  // ============================================================================

  /**
   * @brief Get the previous state before the current state
   * @return The previous State object, or std::nullopt if no previous state exists
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  std::optional<State> get_previous_state() const noexcept;

  /**
   * @brief Get the complete history of state transitions
   * @param max_entries Maximum number of entries to return (0 = unlimited)
   * @return Vector of StateHistoryEntry objects, most recent first
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  std::vector<StateHistoryEntry> get_state_history(size_t max_entries = 0) const;

  /**
   * @brief Get the duration the node has been in the current state
   * @return Duration since entering the current state
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  std::chrono::steady_clock::duration get_time_in_current_state() const noexcept;

  // ============================================================================
  // Safe Transition Methods
  // ============================================================================

  /**
   * @brief Perform a transition with automatic rollback on failure
   * @param transition_id The transition to perform
   * @return TransitionResult containing success status and error details
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  TransitionResult safe_transition(uint8_t transition_id);

  /**
   * @brief Safely transition from Unconfigured to Inactive state
   * @return TransitionResult containing success status and error details
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  TransitionResult safe_configure();

  /**
   * @brief Safely transition from Inactive to Active state
   * @return TransitionResult containing success status and error details
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  TransitionResult safe_activate();

  /**
   * @brief Safely transition from Active to Inactive state
   * @return TransitionResult containing success status and error details
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  TransitionResult safe_deactivate();

  /**
   * @brief Safely transition from Inactive to Unconfigured state
   * @return TransitionResult containing success status and error details
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  TransitionResult safe_cleanup();

  // ============================================================================
  // Transition Validation Methods
  // ============================================================================

  /**
   * @brief Check if a transition to the specified state is valid
   * @param target_state_id The target state ID to check
   * @return true if a valid transition path exists to the target state
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  bool can_transition_to(uint8_t target_state_id) const noexcept;

  // ============================================================================
  // Transition Callback Methods
  // ============================================================================

  /**
   * @brief Register a callback to be invoked on any state transition
   * @param callback Function to call with TransitionContext
   * @return Handle that can be used to unregister the callback
   */
  [[nodiscard]] RCLCPP_LIFECYCLE_PUBLIC
  TransitionCallbackHandle register_on_transition_callback(
    std::function<void(const TransitionContext &)> callback);

  /**
   * @brief Unregister a previously registered transition callback
   * @param handle The handle returned from register_on_transition_callback
   * @return true if the callback was successfully unregistered
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool unregister_transition_callback(TransitionCallbackHandle handle);

protected:
  /**
   * @brief Called internally when a transition occurs
   * @param from_state State before transition
   * @param to_state State after transition
   * @param transition The transition that occurred
   * @param success Whether the transition succeeded
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void notify_transition_callbacks(
    const State & from_state,
    const State & to_state,
    const Transition & transition,
    bool success);

private:
  /// State history tracking
  std::unique_ptr<StateHistory> state_history_;

  /// Registered transition callbacks
  std::map<TransitionCallbackHandle, std::function<void(const TransitionContext &)>>
    transition_callbacks_;

  /// Mutex for thread-safe callback management
  mutable std::mutex callbacks_mutex_;

  /// Counter for generating unique callback handles
  TransitionCallbackHandle next_callback_handle_ = 1;

  /// Record a state transition in history
  void record_transition(const State & state, const std::string & trigger);
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_ENHANCED_HPP_
