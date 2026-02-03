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

#ifndef RCLCPP_LIFECYCLE__STATE_HISTORY_HPP_
#define RCLCPP_LIFECYCLE__STATE_HISTORY_HPP_

#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_lifecycle
{

/**
 * @brief Entry in the state history
 *
 * Each entry records a state that was entered, along with metadata
 * about when and how the transition occurred.
 */
struct StateHistoryEntry
{
  /// The state that was entered
  State state;

  /// When the state was entered (steady_clock for reliable duration calculations)
  std::chrono::steady_clock::time_point timestamp;

  /// What triggered the transition (transition label or "initial" for the first state)
  std::string trigger;
};

/**
 * @class StateHistory
 * @brief Tracks the history of lifecycle state transitions
 *
 * The StateHistory class provides a thread-safe mechanism for recording
 * and querying the history of state transitions in a lifecycle node.
 * This is useful for debugging, monitoring, and implementing sophisticated
 * error recovery strategies.
 *
 * The history has a configurable maximum size to prevent unbounded memory
 * growth. When the limit is reached, the oldest entries are removed.
 *
 * Example usage:
 * @code{.cpp}
 * StateHistory history(100);  // Keep last 100 entries
 *
 * // Record a state transition
 * history.record_state(state, "configure");
 *
 * // Get the previous state
 * auto prev = history.get_previous_state();
 * if (prev.has_value()) {
 *   RCLCPP_INFO(logger, "Previous state: %s", prev->label().c_str());
 * }
 *
 * // Get all history entries
 * auto entries = history.get_entries(10);
 * for (const auto& entry : entries) {
 *   RCLCPP_INFO(logger, "State %s at %ld",
 *     entry.state.label().c_str(),
 *     entry.timestamp.time_since_epoch().count());
 * }
 * @endcode
 */
class StateHistory
{
public:
  /**
   * @brief Construct a StateHistory with a maximum size
   * @param max_size Maximum number of entries to retain (0 for unlimited)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit StateHistory(size_t max_size = 100);

  /**
   * @brief Destructor
   */
  RCLCPP_LIFECYCLE_PUBLIC
  ~StateHistory() = default;

  // Non-copyable but moveable
  StateHistory(const StateHistory &) = delete;
  StateHistory & operator=(const StateHistory &) = delete;
  StateHistory(StateHistory &&) = default;
  StateHistory & operator=(StateHistory &&) = default;

  /**
   * @brief Record a state transition
   * @param state The state that was entered
   * @param trigger What triggered the transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void record_state(const State & state, const std::string & trigger);

  /**
   * @brief Get the current (most recent) state
   * @return The current state, or std::nullopt if no states recorded
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::optional<State> get_current_state() const;

  /**
   * @brief Get the previous state (before the current one)
   * @return The previous state, or std::nullopt if not available
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::optional<State> get_previous_state() const;

  /**
   * @brief Get history entries, most recent first
   * @param max_entries Maximum number of entries to return (0 for all)
   * @return Vector of StateHistoryEntry objects
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<StateHistoryEntry> get_entries(size_t max_entries = 0) const;

  /**
   * @brief Get the time spent in the current state
   * @return Duration since entering the current state, or zero if no state
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::chrono::steady_clock::duration get_time_in_current_state() const;

  /**
   * @brief Get the number of entries in the history
   * @return Current history size
   */
  RCLCPP_LIFECYCLE_PUBLIC
  size_t size() const;

  /**
   * @brief Check if the history is empty
   * @return true if no entries recorded
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool empty() const;

  /**
   * @brief Clear all history entries
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void clear();

  /**
   * @brief Set the maximum history size
   * @param max_size New maximum size (0 for unlimited)
   *
   * If the new size is smaller than the current number of entries,
   * the oldest entries will be removed.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void set_max_size(size_t max_size);

  /**
   * @brief Get the maximum history size
   * @return Maximum number of entries (0 means unlimited)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  size_t get_max_size() const;

private:
  mutable std::mutex mutex_;
  std::deque<StateHistoryEntry> entries_;
  size_t max_size_;
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__STATE_HISTORY_HPP_
