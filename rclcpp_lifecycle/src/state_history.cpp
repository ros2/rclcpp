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

#include "rclcpp_lifecycle/state_history.hpp"

#include <algorithm>

namespace rclcpp_lifecycle
{

StateHistory::StateHistory(size_t max_size)
: max_size_(max_size)
{
}

void StateHistory::record_state(const State & state, const std::string & trigger)
{
  std::lock_guard<std::mutex> lock(mutex_);

  StateHistoryEntry entry;
  entry.state = state;
  entry.timestamp = std::chrono::steady_clock::now();
  entry.trigger = trigger;

  entries_.push_back(entry);

  // Enforce max size limit
  if (max_size_ > 0 && entries_.size() > max_size_) {
    entries_.pop_front();
  }
}

std::optional<State> StateHistory::get_current_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (entries_.empty()) {
    return std::nullopt;
  }

  return entries_.back().state;
}

std::optional<State> StateHistory::get_previous_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (entries_.size() < 2) {
    return std::nullopt;
  }

  return entries_[entries_.size() - 2].state;
}

std::vector<StateHistoryEntry> StateHistory::get_entries(size_t max_entries) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<StateHistoryEntry> result;

  // Return entries in reverse chronological order (most recent first)
  size_t count = max_entries == 0 ? entries_.size() : std::min(max_entries, entries_.size());
  result.reserve(count);

  auto it = entries_.rbegin();
  for (size_t i = 0; i < count && it != entries_.rend(); ++i, ++it) {
    result.push_back(*it);
  }

  return result;
}

std::chrono::steady_clock::duration StateHistory::get_time_in_current_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (entries_.empty()) {
    return std::chrono::steady_clock::duration::zero();
  }

  return std::chrono::steady_clock::now() - entries_.back().timestamp;
}

size_t StateHistory::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.size();
}

bool StateHistory::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.empty();
}

void StateHistory::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.clear();
}

void StateHistory::set_max_size(size_t max_size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  max_size_ = max_size;

  // Remove oldest entries if necessary
  if (max_size_ > 0) {
    while (entries_.size() > max_size_) {
      entries_.pop_front();
    }
  }
}

size_t StateHistory::get_max_size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return max_size_;
}

}  // namespace rclcpp_lifecycle
