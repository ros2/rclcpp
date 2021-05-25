// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/detail/mutex_two_priorities.hpp"

#include <mutex>

namespace rclcpp
{
namespace detail
{

using LowPriorityLockable = MutexTwoPriorities::LowPriorityLockable;
using HighPriorityLockable = MutexTwoPriorities::HighPriorityLockable;

HighPriorityLockable::HighPriorityLockable(MutexTwoPriorities & parent)
: parent_(parent)
{}

void
HighPriorityLockable::lock()
{
  std::unique_lock<std::mutex> guard{parent_.cv_mutex_};
  if (parent_.data_taken_) {
    ++parent_.hp_waiting_count_;
    while (parent_.data_taken_) {
      parent_.hp_cv_.wait(guard);
    }
    --parent_.hp_waiting_count_;
  }
  parent_.data_taken_ = true;
}

void
HighPriorityLockable::unlock()
{
  bool notify_lp{false};
  {
    std::lock_guard<std::mutex> guard{parent_.cv_mutex_};
    parent_.data_taken_ = false;
    notify_lp = 0u == parent_.hp_waiting_count_;
  }
  if (notify_lp) {
    parent_.lp_cv_.notify_one();
  } else {
    parent_.hp_cv_.notify_one();
  }
}

LowPriorityLockable::LowPriorityLockable(MutexTwoPriorities & parent)
: parent_(parent)
{}

void
LowPriorityLockable::lock()
{
  std::unique_lock<std::mutex> guard{parent_.cv_mutex_};
  while (parent_.data_taken_ || parent_.hp_waiting_count_) {
    parent_.lp_cv_.wait(guard);
  }
  parent_.data_taken_ = true;
}

void
LowPriorityLockable::unlock()
{
  bool notify_lp{false};
  {
    std::lock_guard<std::mutex> guard{parent_.cv_mutex_};
    parent_.data_taken_ = false;
    notify_lp = 0u == parent_.hp_waiting_count_;
  }
  if (notify_lp) {
    parent_.lp_cv_.notify_one();
  } else {
    parent_.hp_cv_.notify_one();
  }
}

HighPriorityLockable
MutexTwoPriorities::get_high_priority_lockable()
{
  return HighPriorityLockable{*this};
}

LowPriorityLockable
MutexTwoPriorities::get_low_priority_lockable()
{
  return LowPriorityLockable{*this};
}

}  // namespace detail
}  // namespace rclcpp
