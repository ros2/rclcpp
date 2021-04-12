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
  bool did_aument_count{false};
  std::unique_lock<std::mutex> guard{parent_.cv_mutex_};
  if (parent_.data_taken_) {
    ++parent_.hp_waiting_count_;
    did_aument_count = true;
  }
  while (parent_.data_taken_) {
    parent_.cv_.wait(guard);
  }
  if (did_aument_count) {
    --parent_.hp_waiting_count_;
  }
  parent_.data_taken_ = true;
}

void
HighPriorityLockable::unlock()
{
  {
    std::lock_guard<std::mutex> guard{parent_.cv_mutex_};
    parent_.data_taken_ = false;
  }
  // We need to notify_all(), and not only one,
  // because the notified thread might be low priority
  // when a high priority thread is still waiting.
  // In that case, the low priority thread will go to sleep again,
  // release the mutex, and the high priority thread can continue.
  parent_.cv_.notify_all();
}

LowPriorityLockable::LowPriorityLockable(MutexTwoPriorities & parent)
: parent_(parent)
{}

void
LowPriorityLockable::lock()
{
  std::unique_lock<std::mutex> guard{parent_.cv_mutex_};
  while (parent_.data_taken_ || parent_.hp_waiting_count_) {
    parent_.cv_.wait(guard);
  }
  parent_.data_taken_ = true;
}

void
LowPriorityLockable::unlock()
{
  {
    std::lock_guard<std::mutex> guard{parent_.cv_mutex_};
    parent_.data_taken_ = false;
  }
  // See comment in HighPriorityLockable::unlock() to understand
  // why not using notify_one().
  parent_.cv_.notify_all();
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
