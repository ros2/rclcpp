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
  parent_.data_.lock();
}

void
HighPriorityLockable::unlock()
{
  parent_.data_.unlock();
}

LowPriorityLockable::LowPriorityLockable(MutexTwoPriorities & parent)
: parent_(parent)
{}

void
LowPriorityLockable::lock()
{
  std::unique_lock<std::mutex> barrier_guard{parent_.barrier_};
  parent_.data_.lock();
  barrier_guard.release();
}

void
LowPriorityLockable::unlock()
{
  std::lock_guard<std::mutex> barrier_guard{parent_.barrier_, std::adopt_lock};
  parent_.data_.unlock();
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
