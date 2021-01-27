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

using LowPriorityMutex = MutexTwoPriorities::LowPriorityMutex;
using HighPriorityMutex = MutexTwoPriorities::HighPriorityMutex;

HighPriorityMutex::HighPriorityMutex(MutexTwoPriorities & parent) : parent_(parent) {}

void
HighPriorityMutex::lock()
{
  parent_.data_.lock();
}

void
HighPriorityMutex::unlock()
{
  parent_.data_.unlock();
}

LowPriorityMutex::LowPriorityMutex(MutexTwoPriorities & parent) : parent_(parent) {}

void
LowPriorityMutex::lock()
{
  std::unique_lock<std::mutex> barrier_guard{parent_.barrier_};
  parent_.data_.lock();
  barrier_guard.release();
}

void
LowPriorityMutex::unlock()
{
  std::lock_guard<std::mutex> barrier_guard{parent_.barrier_, std::adopt_lock};
  parent_.data_.unlock();
}

HighPriorityMutex
MutexTwoPriorities::get_high_priority_mutex()
{
  return HighPriorityMutex{*this};
}

LowPriorityMutex
MutexTwoPriorities::get_low_priority_mutex()
{
  return LowPriorityMutex{*this};
}

}  // namespace detail
}  // namespace rclcpp
