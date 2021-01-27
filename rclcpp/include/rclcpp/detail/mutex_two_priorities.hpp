// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__MUTEX_TWO_PRIORITIES_HPP_
#define RCLCPP__DETAIL__MUTEX_TWO_PRIORITIES_HPP_

#include <mutex>

namespace rclcpp
{
namespace detail
{
/// \internal A mutex that has two locking mechanism, one with higher priority than the other.
/**
 * After the current mutex owner release the lock, a thread that used the high
 * priority mechanism will have priority over threads that used the low priority mechanism.
 */
class MutexTwoPriorities
{
public:
  class HighPriorityMutex
  {
public:
    explicit HighPriorityMutex(MutexTwoPriorities & parent)
    : parent_(parent) {}

    void lock()
    {
      parent_.data_.lock();
    }

    void unlock()
    {
      parent_.data_.unlock();
    }

private:
    MutexTwoPriorities & parent_;
  };

  class LowPriorityMutex
  {
public:
    explicit LowPriorityMutex(MutexTwoPriorities & parent)
    : parent_(parent) {}

    void lock()
    {
      std::unique_lock<std::mutex> barrier_guard{parent_.barrier_};
      parent_.data_.lock();
      barrier_guard.release();
    }

    void unlock()
    {
      // data_.unlock(); low_prio_.unlock()
      std::lock_guard<std::mutex> barrier_guard{parent_.barrier_, std::adopt_lock};
      parent_.data_.unlock();
    }

private:
    MutexTwoPriorities & parent_;
  };

  HighPriorityMutex get_high_priority_mutex() {return HighPriorityMutex{*this};}
  LowPriorityMutex get_low_priority_mutex() {return LowPriorityMutex{*this};}

private:
  // Implementation detail: the idea here is that only one low priority thread can be
  // trying to take the data_ mutex while the others are excluded by the barrier_ mutex.
  // All high priority threads are already waiting for the data_ mutex.
  std::mutex barrier_;
  std::mutex data_;
  };

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__MUTEX_TWO_PRIORITIES_HPP_
