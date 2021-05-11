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

#ifndef RCLCPP__DETAIL__MUTEX_TWO_PRIORITIES_HPP_
#define RCLCPP__DETAIL__MUTEX_TWO_PRIORITIES_HPP_

#include <condition_variable>
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
  class HighPriorityLockable
  {
public:
    explicit HighPriorityLockable(MutexTwoPriorities & parent);

    void lock();

    void unlock();

private:
    MutexTwoPriorities & parent_;
  };

  class LowPriorityLockable
  {
public:
    explicit LowPriorityLockable(MutexTwoPriorities & parent);

    void lock();

    void unlock();

private:
    MutexTwoPriorities & parent_;
  };

  HighPriorityLockable
  get_high_priority_lockable();

  LowPriorityLockable
  get_low_priority_lockable();

private:
  std::condition_variable hp_cv_;
  std::condition_variable lp_cv_;
  std::mutex cv_mutex_;
  size_t hp_waiting_count_{0u};
  bool data_taken_{false};
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__MUTEX_TWO_PRIORITIES_HPP_
