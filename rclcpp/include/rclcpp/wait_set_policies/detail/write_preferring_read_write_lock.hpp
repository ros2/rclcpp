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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__WRITE_PREFERRING_READ_WRITE_LOCK_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__WRITE_PREFERRING_READ_WRITE_LOCK_HPP_

#include <condition_variable>
#include <functional>
#include <mutex>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

/// Writer-perferring read-write lock.
/**
 * This class is based on an implementation of a "write-preferring RW lock" as described in this
 * wikipedia page:
 *
 * https://en.wikipedia.org/wiki/Readers%E2%80%93writer_lock#Using_a_condition_variable_and_a_mutex
 *
 * Copying here for posterity:
 *
 * \verbatim
 *   For a write-preferring RW lock one can use two integer counters and one boolean flag:
 *
 *       num_readers_active: the number of readers that have acquired the lock (integer)
 *       num_writers_waiting: the number of writers waiting for access (integer)
 *       writer_active: whether a writer has acquired the lock (boolean).
 *
 *   Initially num_readers_active and num_writers_waiting are zero and writer_active is false.
 *
 *   The lock and release operations can be implemented as
 *
 *   Begin Read
 *
 *       Lock g
 *       While num_writers_waiting > 0 or writer_active:
 *           wait cond, g[a]
 *       Increment num_readers_active
 *       Unlock g.
 *
 *   End Read
 *
 *       Lock g
 *       Decrement num_readers_active
 *       If num_readers_active = 0:
 *           Notify cond (broadcast)
 *       Unlock g.
 *
 *   Begin Write
 *
 *       Lock g
 *       Increment num_writers_waiting
 *       While num_readers_active > 0 or writer_active is true:
 *           wait cond, g
 *       Decrement num_writers_waiting
 *       Set writer_active to true
 *       Unlock g.
 *
 *   End Write
 *
 *       Lock g
 *       Set writer_active to false
 *       Notify cond (broadcast)
 *       Unlock g.
 * \endverbatim
 *
 * It will prefer any waiting write calls to any waiting read calls, meaning
 * that excessive write calls can starve read calls.
 *
 * This class diverges from that design in two important ways.
 * First, it is a single reader, single writer version.
 * Second, it allows for user defined code to be run after a writer enters the
 * waiting state, and the purpose of this feature is to allow the user to
 * interrupt any potentially long blocking read activities.
 *
 * Together these two features allow new waiting writers to not only ensure
 * they get the lock before any queued readers, but also that it can safely
 * interrupt read activities if needed, without allowing new read activities to
 * start before it gains the lock.
 *
 * The first difference prevents the case that a multiple read activities occur
 * at the same time but the writer can only reliably interrupt one of them.
 * By preventing multiple read activities concurrently, this case is avoided.
 * The second difference allows the user to define how to interrupt read
 * activity that could be blocking the write activities that need to happen
 * as soon as possible.
 *
 * To implement the differences, this class replaces the "num_readers_active"
 * counter with a "reader_active" boolean.
 * It also changes the "Begin Read" section from above, like this:
 *
 * \verbatim
 *   Begin Read
 *
 *       Lock g
 *       While num_writers_waiting > 0 or writer_active or reader_active:  // changed
 *           wait cond, g[a]
 *       Set reader_active to true  // changed
 *       Unlock g.
 * \endverbatim
 *
 * And changes the "End Read" section from above, like this:
 *
 * \verbatim
 *   End Read
 *
 *       Lock g
 *       Set reader_active to false  // changed
 *       Notify cond (broadcast)  // changed, now unconditional
 *       Unlock g.
 * \endverbatim
 *
 * The "Begin Write" section is also updated as follows:
 *
 * \verbatim
 *   Begin Write
 *
 *       Lock g
 *       Increment num_writers_waiting
 *       Call user defined enter_waiting function  // new
 *       While reader_active is true or writer_active is true:  // changed
 *           wait cond, g
 *       Decrement num_writers_waiting
 *       Set writer_active to true
 *       Unlock g.
 * \endverbatim
 *
 * The implementation uses a single condition variable, single lock, and several
 * state variables.
 *
 * The typical use of this class is as follows:
 *
 *     class MyClass
 *     {
 *       WritePreferringReadWriteLock wprw_lock_;
 *     public:
 *       MyClass() {}
 *       void do_some_reading()
 *       {
 *         using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
 *         std::lock_guard<WritePreferringReadWriteLock::ReadMutex> lock(wprw_lock_.get_read_mutex());
 *         // Do reading...
 *       }
 *       void do_some_writing()
 *       {
 *         using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
 *         std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
 *         // Do writing...
 *       }
 *     };
 */
class WritePreferringReadWriteLock final
{
public:
  RCLCPP_PUBLIC
  explicit WritePreferringReadWriteLock(std::function<void()> enter_waiting_function = nullptr);

  /// Read mutex for the WritePreferringReadWriteLock.
  /**
   * Implements the "C++ named requirements: BasicLockable".
   */
  class RCLCPP_PUBLIC ReadMutex
  {
public:
    void
    lock();

    void
    unlock();

protected:
    explicit ReadMutex(WritePreferringReadWriteLock & parent_lock);

    WritePreferringReadWriteLock & parent_lock_;

    friend WritePreferringReadWriteLock;
  };

  /// Write mutex for the WritePreferringReadWriteLock.
  /**
   * Implements the "C++ named requirements: BasicLockable".
   */
  class RCLCPP_PUBLIC WriteMutex
  {
public:
    void
    lock();

    void
    unlock();

protected:
    explicit WriteMutex(WritePreferringReadWriteLock & parent_lock);

    WritePreferringReadWriteLock & parent_lock_;

    friend WritePreferringReadWriteLock;
  };

  /// Return read mutex which can be used with standard constructs like std::lock_guard.
  RCLCPP_PUBLIC
  ReadMutex &
  get_read_mutex();

  /// Return write mutex which can be used with standard constructs like std::lock_guard.
  RCLCPP_PUBLIC
  WriteMutex &
  get_write_mutex();

protected:
  bool reader_active_ = false;
  std::size_t number_of_writers_waiting_ = 0;
  bool writer_active_ = false;
  std::mutex mutex_;
  std::condition_variable condition_variable_;
  ReadMutex read_mutex_;
  WriteMutex write_mutex_;
  std::function<void()> enter_waiting_function_;
};

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__WRITE_PREFERRING_READ_WRITE_LOCK_HPP_
