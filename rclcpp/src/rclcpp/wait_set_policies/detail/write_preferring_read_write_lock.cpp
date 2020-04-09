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

#include "rclcpp/wait_set_policies/detail/write_preferring_read_write_lock.hpp"

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

WritePreferringReadWriteLock::WritePreferringReadWriteLock(
  std::function<void()> enter_waiting_function)
: read_mutex_(*this), write_mutex_(*this), enter_waiting_function_(enter_waiting_function)
{}

WritePreferringReadWriteLock::ReadMutex &
WritePreferringReadWriteLock::get_read_mutex()
{
  return read_mutex_;
}

WritePreferringReadWriteLock::WriteMutex &
WritePreferringReadWriteLock::get_write_mutex()
{
  return write_mutex_;
}

WritePreferringReadWriteLock::ReadMutex::ReadMutex(WritePreferringReadWriteLock & parent_lock)
: parent_lock_(parent_lock)
{}

void
WritePreferringReadWriteLock::ReadMutex::lock()
{
  std::unique_lock<std::mutex> lock(parent_lock_.mutex_);
  while (
    parent_lock_.number_of_writers_waiting_ > 0 ||
    parent_lock_.writer_active_ ||
    parent_lock_.reader_active_)
  {
    parent_lock_.condition_variable_.wait(lock);
  }
  parent_lock_.reader_active_ = true;
  // implicit unlock of parent_lock_.mutex_
}

void
WritePreferringReadWriteLock::ReadMutex::unlock()
{
  std::unique_lock<std::mutex> lock(parent_lock_.mutex_);
  parent_lock_.reader_active_ = false;
  parent_lock_.condition_variable_.notify_all();
  // implicit unlock of parent_lock_.mutex_
}

WritePreferringReadWriteLock::WriteMutex::WriteMutex(WritePreferringReadWriteLock & parent_lock)
: parent_lock_(parent_lock)
{}

void
WritePreferringReadWriteLock::WriteMutex::lock()
{
  std::unique_lock<std::mutex> lock(parent_lock_.mutex_);
  parent_lock_.number_of_writers_waiting_ += 1;
  if (nullptr != parent_lock_.enter_waiting_function_) {
    parent_lock_.enter_waiting_function_();
  }
  while (parent_lock_.reader_active_ || parent_lock_.writer_active_) {
    parent_lock_.condition_variable_.wait(lock);
  }
  parent_lock_.number_of_writers_waiting_ -= 1;
  parent_lock_.writer_active_ = true;
  // implicit unlock of parent_lock_.mutex_
}

void
WritePreferringReadWriteLock::WriteMutex::unlock()
{
  std::unique_lock<std::mutex> lock(parent_lock_.mutex_);
  parent_lock_.writer_active_ = false;
  parent_lock_.condition_variable_.notify_all();
  // implicit unlock of parent_lock_.mutex_
}

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp
